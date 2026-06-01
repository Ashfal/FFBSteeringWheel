// =========================================================================
// Main Entry Point — Core 0 (USB, Inputs, Status)
// =========================================================================
// Initializes system, loads flash data, launches Core 1, and runs
// the TinyUSB device task, button/pedal sampling, and LED status in a loop.
// Handles long-press flash calibration logic.
// =========================================================================

#include <stdio.h>
#include <atomic>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "bsp/board_api.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"
#include "tusb.h"

#include "config.h"
#include "shared_state.h"
#include "core1_entry.h"
#include "usb_hid.h"
#include "button_reader.h"
#include "pedal_reader.h"
#include "led_controller.h"
#include "flash_storage.h"

// Instantiate the global shared state
SharedState g_shared_state;

// Link LED status to the global controller pointer
extern std::atomic<uint8_t>* g_led_status_ptr;

static ButtonReader g_buttons;
static PedalReader  g_pedals;
static LEDController g_led;
static FlashStorage g_flash;

void handle_flash_calibration_loop() {
    // 1. Tell Core 1 to run motor sweeps and find center
    g_shared_state.led_status.set(SystemStatus::MotorSweepsActive);
    multicore_fifo_push_blocking(1); // CMD_RUN_FLASH_CAL
    multicore_fifo_pop_blocking();   // Wait for Ack (sweeps finished)
    g_shared_state.led_status.clear(SystemStatus::MotorSweepsActive);

    // 2. Pedal Calibration Phase
    g_shared_state.led_status.set(SystemStatus::PedalCalActive);

    // Let's just track min/max for pedals
    uint16_t accel_min = 4095, accel_max = 0;
    uint16_t brake_min = 4095, brake_max = 0;

    // Wait for user to release all buttons first
    while (g_buttons.get_buttons() != 0) {
        g_buttons.update();
        g_led.update();
        sleep_ms(1);
    }

    // Phase 2: User pumps pedals. Long press cal button again to save.
    uint64_t press_time_us = 0;
    bool save_triggered = false;
    
    while (!save_triggered) {
        g_buttons.update();
        g_pedals.update();
        g_led.update();

        // Read raw compensated values so calibration matches regular operation.
        uint16_t a_raw = 0;
        uint16_t b_raw = 0;
        g_pedals.read_raw_compensated(a_raw, b_raw);

        if (a_raw < accel_min) accel_min = a_raw;
        if (a_raw > accel_max) accel_max = a_raw;
        if (b_raw < brake_min) brake_min = b_raw;
        if (b_raw > brake_max) brake_max = b_raw;

        if (g_buttons.get_buttons() != 0) {
            if (press_time_us == 0) {
                press_time_us = time_us_64();
            } else if ((time_us_64() - press_time_us) / 1000 > LONG_PRESS_MS) {
                save_triggered = true;
            }
        } else {
            press_time_us = 0;
        }
        sleep_ms(1);
    }

    // Save everything to flash
    FlashCalibrationData data;
    data.magic = 0xFEEDFACE;
    data.version = 1;
    data.center_position = g_shared_state.center_offset.load();
    data.accel_min = accel_min;
    data.accel_max = accel_max;
    data.brake_min = brake_min;
    data.brake_max = brake_max;
    
    data.cw_zero_pwm = g_shared_state.cal_luts.cw_zero_pwm;
    data.ccw_zero_pwm = g_shared_state.cal_luts.ccw_zero_pwm;
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        data.cw_speed[i] = g_shared_state.cal_luts.cw_speed[i];
        data.ccw_speed[i] = g_shared_state.cal_luts.ccw_speed[i];
    }

    g_flash.save(data);

    g_shared_state.led_status.clear(SystemStatus::PedalCalActive);

    // Reboot to apply new flash settings cleanly
    watchdog_enable(1, 1);
    while(1);
}


int main() {
    board_init();
    stdio_init_all();

    // Link LED status
    g_led_status_ptr = &g_shared_state.led_status.status;

    g_led.init();
    g_buttons.init();
    g_pedals.init();

    // Init USB HID layer (sets up spinlocks)
    usb_hid_init(g_shared_state);

    // Load flash data
    FlashCalibrationData cal_data;
    bool has_flash = g_flash.load(cal_data);
    if (has_flash) {
        g_shared_state.center_offset.store(cal_data.center_position);
        g_pedals.set_calibration(cal_data.accel_min, cal_data.accel_max,
                                 cal_data.brake_min, cal_data.brake_max);
                                 
        g_shared_state.cal_luts.cw_zero_pwm = cal_data.cw_zero_pwm;
        g_shared_state.cal_luts.ccw_zero_pwm = cal_data.ccw_zero_pwm;
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
            g_shared_state.cal_luts.cw_speed[i] = cal_data.cw_speed[i];
            g_shared_state.cal_luts.ccw_speed[i] = cal_data.ccw_speed[i];
        }
        g_shared_state.cal_luts.valid = true;
    } else {
        g_shared_state.led_status.set(SystemStatus::FlashCalMissing);
        // Provide safe defaults
        g_pedals.set_calibration(100, 4000, 100, 4000);
    }

    // Pass shared state to Core 1
    core1_set_shared_state(&g_shared_state);

    // Launch Core 1
    multicore_launch_core1(core1_main);

    // Wait for Core 1 to be ready for the calibration command
    // (Core 1 blocks on FIFO immediately)
    
    // Wait for user to press button to boot normally or start flash cal
    if (has_flash) {
        g_shared_state.led_status.set(SystemStatus::BootWait);
    } else {
        g_shared_state.led_status.set(SystemStatus::FlashCalMissing);
    }
    
    bool long_press = false;
    uint64_t press_time_us = 0;

    while (true) {
        g_buttons.update();
        g_led.update();

        if (g_buttons.get_buttons() != 0) {
            if (press_time_us == 0) {
                press_time_us = time_us_64();
            } else if ((time_us_64() - press_time_us) / 1000 > LONG_PRESS_MS) {
                long_press = true;
                break;
            }
        } else {
            if (press_time_us > 0) {
                // Short press released
                break;
            }
        }
        sleep_ms(1);
    }

    g_shared_state.led_status.clear(SystemStatus::BootWait);
    g_shared_state.led_status.clear(SystemStatus::FlashCalMissing);

    if (long_press || !has_flash) {
        // Long press OR no flash data: force Flash calibration
        handle_flash_calibration_loop();
    } else {
        // Short press: Normal Boot
        multicore_fifo_push_blocking(0); // CMD_BOOT_NORMAL
        // We do NOT wait for Ack on Boot Normal, Core 1 just starts running immediately.
    }

    // Init TinyUSB
    tusb_init();

    uint64_t last_usb_report_time_us = 0;
    uint64_t last_pedal_read_time_us = 0;
    uint64_t last_button_read_time_us = 0;

    // Main Loop
    while (true) {
        // TinyUSB task (handles incoming FFB packets and USB control)
        // This must run frequently, ideally >1000Hz (every <1ms)
        tud_task();

        uint64_t now_us = time_us_64();

        // Read pedals at PEDAL_UPDATE_INTERVAL_US (0.5ms)
        if (now_us - last_pedal_read_time_us >= PEDAL_UPDATE_INTERVAL_US) {
            g_pedals.update();
            g_shared_state.pedal_accel.store(g_pedals.get_accel());
            g_shared_state.pedal_brake.store(g_pedals.get_brake());
            last_pedal_read_time_us = now_us;
        }

        // Read buttons at BUTTON_UPDATE_INTERVAL_US (2ms)
        if (now_us - last_button_read_time_us >= BUTTON_UPDATE_INTERVAL_US) {
            g_buttons.update();
            g_shared_state.buttons.store(g_buttons.get_buttons());
            last_button_read_time_us = now_us;
        }

        // Send joystick report at 100Hz (10ms)
        if (now_us - last_usb_report_time_us >= 10000) {
            // Update LED at the same low frequency as the HID report
            g_led.update();
            
            usb_hid_send_input_report(g_shared_state);
            last_usb_report_time_us = now_us;
        }

        // Prevent pegging Core 0, but keep loop fast enough for USB 1000Hz polling
        sleep_us(250);
    }

    return 0;
}
