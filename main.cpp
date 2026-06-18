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
#include "pico/bootrom.h"
#include "tusb.h"

#include "config.h"
#include "shared_state.h"
#include "core1_entry.h"
#include "usb_hid.h"
#include "button_reader.h"
#include "pedal_reader.h"
#include "led_controller.h"
#include "flash_storage.h"
#include "debug_serial.h"
#include "calibration.h"

// Instantiate the global shared state
SharedState g_shared_state;

// Link LED status to the global controller pointer
extern StatusState* g_led_status_state_ptr;

static ButtonReader g_buttons;
static PedalReader  g_pedals;
static LEDController g_led;
static FlashStorage g_flash;


int main() {
    board_init();
    stdio_init_all();

    // Link LED status
    g_led_status_state_ptr = &g_shared_state.led_status;

    g_led.init();
    g_buttons.init();
    g_pedals.init();

    // Init USB HID layer (sets up spinlocks)
    usb_hid_init(g_shared_state);
    
    // Init Debug Serial console
    debug_serial_init(g_shared_state);

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
        debug_log_error(SystemStatus::FlashCalMissing);
        // Provide safe defaults
        g_pedals.set_calibration(100, 4000, 100, 4000);
    }

    // Wait for user to press button to boot normally or start flash cal
    if (has_flash) {
        g_shared_state.led_status.set(SystemStatus::BootWait);
    } else {
        g_shared_state.led_status.set(SystemStatus::FlashCalMissing);
        debug_log_error(SystemStatus::FlashCalMissing);
    }
    
    bool long_press = false;
    bool bootloader_mode = false;
    uint64_t press_time_us = 0;
    int max_buttons_pressed = 0;

    while (true) {
        g_buttons.update();
        g_led.update();

        uint16_t buttons_state = g_buttons.get_buttons();
        if (buttons_state != 0) {
            if (press_time_us == 0) {
                press_time_us = time_us_64();
                max_buttons_pressed = 0;
            }
            int num_pressed = __builtin_popcount(buttons_state);
            if (num_pressed > max_buttons_pressed) {
                max_buttons_pressed = num_pressed;
            }
            
            if ((time_us_64() - press_time_us) / 1000 > LONG_PRESS_MS) {
                if (max_buttons_pressed >= 2) {
                    bootloader_mode = true;
                } else {
                    long_press = true;
                }
                break;
            }
        } else {
            if (press_time_us > 0) {
                // Short press released
                break;
            }
        }
        sleep_us(BUTTON_UPDATE_INTERVAL_US);
    }

    g_shared_state.led_status.clear(SystemStatus::BootWait);
    g_shared_state.led_status.clear(SystemStatus::FlashCalMissing);

    if (bootloader_mode) {
        // Flash LED quickly to indicate bootloader transition
        for (int i = 0; i < 10; i++) {
            gpio_put(PIN_LED, 1);
            sleep_ms(50);
            gpio_put(PIN_LED, 0);
            sleep_ms(50);
        }
        reset_usb_boot(1u << PIN_LED, 0);
        while (true) {
            tight_loop_contents();
        }
    }

    if (long_press || !has_flash) {
        // Long press OR no flash data: force Flash calibration on Core 0
        run_calibration(g_shared_state, g_buttons, g_pedals, g_led, g_flash);
        // run_calibration handles reboot, so this never returns
    } else {
        // Short press: Normal Boot
        // Pass shared state to Core 1 and launch it
        core1_set_shared_state(g_shared_state);
        multicore_launch_core1(core1_main);
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
        
        // Debug serial commands
        debug_serial_update();

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
