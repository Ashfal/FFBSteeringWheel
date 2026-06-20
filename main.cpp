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
static SharedState g_shared_state;

static ButtonReader g_buttons;
static PedalReader  g_pedals;
static LEDController g_led;
static FlashStorage g_flash;

int main() {
    board_init();
    stdio_init_all();

    g_led.init(g_shared_state);
    g_buttons.init();
    g_pedals.init();

    // Init USB HID layer (sets up spinlocks)
    usb_hid_init(g_shared_state);
    
    // Init Debug Serial console
    debug_serial_init(g_shared_state, g_pedals, g_flash);

    // Load flash data
    FlashCalibrationData cal_data;
    bool has_flash = g_flash.load(cal_data);

    if (has_flash) {
        g_shared_state.center_offset.store(cal_data.center_position);
        g_pedals.set_calibration(cal_data.accel_min, cal_data.accel_max,
                                 cal_data.brake_min, cal_data.brake_max);
                                 
        int32_t half_angle_deg = cal_data.wheel_angle_deg / 2;
        int32_t max_half_angle_counts = (half_angle_deg * WHEEL_COUNTS_PER_REV) / 360;
        g_shared_state.max_half_angle_counts.store(max_half_angle_counts);
        g_shared_state.wheel_angle_deg.store(cal_data.wheel_angle_deg);
                                 
        g_shared_state.cal_luts.cw_zero_pwm = cal_data.cw_zero_pwm;
        g_shared_state.cal_luts.ccw_zero_pwm = cal_data.ccw_zero_pwm;
        for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
            g_shared_state.cal_luts.cw_speed[i] = cal_data.cw_speed[i];
            g_shared_state.cal_luts.ccw_speed[i] = cal_data.ccw_speed[i];
        }
        g_shared_state.cal_luts.valid = true;

        g_shared_state.led_status.set(SystemStatus::BootWait);
    } else {
        // Provide safe defaults
        g_pedals.set_calibration(100, 4000, 100, 4000);
        
        int32_t half_angle_deg = DEFAULT_MAX_WHEEL_ANGLE_DEG / 2;
        int32_t max_half_angle_counts = (half_angle_deg * WHEEL_COUNTS_PER_REV) / 360;
        g_shared_state.max_half_angle_counts.store(max_half_angle_counts);
        g_shared_state.wheel_angle_deg.store(DEFAULT_MAX_WHEEL_ANGLE_DEG);

        g_shared_state.led_status.set(SystemStatus::FlashCalMissing);
        debug_log_error(SystemStatus::FlashCalMissing);
    }
    
    // Preload true button state
    for (int i = 0; i < DEBOUNCE_READS + 1; i++) {
        g_buttons.update();
        sleep_us(BUTTON_UPDATE_INTERVAL_US);
    }

    // Wait for user to release all buttons first
    // We flash the LED to indicate that there is a button pressed or stuck
    if (g_buttons.get_buttons() != 0) {
        g_shared_state.led_status.set(SystemStatus::RapidFlash);
        while (g_buttons.get_buttons() != 0) {
            g_buttons.update();
            g_led.update();
            sleep_us(BUTTON_UPDATE_INTERVAL_US);
        }
        g_shared_state.led_status.clear(SystemStatus::RapidFlash);
    }

    uint64_t press_time_us = 0;
    bool long_press = false;
    int num_pressed = 0;

    while (true) {
        g_buttons.update();
        g_led.update();

        uint16_t buttons_state = g_buttons.get_buttons();
        if (buttons_state != 0) {
            if (press_time_us == 0) {
                press_time_us = time_us_64();
            }
            else if ((time_us_64() - press_time_us) / 1000 > LONG_PRESS_MS) {
                long_press = true;
                num_pressed = __builtin_popcount(buttons_state);
                break;
            }
        } 
        else if (press_time_us > 0) { // Short press released
            if (has_flash) break;
            else press_time_us = 0;
        }
        sleep_us(BUTTON_UPDATE_INTERVAL_US);
    }

    g_shared_state.led_status.clear(SystemStatus::BootWait);
    g_shared_state.led_status.clear(SystemStatus::FlashCalMissing);

    if (long_press) {
        if (num_pressed == 2) { //Bootloader Mode
            // Flash LED quickly to indicate bootloader transition
            g_shared_state.led_status.set(SystemStatus::RapidFlash);
            g_led.sleep_ms(1000);
            reset_usb_boot(1u << PIN_LED, 0);
            while (true) {
                tight_loop_contents();
            }
        }
        else if (num_pressed == 1) {
            run_calibration(g_shared_state, g_buttons, g_pedals, g_led, g_flash);
            // run_calibration handles reboot, so this never returns
        }
    }

    // Pass shared state to Core 1 and launch it
    core1_set_shared_state(g_shared_state);
    multicore_launch_core1(core1_main);

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
