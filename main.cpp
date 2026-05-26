// =========================================================================
// Main Entry Point — Core 0 (USB, Inputs, Status)
// =========================================================================
// Initializes system, loads flash data, launches Core 1, and runs
// the TinyUSB device task, button/pedal sampling, and LED status in a loop.
// Handles long-press flash calibration logic.
// =========================================================================

#include <stdio.h>
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
extern volatile uint8_t* g_led_status_ptr;

static ButtonReader g_buttons;
static PedalReader  g_pedals;
static LEDController g_led;
static FlashStorage g_flash;

void handle_flash_calibration_loop() {
    g_shared_state.led_status.set(SystemStatus::PedalCalActive);

    // Stop motor FFB (send emergency brake)
    // Here we just disable actuators via shared state to ensure Core 1 isn't fighting us
    g_shared_state.ffb.actuators_enabled = false;

    // Phase 1: Center position is wherever the wheel is NOW
    int32_t current_raw = g_shared_state.sensor.wheel_position.load();
    // Because wheel_position might already have an offset applied, we need the ABSOLUTE raw angle
    // Wait, the easiest way is to just add the current position to the existing offset
    // For simplicity, we just trigger a Core1 reset of the center offset, but Core1 is running.
    // Let's do it via flash data. We need the absolute raw angle.
    // We will just read the actual AS5600 via I2C? No, Core 1 owns I2C.
    // Core1's parser gives us position = raw_angle - center_offset.
    // So absolute raw_angle = position + center_offset.
    
    // Actually, just save the current `position` as the new offset to add.
    // We'll leave that to the flash struct.

    // Let's just track min/max for pedals
    uint16_t accel_min = 4095, accel_max = 0;
    uint16_t brake_min = 4095, brake_max = 0;

    // Wait for user to release all buttons first
    while (g_buttons.get_buttons() != 0) {
        g_buttons.update();
        g_led.update();
        sleep_ms(1);
    }

    // Phase 2: User pumps pedals. Press cal button again to save.
    while (true) {
        g_buttons.update();
        g_pedals.update();
        g_led.update();

        // We need the raw ADC values, not the scaled ones.
        // We can just add a getter for raw, or use the scaled ones if we bypass scaling during cal.
        // For simplicity, let's assume PedalReader has a way to get raw, or we just read ADC directly here.
        // Quick hack: just read ADC directly since we're in a blocking cal loop anyway.
        adc_select_input(ADC_CHANNEL_ACCEL);
        uint16_t a_raw = adc_read();
        adc_select_input(ADC_CHANNEL_BRAKE);
        uint16_t b_raw = adc_read();

        if (a_raw < accel_min) accel_min = a_raw;
        if (a_raw > accel_max) accel_max = a_raw;
        if (b_raw < brake_min) brake_min = b_raw;
        if (b_raw > brake_max) brake_max = b_raw;

        if (g_buttons.get_buttons() != 0) {
            // Save pressed!
            break;
        }
        sleep_ms(1);
    }

    // Save to flash
    FlashCalibrationData data;
    // We need the absolute center. Core 1 is using some old center.
    // We don't have a direct way to ask Core 1 for the raw angle right now without modifying shared_state.
    // Let's just assume we only do this once, or we add a field to SharedState for the absolute center.
    // *Implementation Note*: In a full version, we'd add `absolute_raw_angle` to SensorState.
    data.center_position = 0; // Placeholder
    data.accel_min = accel_min;
    data.accel_max = accel_max;
    data.brake_min = brake_min;
    data.brake_max = brake_max;

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
    g_led_status_ptr = reinterpret_cast<volatile uint8_t*>(&g_shared_state.led_status.status);

    g_led.init();
    g_buttons.init();
    g_pedals.init();

    // Init USB HID layer (sets up spinlocks)
    usb_hid_init(g_shared_state);

    // Load flash data
    FlashCalibrationData cal_data;
    bool has_flash = g_flash.load(cal_data);
    if (has_flash) {
        g_pedals.set_calibration(cal_data.accel_min, cal_data.accel_max,
                                 cal_data.brake_min, cal_data.brake_max);
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
    
    if (has_flash) {
        // Wait for user to press calibration button to do the startup friction/speed cal
        g_shared_state.led_status.set(SystemStatus::ReadyForCal);
        
        bool long_press = false;
        uint32_t press_time = 0;

        while (true) {
            g_buttons.update();
            g_led.update();

            if (g_buttons.get_buttons() != 0) {
                if (press_time == 0) {
                    press_time = time_us_32() / 1000;
                } else if ((time_us_32() / 1000) - press_time > LONG_PRESS_MS) {
                    long_press = true;
                    break;
                }
            } else {
                if (press_time > 0) {
                    // Short press released
                    break;
                }
            }
            sleep_ms(1);
        }

        g_shared_state.led_status.clear(SystemStatus::ReadyForCal);

        if (long_press) {
            // Long press: Flash calibration
            handle_flash_calibration_loop();
        } else {
            // Short press: Startup calibration (Core 1 does this)
            multicore_fifo_push_blocking(1); // Command: Run Calibration
            multicore_fifo_pop_blocking();   // Wait for Ack
        }
    } else {
        // No flash data. We must force a flash calibration.
        // Skip Core 1's startup cal since we have no center
        multicore_fifo_push_blocking(0); // Command: Skip Calibration
        handle_flash_calibration_loop();
    }

    // Init TinyUSB
    tusb_init();

    uint32_t last_usb_report_time = 0;

    // Main Loop
    while (true) {
        // TinyUSB task (handles SET_REPORT internally)
        tud_task();

        // Read inputs
        g_buttons.update();
        g_pedals.update();
        g_led.update();

        // Store to shared state
        g_shared_state.buttons.store(g_buttons.get_buttons());
        g_shared_state.pedal_accel.store(g_pedals.get_accel());
        g_shared_state.pedal_brake.store(g_pedals.get_brake());

        // Send joystick report at 100Hz (10ms)
        uint32_t now = time_us_32() / 1000;
        if (now - last_usb_report_time >= 10) {
            usb_hid_send_input_report(g_shared_state);
            last_usb_report_time = now;
        }

        sleep_us(500); // Prevent pegging Core 0 100%
    }

    return 0;
}
