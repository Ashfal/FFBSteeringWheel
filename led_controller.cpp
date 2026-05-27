// =========================================================================
// LED Status Controller — Non-blocking flash code display
// =========================================================================
// Solid ON during normal operation.
// Flash codes: N blinks (200ms ON, 500ms OFF), then 2s pause.
// Priority: highest SystemStatus enum value wins.
// =========================================================================

#include <atomic>
#include "led_controller.h"
#include "hardware/gpio.h"
#include "pico/time.h"

// External access to the shared LED status
// (linked from main.cpp's SharedState)
extern std::atomic<uint8_t>* g_led_status_ptr;

void LEDController::init() {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);  // Start with LED on (normal)
    phase_ = FlashPhase::IDLE;
}

void LEDController::update() {
    // Read current status code from the atomic
    uint8_t code = 0;
    if (g_led_status_ptr) {
        code = g_led_status_ptr->load();
    }

    uint64_t now = time_us_64();

    // If status is Normal (0), keep LED solid on
    if (code == 0) {
        gpio_put(PIN_LED, 1);
        phase_ = FlashPhase::IDLE;
        current_code_ = 0;
        return;
    }

    // If the code changed, restart the flash sequence
    if (code != current_code_) {
        current_code_ = code;
        flash_count_ = code;
        flashes_done_ = 0;
        phase_ = FlashPhase::FLASH_ON;
        phase_start_us_ = now;
        gpio_put(PIN_LED, 1);
        return;
    }

    // State machine for flashing
    uint64_t elapsed_us = now - phase_start_us_;

    switch (phase_) {
        case FlashPhase::IDLE:
            // Shouldn't get here if code != 0, but start fresh
            phase_ = FlashPhase::FLASH_ON;
            phase_start_us_ = now;
            flashes_done_ = 0;
            gpio_put(PIN_LED, 1);
            break;

        case FlashPhase::FLASH_ON:
            if (elapsed_us >= LED_FLASH_ON_MS * 1000ULL) {
                gpio_put(PIN_LED, 0);
                flashes_done_++;
                if (flashes_done_ >= flash_count_) {
                    // All flashes done, enter pause
                    phase_ = FlashPhase::PAUSE;
                } else {
                    phase_ = FlashPhase::FLASH_OFF;
                }
                phase_start_us_ = now;
            }
            break;

        case FlashPhase::FLASH_OFF:
            if (elapsed_us >= LED_FLASH_OFF_MS * 1000ULL) {
                gpio_put(PIN_LED, 1);
                phase_ = FlashPhase::FLASH_ON;
                phase_start_us_ = now;
            }
            break;

        case FlashPhase::PAUSE:
            if (elapsed_us >= LED_PAUSE_MS * 1000ULL) {
                // Restart the flash sequence
                flashes_done_ = 0;
                phase_ = FlashPhase::FLASH_ON;
                phase_start_us_ = now;
                gpio_put(PIN_LED, 1);
            }
            break;
    }
}

// Global pointer — set by main.cpp
std::atomic<uint8_t>* g_led_status_ptr = nullptr;
