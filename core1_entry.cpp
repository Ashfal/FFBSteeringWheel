// =========================================================================
// Core 1 Main Loop — Hard Real-Time FFB & Motor Control
// =========================================================================
// This core handles the 1ms strict timing loop.
// 1. Hardware alarm fires every 1ms, starting I2C DMA read of AS5600.
// 2. DMA completion IRQ fires when 3 bytes are read.
// 3. IRQ handler parses AS5600, runs FFB processor, and updates motor PWM.
// 4. Hardware watchdog ensures motor is killed if the loop stalls.
// =========================================================================

#include "core1_entry.h"
#include "config.h"
#include "shared_state.h"
#include "i2c_dma.h"
#include "as5600_parser.h"
#include "ffb_processor.h"
#include "motor_control.h"
#include "calibration.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/dma.h"

static SharedState* g_state = nullptr;

static I2CDMA       g_i2c;
static AS5600Parser g_parser;
static FFBProcessor g_ffb;
static MotorControl g_motor;

// Alarm pool and ID for the 1ms trigger
static alarm_pool_t* g_alarm_pool;
static alarm_id_t    g_timer_alarm;

// Watchdog tracking
static volatile uint64_t g_last_loop_time_us = 0;

void core1_set_shared_state(SharedState* state) {
    g_state = state;
}

// =========================================================================
// DMA Interrupt Handler — The actual FFB Loop
// =========================================================================
static void dma_isr() {
    if (!g_i2c.handle_isr()) return; // Not our interrupt

    g_last_loop_time_us = time_us_64();
    const uint8_t* raw_data = g_i2c.get_data();

    uint8_t status = raw_data[0];
    uint16_t angle = (static_cast<uint16_t>(raw_data[1]) << 8) | raw_data[2];

    // 1. Parse Sensor
    bool valid = g_parser.update(status, angle);
    
    // Update shared state
    g_state->sensor.wheel_position.store(g_parser.get_position());
    g_state->sensor.wheel_velocity.store(g_parser.get_velocity());
    g_state->sensor.error_flags.store(g_parser.get_error_flags());

    if (!valid) {
        // Hardware error (magnet missing/bad)
        g_motor.emergency_stop();
        // Determine exact error for LED status
        uint8_t err = g_parser.get_error_flags();
        if (err & SensorState::ERR_MAGNET_MISSING) {
            g_state->led_status.set(SystemStatus::MagnetMissing);
        } else if (err & SensorState::ERR_MAGNET_HIGH) {
            g_state->led_status.set(SystemStatus::MagnetHigh);
        } else if (err & SensorState::ERR_MAGNET_LOW) {
            g_state->led_status.set(SystemStatus::MagnetLow);
        }
        return; // Skip FFB processing
    }

    // Clear magnet errors if we recovered
    g_state->led_status.clear(SystemStatus::MagnetMissing);
    g_state->led_status.clear(SystemStatus::MagnetHigh);
    g_state->led_status.clear(SystemStatus::MagnetLow);

    // 2. FFB Processing
    uint32_t irq_status = spin_lock_blocking(g_state->ffb.lock);
    FFBOutput out = g_ffb.calculate(g_parser.get_position(),
                                    g_parser.get_velocity(),
                                    g_state->ffb);
    spin_unlock(g_state->ffb.lock, irq_status);

    // 3. Motor Control
    g_motor.set_target(out.pwm, out.direction, g_parser.get_velocity());
}

// =========================================================================
// 1ms Timer Alarm Handler
// =========================================================================
static int64_t timer_callback(alarm_id_t id, void *user_data) {
    (void)id;
    (void)user_data;
    // Trigger the next I2C read
    g_i2c.start_read();
    
    // Return positive interval to reschedule automatically in microseconds
    return I2C_READ_INTERVAL_US;
}

// =========================================================================
// Core 1 Initialization
// =========================================================================
void core1_init() {
    g_i2c.init();
    g_parser.init();
    g_motor.init();

    // Enable DMA interrupt on Core 1
    irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // Create alarm pool for Core 1
    g_alarm_pool = alarm_pool_create(1, 16);
}

// =========================================================================
// Core 1 Main Entry
// =========================================================================
void core1_main() {
    if (!g_state) return;

    g_ffb.init(&g_state->cal_luts);

    // Wait for calibration command from Core 0
    // We block here until main loop tells us to proceed
    uint32_t cal_cmd = multicore_fifo_pop_blocking();
    
    if (cal_cmd == 1) {
        // Run blocking calibration routines
        // (This function will temporarily take over the motor and sensor)
        run_calibration(g_state, g_i2c, g_motor, g_parser);
        
        // Acknowledge calibration complete
        multicore_fifo_push_blocking(1);
    }

    // Apply flash calibration center
    // g_parser.set_center(...) should be called from flash data, handled in main/calibration.
    
    // Apply friction compensation from LUTs
    g_motor.set_calibration_zero(g_state->cal_luts.cw_zero_pwm, g_state->cal_luts.ccw_zero_pwm);

    g_last_loop_time_us = time_us_64();

    // Start the 1ms repeating alarm
    g_timer_alarm = alarm_pool_add_alarm_in_us(g_alarm_pool, I2C_READ_INTERVAL_US, timer_callback, nullptr, true);

    // Start the very first read manually
    g_i2c.start_read();

    // Background Watchdog Loop
    while (true) {
        uint64_t now = time_us_64();
        if (now - g_last_loop_time_us > I2C_WATCHDOG_TIMEOUT_US) {
            // Watchdog fired! Loop stalled.
            g_motor.emergency_stop();
            g_state->led_status.set(SystemStatus::I2CWatchdogFired);
        } else {
            g_state->led_status.clear(SystemStatus::I2CWatchdogFired);
        }

        // Just yield and let interrupts do the work
        tight_loop_contents();
    }
}
