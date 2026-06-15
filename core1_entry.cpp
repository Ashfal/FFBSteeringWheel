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
#include "motor_control.h"
#include "as5600_parser.h"
#include "ffb_processor.h"
#include "i2c_dma.h"
#include "calibration.h"
#include "debug_serial.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "pico/multicore.h"

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

// Transient EMI tolerance counter (file scope so both branches can access it)
static uint8_t g_magnet_error_count = 0;

void core1_set_shared_state(SharedState* state) {
    g_state = state;
}

// =========================================================================
// DMA Interrupt Handler — The actual FFB Loop
// =========================================================================
static void dma_isr() {
    if (!g_i2c.handle_isr()) return; // Not our interrupt

    uint32_t start_time = time_us_32();
    g_last_loop_time_us = time_us_64();
    const uint8_t* raw_data = g_i2c.get_data();

    uint8_t status = raw_data[0];
    uint16_t angle = (static_cast<uint16_t>(raw_data[1]) << 8) | raw_data[2];

    // 1. Parse Sensor
    bool valid = g_parser.update(status, angle);
    
    // Update shared state
    g_state->sensor.wheel_position.store(g_parser.get_position());
    g_state->sensor.wheel_velocity.store(g_parser.get_velocity());
    g_state->sensor.absolute_raw_angle.store(g_parser.get_absolute_raw());
    g_state->sensor.error_flags.store(g_parser.get_error_flags());

    if (!valid) {
        // Transient EMI tolerance: only kill the motor after N consecutive bad reads.
        // This lets the motor coast through 1-2ms glitches from its own magnetic field.
        g_magnet_error_count++;

        if (g_magnet_error_count >= MAGNET_ERROR_TOLERANCE_FRAMES) {
            // Persistent error — kill motor and set LED
            g_motor.stop();
            uint8_t err = g_parser.get_error_flags();
            if (err & SensorState::ERR_MAGNET_MISSING) {
                g_state->led_status.set(SystemStatus::MagnetMissing);
                debug_log_error(SystemStatus::MagnetMissing);
            } else if (err & SensorState::ERR_MAGNET_HIGH) {
                g_state->led_status.set(SystemStatus::MagnetHigh);
                debug_log_error(SystemStatus::MagnetHigh);
            } else if (err & SensorState::ERR_MAGNET_LOW) {
                g_state->led_status.set(SystemStatus::MagnetLow);
                debug_log_error(SystemStatus::MagnetLow);
            } else if (err & SensorState::ERR_DESYNC) {
                g_state->led_status.set(SystemStatus::EncoderDesync);
                debug_log_error(SystemStatus::EncoderDesync);
                alarm_pool_cancel_alarm(g_alarm_pool, g_timer_alarm);
            } else if (err & SensorState::ERR_RECOVERY_DESYNC) {
                g_state->led_status.set(SystemStatus::DesyncAfterRecovery);
                debug_log_error(SystemStatus::DesyncAfterRecovery);
                alarm_pool_cancel_alarm(g_alarm_pool, g_timer_alarm);
            }
        }
        // Below tolerance: motor holds its last commanded value, skip FFB processing
        return;
    }

    // Valid read — reset error counter
    g_magnet_error_count = 0;

    // Clear magnet errors if we recovered
    g_state->led_status.clear(SystemStatus::MagnetMissing);
    g_state->led_status.clear(SystemStatus::MagnetHigh);
    g_state->led_status.clear(SystemStatus::MagnetLow);

    // 2. FFB Processing
    FFBOutput out = g_ffb.calculate(g_parser.get_position(),
                                    g_parser.get_velocity(),
                                    g_state->ffb);

    // 3. Motor Control
    g_motor.set_force(out.force, g_parser.get_velocity());

    // 4. One-shot AGC register read (requested by debug serial on Core 0)
    if (g_state->request_agc_read.load()) {
        static auto* const I2C_PORT = i2c_get_instance(I2C_INSTANCE);
        uint8_t reg = 0x1A; // AS5600 AGC register
        uint8_t agc_val = 0;
        int ret = i2c_write_blocking(I2C_PORT, AS5600_I2C_ADDR, &reg, 1, true);
        if (ret == 1) {
            ret = i2c_read_blocking(I2C_PORT, AS5600_I2C_ADDR, &agc_val, 1, false);
            if (ret == 1) {
                g_state->agc_value.store(agc_val);
            }
        }
        g_state->request_agc_read.store(false);
    }

    uint32_t duration = time_us_32() - start_time;
    
    // Exponential Moving Average (alpha = 1/16)
    static uint32_t loop_time_ema_scaled = 0;
    if (loop_time_ema_scaled == 0) {
        loop_time_ema_scaled = duration << 4;
    } else {
        loop_time_ema_scaled = loop_time_ema_scaled - (loop_time_ema_scaled >> 4) + duration;
    }
    g_state->sensor.loop_time_avg_us.store(loop_time_ema_scaled >> 4, std::memory_order_relaxed);
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


void core1_init_interrupts() {
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

    // Initialize I2C, Motor, and Parser
    g_i2c.init();
    g_parser.init();
    g_motor.init();

    // Wait for boot or calibration command from Core 0
    uint32_t cal_cmd = multicore_fifo_pop_blocking();
    
    if (cal_cmd == 1) {
        // CMD_RUN_FLASH_CAL
        
        // Run the motor and encoder calibration routine (now handles center capture internally)
        run_calibration(g_state, g_i2c, g_motor, g_parser);
        
        // Acknowledge sweeps complete
        std::atomic_thread_fence(std::memory_order_release);
        multicore_fifo_push_blocking(1);
        
        // Core 1 MUST initialize as a lockout victim before halting,
        // so that Core 0 can safely use flash_safe_execute!
        multicore_lockout_victim_init();
        
        // Halt. Core 0 will handle pedal cal and reboot.
        while (true) {
            tight_loop_contents();
        }
    }
    
    // CMD_BOOT_NORMAL
    // Apply flash calibration center
    g_parser.set_center(g_state->center_offset.load());
    g_ffb.init(&g_state->cal_luts);
    
    // Apply friction compensation from LUTs
    g_motor.set_calibration_zero(g_state->cal_luts.cw_zero_pwm, g_state->cal_luts.ccw_zero_pwm);

    core1_init_interrupts();

    g_last_loop_time_us = time_us_64();

    // Start the 1ms repeating alarm
    g_timer_alarm = alarm_pool_add_alarm_in_us(g_alarm_pool, I2C_READ_INTERVAL_US, timer_callback, nullptr, true);

    // Start the very first read manually
    g_i2c.start_read();

    // Background Watchdog Loop
    bool in_error_state = false;
    uint32_t recovery_success_count = 0;
    while (true) {
        uint64_t now = time_us_64();
        if (now - g_last_loop_time_us > I2C_WATCHDOG_TIMEOUT_US) {
            // Watchdog fired! Loop stalled.
            if (!in_error_state) {
                // Only stop the motor and set the LED on the initial fault detection
                g_motor.stop();
                g_state->led_status.set(SystemStatus::I2CWatchdogFired);
                debug_log_error(SystemStatus::I2CWatchdogFired);
                in_error_state = true;
            }
            recovery_success_count = 0; // Reset success counter on any fault

            // Cancel the 1ms alarm to prevent start_read() firing mid-reset
            alarm_pool_cancel_alarm(g_alarm_pool, g_timer_alarm);

            // Aggressively attempt to reset the I2C bus and un-stick the slave
            g_i2c.reset_bus();

            // Re-arm the 1ms alarm now that the bus is reinitialized
            g_timer_alarm = alarm_pool_add_alarm_in_us(
                g_alarm_pool, I2C_READ_INTERVAL_US, timer_callback, nullptr, true);
            
            // Wait a moment for the next alarm to fire and attempt a read
            sleep_ms(5);
        } else {
            if (in_error_state) {
                recovery_success_count++;
                if (recovery_success_count > 50) { // 50 loops * 2ms = 100ms of stability
                    // We successfully recovered! The 1ms timer fired and ISR completed consistently.
                    g_state->led_status.clear(SystemStatus::I2CWatchdogFired);
                    in_error_state = false;
                }
            }
        }

        // Just yield and let interrupts do the work
        sleep_ms(2);
    }
}
