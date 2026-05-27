// =========================================================================
// Calibration Routine
// =========================================================================
// Sweeps the motor to find static friction limits and populate speed LUTs.
// Runs synchronously on Core 1 before the normal DMA loop starts.
// =========================================================================

#include "calibration.h"
#include "config.h"
#include "pico/time.h"

// Helper to do a blocking I2C read during calibration
static bool block_read_sensor(I2CDMA& i2c, AS5600Parser& parser) {
    i2c.start_read();
    // Wait for ISR flag
    while (!i2c.handle_isr()) {
        tight_loop_contents();
    }
    
    const uint8_t* raw = i2c.get_data();
    uint8_t status = raw[0];
    uint16_t angle = (static_cast<uint16_t>(raw[1]) << 8) | raw[2];
    return parser.update(status, angle);
}

// Find minimum PWM to overcome static friction (velocity > 0)
static uint16_t find_zero_pwm(MotorControl::Direction dir, MotorControl& motor, I2CDMA& i2c, AS5600Parser& parser) {
    uint16_t test_pwm = 0;
    
    // Ensure we are stopped
    motor.stop();
    sleep_ms(200);

    while (test_pwm < PWM_WRAP) {
        test_pwm += 10;
        // Pass FORWARD_VELOCITY_THRESHOLD to bypass the stall governor here, 
        // since we actively NEED to exceed it to measure static friction.
        int32_t fake_vel = (dir == MotorControl::Direction::CW) ? FORWARD_VELOCITY_THRESHOLD : -FORWARD_VELOCITY_THRESHOLD;
        motor.set_pwm(test_pwm, dir, fake_vel);
        
        // Wait briefly for movement
        sleep_ms(10);
        
        block_read_sensor(i2c, parser);
        int32_t vel = parser.get_velocity();
        
        // If we consistently have velocity in the right direction, we found it
        if ((dir == MotorControl::Direction::CW && vel > 10) || (dir == MotorControl::Direction::CCW && vel < -10)) {
            // Confirm with a second reading
            sleep_ms(10);
            block_read_sensor(i2c, parser);
            vel = parser.get_velocity();
            if ((dir == MotorControl::Direction::CW && vel > 10) || (dir == MotorControl::Direction::CCW && vel < -10)) {
                break;
            }
        }
    }
    
    motor.stop();
    sleep_ms(100);
    return test_pwm;
}

// Sweep at a constant force and find maximum stable speed
static int32_t measure_max_speed(int32_t force, MotorControl& motor, I2CDMA& i2c, AS5600Parser& parser) {
    motor.set_force(force, 0);
    
    bool is_cw = force > 0;
    
    int32_t max_vel = 0;
    uint32_t samples = 0;
    
    // Sweep for up to 500ms or until we travel MIN_SWEEP_COUNTS
    int32_t start_pos = parser.get_position();
    uint64_t start_time = time_us_64();
    
    while (true) {
        sleep_ms(2); // ~500Hz sampling
        if (!block_read_sensor(i2c, parser)) continue;
        
        int32_t vel = parser.get_velocity();
        int32_t pos = parser.get_position();
        
        // Update the motor target so the stall governor releases its clamp as we speed up
        motor.set_force(force, vel);
        
        // Track absolute maximum velocity
        if (is_cw && vel > max_vel) max_vel = vel;
        if (!is_cw && vel < max_vel) max_vel = vel; // Note: max_vel will be negative
        
        samples++;
        
        // Stop conditions
        uint64_t elapsed = time_us_64() - start_time;
        int32_t distance = pos - start_pos;
        if (!is_cw) distance = -distance;
        
        if (elapsed > 500000) break; // 500ms timeout
        if (distance > CAL_MIN_SWEEP_COUNTS) break; // Travelled enough distance
        
        // Safety check: end stop
        if (pos > MAX_HALF_ANGLE_COUNTS - 1000 || pos < -MAX_HALF_ANGLE_COUNTS + 1000) {
            break;
        }
    }
    
    motor.stop();
    sleep_ms(100); // Wait for wheel to settle
    
    // Convert to absolute value for LUTs
    return (max_vel >= 0) ? max_vel : -max_vel;
}

void run_calibration(SharedState* state, I2CDMA& i2c, MotorControl& motor, AS5600Parser& parser) {
    if (!state) return;
    
    state->led_status.set(SystemStatus::StartupCalActive);
    
    CalibrationLUTs& luts = state->cal_luts;
    luts.valid = false;
    
    // Read initial state
    block_read_sensor(i2c, parser);
    
    // 1. Find Zero PWM
    luts.cw_zero_pwm = find_zero_pwm(MotorControl::Direction::CW, motor, i2c, parser);
    luts.ccw_zero_pwm = find_zero_pwm(MotorControl::Direction::CCW, motor, i2c, parser);
    
    // Apply friction compensation immediately so the speed sweeps are accurate
    motor.set_calibration_zero(luts.cw_zero_pwm, luts.ccw_zero_pwm);
    
    // 2. Measure speeds at different force levels
    for (uint8_t i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        int32_t force = CAL_FORCE_LEVELS[i];
        
        luts.cw_speed[i] = measure_max_speed(force, motor, i2c, parser);
        luts.ccw_speed[i] = measure_max_speed(-force, motor, i2c, parser);
    }
    
    // Verify LUTs are somewhat sane (speed should monotonically increase)
    // If not, we could apply a smoothing pass here, but for now we just accept it.
    
    luts.valid = true;
    
    state->led_status.clear(SystemStatus::StartupCalActive);
}
