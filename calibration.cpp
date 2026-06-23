// =========================================================================
// Calibration Routine
// =========================================================================
// Sweeps the motor to find static friction limits and populate speed LUTs.
// Runs synchronously on Core 0, before Core 1 is launched, with full peripheral access.
// =========================================================================

#include "calibration.h"
#include "config.h"
#include "pico/time.h"
#include "i2c_dma.h"
#include "motor_control.h"
#include "as5600_parser.h"
#include "hardware/watchdog.h"

// Helper to do a blocking I2C read during calibration
// Returns false if the read times out or the sensor reports an error.
static bool block_read_sensor(I2CDMA& i2c, AS5600Parser& parser) {
    i2c.start_read();
    // Wait for DMA completion with timeout (10ms — well above the ~0.3ms I2C transfer)
    uint64_t deadline = time_us_64() + 10000;
    while (!i2c.handle_isr()) {
        if (time_us_64() > deadline) {
            return false;  // Timed out — AS5600 not responding
        }
        tight_loop_contents();
    }
    
    const uint8_t* raw = i2c.get_data();
    uint8_t status = raw[0];
    uint16_t angle = (static_cast<uint16_t>(raw[1]) << 8) | raw[2];
    return parser.update(status, angle);
}

// Find minimum PWM to overcome static friction
static uint16_t find_zero_pwm(MotorControl::Direction dir, MotorControl& motor, I2CDMA& i2c, AS5600Parser& parser, LEDController& led) {
    uint16_t test_pwm = PWM_WRAP / 10; //start at 10% PWM
    
    // Ensure we are stopped
    motor.brake();
    led.sleep_ms(200);

    block_read_sensor(i2c, parser);
    int32_t start_pos = parser.get_position();

    while (test_pwm < STALL_PWM_MAX) {
        motor.set_pwm(test_pwm, dir, 0);
        
        // Wait briefly for movement
        led.sleep_ms(50);
        
        block_read_sensor(i2c, parser);
        int32_t pos = parser.get_position();
        
        int32_t delta = pos - start_pos;
        if (delta < 0) delta = -delta;

        int32_t vel = parser.get_velocity();
        if (vel < 0) vel = -vel;

        // If we moved, static friction is broken
        if (vel > 0) {
            // Now prove it can hold said movement
            if (delta > CAL_ZERO_MIN_SWEEP_COUNTS) break;
            else continue;
        } else {
            //if it stops we reset the position and start over
            start_pos = pos;
        }

        test_pwm += 10;
    }
    
    motor.brake();
    // Wait for wheel to settle
    uint64_t settle_start = time_us_64();
    while (time_us_64() - settle_start < 1000000) {
        block_read_sensor(i2c, parser);
        if (parser.get_velocity() == 0) break;
        led.update();
        sleep_ms(10);
    }
    return test_pwm;
}

// Sweep at a constant force and find maximum stable speed
static int32_t measure_max_speed(int32_t force, MotorControl& motor, I2CDMA& i2c, AS5600Parser& parser, LEDController& led) {
    motor.set_force(force, 0);
    
    bool is_cw = force > 0;
    
    int32_t max_vel = 0;
    uint32_t samples = 0;
    
    // Sweep for up to 500ms or until we travel MIN_SWEEP_COUNTS
    int32_t start_pos = parser.get_position();
    uint64_t start_time = time_us_64();
    
    while (true) {
        if (time_us_64() - start_time > 5000000) break; // 5s timeout

        led.update();
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
        int32_t distance = pos - start_pos;
        if (!is_cw) distance = -distance;
        
        if (distance > CAL_FORCE_MIN_SWEEP_COUNTS) break; // Travelled enough distance
    }
    
    motor.brake();
    // Wait for wheel to settle
    uint64_t settle_start = time_us_64();
    while (time_us_64() - settle_start < 2000000) {
        block_read_sensor(i2c, parser);
        if (parser.get_velocity() == 0) break;
        led.update();
        sleep_ms(10);
    }
    
    // Convert to absolute value for LUTs
    return (max_vel >= 0) ? max_vel : -max_vel;
}

void run_calibration(SharedState& state, ButtonReader& buttons, PedalReader& pedals, LEDController& led, FlashStorage& flash) {

    // Instantiate hardware for calibration locally on the stack
    I2CDMA i2c;
    AS5600Parser parser;
    MotorControl motor;

    if (!i2c.init()) {
        state.cal_state.valid = false;
        state.led_status.set(SystemStatus::EncoderConfWriteFailed);
        while (true) {
            led.sleep_ms(10);
        }
    }
    parser.init();
    motor.init(&state.cal_state);

    state.led_status.force(SystemStatus::MotorSweepsActive);
    led.update();
    
    // 1. Grab absolute raw center
    // Do a blocking read to get the raw angle safely
    bool initial_read_ok = false;
    for (int retry = 0; retry < 5; retry++) {
        if (block_read_sensor(i2c, parser)) {
            initial_read_ok = true;
            break;
        }
        sleep_ms(5);
    }
    
    if (!initial_read_ok) {
        state.cal_state.valid = false;
        // If sensor is dead, we can't continue safely
        state.led_status.set(SystemStatus::FlashWriteFailed);
        while (true) {
            led.update();
            sleep_ms(1);
        }
    }
    
    int32_t raw_center = parser.get_absolute_raw() & 0x0FFF;
    
    // Save to shared state and parser
    state.cal_state.center_offset.store(raw_center);
    parser.init();
    parser.set_center(raw_center);

    CalibrationState& luts = state.cal_state;
    luts.valid = false;
    
    // Read initial state
    block_read_sensor(i2c, parser);
    
    // 1. Find Zero PWM
    uint16_t cw_zero = find_zero_pwm(MotorControl::Direction::CW, motor, i2c, parser, led);
    uint16_t ccw_zero = find_zero_pwm(MotorControl::Direction::CCW, motor, i2c, parser, led);
    
    // Apply friction compensation immediately so the speed sweeps are accurate
    motor.set_calibration_zero(cw_zero, ccw_zero);
    
    luts.cw_zero_pwm.store(cw_zero);
    luts.ccw_zero_pwm.store(ccw_zero);
    
    // 2. Measure speeds at different force levels
    for (uint8_t i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        int32_t force = CAL_FORCE_LEVELS[i];
        
        luts.cw_speed[i].store(measure_max_speed(force, motor, i2c, parser, led));
        luts.ccw_speed[i].store(measure_max_speed(-force, motor, i2c, parser, led));
    }
    
    // Verify LUTs are somewhat sane (speed should monotonically increase)
    // If not, we could apply a smoothing pass here, but for now we just accept it.
    
    luts.valid.store(true);
    state.led_status.clear(SystemStatus::MotorSweepsActive);

    // 3. Pedal Calibration Phase
    state.led_status.force(SystemStatus::PedalCalActive);

    // Let's just track min/max for pedals
    uint16_t accel_min = 4095, accel_max = 0;
    uint16_t brake_min = 4095, brake_max = 0;

    // Wait for user to release all buttons first
    while (buttons.get_buttons() != 0) {
        buttons.update();
        led.update();
        sleep_us(BUTTON_UPDATE_INTERVAL_US);
    }

    // Phase 2: User pumps pedals. Long press cal button again to save.
    uint64_t press_time_us = 0;
    bool save_triggered = false;
    
    while (!save_triggered) {
        buttons.update();
        pedals.update();
        led.update();

        // Read raw compensated values so calibration matches regular operation.
        uint16_t a_raw = 0;
        uint16_t b_raw = 0;
        pedals.read_raw_compensated(a_raw, b_raw);

        if (a_raw < accel_min) accel_min = a_raw;
        if (a_raw > accel_max) accel_max = a_raw;
        if (b_raw < brake_min) brake_min = b_raw;
        if (b_raw > brake_max) brake_max = b_raw;

        if (buttons.get_buttons() != 0) {
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

    state.led_status.clear(SystemStatus::PedalCalActive);

    // Flash LED quickly to indicate flash save
    state.led_status.set(SystemStatus::RapidFlash);
    led.sleep_ms(1000);
    state.led_status.clear(SystemStatus::RapidFlash);

    // Save everything to flash
    FlashCalibrationData data;
    data.center_position = state.cal_state.center_offset.load();
    data.accel_min = accel_min;
    data.accel_max = accel_max;
    data.brake_min = brake_min;
    data.brake_max = brake_max;
    
    data.cw_zero_pwm = state.cal_state.cw_zero_pwm.load();
    data.ccw_zero_pwm = state.cal_state.ccw_zero_pwm.load();
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        data.cw_speed[i] = state.cal_state.cw_speed[i].load();
        data.ccw_speed[i] = state.cal_state.ccw_speed[i].load();
    }
    data.wheel_angle_deg = state.cal_state.wheel_angle_deg.load();
    data.system_damper_strength = state.cal_state.system_damper_strength.load();
    data.forward_max_pwm = state.cal_state.forward_max_pwm.load();
    data.force_scale_percent = state.cal_state.force_scale_percent.load();
    data.friction_fade_force = state.cal_state.friction_fade_force.load();

    // Use core1_running = false since Core 1 isn't running yet
    bool save_success = flash.save(data, false);

    if (!save_success) {
        state.led_status.set(SystemStatus::FlashWriteFailed);
        // Do not reboot; just stay here and flash the error code
        while (true) {
            led.update();
            sleep_ms(10);
        }
    }

    // Reboot to apply new flash settings cleanly
    watchdog_reboot(0, 0, 1);
    while (true) {
        tight_loop_contents();
    }
}
