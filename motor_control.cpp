// =========================================================================
// Motor Control — BTS7960 Driver
// =========================================================================
// Manages the BTS7960 H-Bridge via hardware PWM.
// Features:
// - Hardware PWM (20kHz, phase-correct preferred but edge-aligned used here)
// - Non-blocking dead-time delay on direction change
// - Stall protection governor to prevent burning out the driver
// - Static friction compensation (from calibration)
// =========================================================================

#include "motor_control.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/time.h"

void MotorControl::init() {
    // Configure PWM pins
    gpio_set_function(PIN_PWM_LPWM, GPIO_FUNC_PWM);
    gpio_set_function(PIN_PWM_RPWM, GPIO_FUNC_PWM);

    uint slice_l = pwm_gpio_to_slice_num(PIN_PWM_LPWM);
    uint slice_r = pwm_gpio_to_slice_num(PIN_PWM_RPWM);

    // Assuming LPWM and RPWM might be on different slices, configure both.
    // Ideally they are on the same slice (e.g. 6=3A, 7=3B).
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f); // 125MHz
    pwm_config_set_wrap(&config, PWM_WRAP); // TOP = 6249

    pwm_init(slice_l, &config, true);
    if (slice_l != slice_r) {
        pwm_init(slice_r, &config, true);
    }

    pwm_set_gpio_level(PIN_PWM_LPWM, 0);
    pwm_set_gpio_level(PIN_PWM_RPWM, 0);

    // Configure EN pin
    gpio_init(PIN_PWM_EN);
    gpio_set_dir(PIN_PWM_EN, GPIO_OUT);
    gpio_put(PIN_PWM_EN, 0); // Disable initially

    current_direction_ = Direction::OFF;
}

void MotorControl::stop() {
    apply_pwm(0, Direction::OFF);
}

void MotorControl::emergency_stop() {
    stop();
    gpio_put(PIN_PWM_EN, 0);
}

void MotorControl::set_calibration_zero(uint16_t cw_zero, uint16_t ccw_zero) {
    cw_zero_pwm_ = cw_zero;
    ccw_zero_pwm_ = ccw_zero;
}

void MotorControl::set_force(int32_t force, int32_t velocity) {
    if (force == 0) {
        stop();
        return;
    }

    Direction dir = (force > 0) ? Direction::CW : Direction::CCW;
    uint32_t abs_force = (force > 0) ? force : -force;
    if (abs_force > 10000) abs_force = 10000;

    // Determine the zero PWM offset based on direction
    uint16_t zero_pwm = (dir == Direction::CW) ? cw_zero_pwm_ : ccw_zero_pwm_;

    // Scale the requested force (0..10000) into the active range (zero_pwm..FORWARD_MAX_PWM)
    uint32_t pwm = 0;
    if (FORWARD_MAX_PWM > zero_pwm) {
        uint32_t active_range = FORWARD_MAX_PWM - zero_pwm;
        pwm = zero_pwm + ((abs_force * active_range) / 10000);
    }
    
    set_pwm(static_cast<uint16_t>(pwm), dir, velocity);
}

void MotorControl::set_pwm(uint16_t pwm, Direction dir, int32_t velocity) {
    if (pwm == 0 || dir == Direction::OFF) {
        stop();
        return;
    }

    if (pwm > PWM_WRAP) pwm = PWM_WRAP;

    // ---- Stall Protection Governor ----
    // Differentiate between moving "forward" (with the motor) and "backwards" (against the motor)
    bool is_forward = (dir == Direction::CW && velocity > 0) || (dir == Direction::CCW && velocity < 0);
    bool is_stalled = (velocity == 0);
    uint32_t abs_velocity = (velocity >= 0) ? velocity : -velocity;
    
    uint16_t max_allowed_pwm = PWM_WRAP;
    
    if (is_stalled) {
        max_allowed_pwm = STALL_PWM_MAX;
    } else if (is_forward) {
        if (abs_velocity < static_cast<uint32_t>(FORWARD_VELOCITY_THRESHOLD)) {
            // Linearly increase from STALL_PWM_MAX to PWM_WRAP
            uint32_t range = PWM_WRAP - STALL_PWM_MAX;
            max_allowed_pwm = STALL_PWM_MAX + ((abs_velocity * range) / FORWARD_VELOCITY_THRESHOLD);
        } else {
            max_allowed_pwm = PWM_WRAP;
        }
    } else {
        // Moving backwards (user fighting the motor)
        if (abs_velocity < static_cast<uint32_t>(BACKWARDS_VELOCITY_THRESHOLD)) {
            // Linearly decrease from STALL_PWM_MAX to BACKWARDS_PWM_MAX
            uint32_t range = STALL_PWM_MAX - BACKWARDS_PWM_MAX;
            max_allowed_pwm = STALL_PWM_MAX - ((abs_velocity * range) / BACKWARDS_VELOCITY_THRESHOLD);
        } else {
            max_allowed_pwm = BACKWARDS_PWM_MAX;
        }
    }

    if (pwm > max_allowed_pwm) {
        pwm = max_allowed_pwm;
    }

    apply_pwm(pwm, dir);
}

void MotorControl::apply_pwm(uint16_t pwm, Direction dir) {
    bool dir_changed = (dir != current_direction_);

    if (pwm == 0 || dir == Direction::OFF) {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        gpio_put(PIN_PWM_EN, 0);
        current_direction_ = Direction::OFF;
        return;
    }

    if (dir_changed) {
        // ---- Dead-Time Insertion ----
        // Before changing direction, turn both off and wait to prevent shoot-through
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        
        // Block tightly for DEAD_TIME_US (typically 50us)
        busy_wait_us_32(DEAD_TIME_US);
        
        current_direction_ = dir;
    }

    // Apply new duty cycle
    if (dir == Direction::CW) {
        pwm_set_gpio_level(PIN_PWM_LPWM, pwm);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
    } else {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, pwm);
    }
    gpio_put(PIN_PWM_EN, 1);
}
