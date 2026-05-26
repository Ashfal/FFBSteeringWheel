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

    current_direction_ = Direction::BRAKE;
}

void MotorControl::emergency_stop() {
    pwm_set_gpio_level(PIN_PWM_LPWM, 0);
    pwm_set_gpio_level(PIN_PWM_RPWM, 0);
    gpio_put(PIN_PWM_EN, 0);
    current_direction_ = Direction::BRAKE;
}

void MotorControl::set_target(uint16_t pwm, Direction direction, int32_t velocity) {
    if (direction == Direction::BRAKE || pwm == 0) {
        apply_pwm(0, Direction::BRAKE);
        return;
    }

    // ---- Friction Compensation ----
    // Add the calibrated zero-friction PWM offset
    uint16_t zero_pwm = (direction == Direction::CW) ? cw_zero_pwm_ : ccw_zero_pwm_;
    
    // Scale the requested PWM (0..PWM_WRAP) into the active range (zero_pwm..PWM_WRAP)
    if (PWM_WRAP > zero_pwm) {
        uint32_t active_range = PWM_WRAP - zero_pwm;
        pwm = zero_pwm + ((static_cast<uint32_t>(pwm) * active_range) / PWM_WRAP);
    }
    if (pwm > PWM_WRAP) pwm = PWM_WRAP;

    // ---- Stall Protection Governor ----
    // If the wheel is moving very slowly or stalled, we must limit the max PWM
    // to prevent the BTS7960 from burning out due to prolonged high current.
    uint32_t abs_velocity = (velocity >= 0) ? velocity : -velocity;
    
    uint16_t max_allowed_pwm = PWM_WRAP;
    
    if (abs_velocity < static_cast<uint32_t>(STALL_VELOCITY_THRESHOLD)) {
        // Interpolate between STALL_PWM_MAX (at v=0) and PWM_WRAP (at v=THRESHOLD)
        uint32_t range = PWM_WRAP - STALL_PWM_MAX;
        max_allowed_pwm = STALL_PWM_MAX + ((abs_velocity * range) / STALL_VELOCITY_THRESHOLD);
    }

    if (pwm > max_allowed_pwm) {
        pwm = max_allowed_pwm;
    }

    apply_pwm(pwm, direction);
}

void MotorControl::apply_pwm(uint16_t pwm, Direction dir) {
    if (dir != current_direction_) {
        // ---- Dead-Time Insertion ----
        // Before changing direction, turn both off and wait to prevent shoot-through
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        
        // Disable EN during dead-time
        gpio_put(PIN_PWM_EN, 0);

        // Blocking wait (50us is very short, acceptable in 1ms loop)
        busy_wait_us_32(DEAD_TIME_US);

        current_direction_ = dir;
    }

    if (dir == Direction::BRAKE || pwm == 0) {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        gpio_put(PIN_PWM_EN, 0);
    } else if (dir == Direction::CW) {
        pwm_set_gpio_level(PIN_PWM_RPWM, 0);
        pwm_set_gpio_level(PIN_PWM_LPWM, pwm);
        gpio_put(PIN_PWM_EN, 1);
    } else if (dir == Direction::CCW) {
        pwm_set_gpio_level(PIN_PWM_LPWM, 0);
        pwm_set_gpio_level(PIN_PWM_RPWM, pwm);
        gpio_put(PIN_PWM_EN, 1);
    }
}
