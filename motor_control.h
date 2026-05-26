#pragma once
#include <cstdint>
#include "config.h"
#include "shared_state.h"

class MotorControl {
public:
    void init();

    // Set minimum PWM needed to overcome static friction (from calibration)
    void set_calibration_zero(uint16_t cw_val, uint16_t ccw_val) {
        cw_zero_pwm_ = cw_val;
        ccw_zero_pwm_ = ccw_val;
    }

    // Set the target PWM and direction.
    // Handles dead-time, friction compensation, and stall protection.
    // velocity is required for the stall governor.
    void set_target(uint16_t pwm, Direction direction, int32_t velocity);

    // Immediate emergency stop
    void emergency_stop();

private:
    uint16_t cw_zero_pwm_ = 0;
    uint16_t ccw_zero_pwm_ = 0;
    Direction current_direction_ = Direction::BRAKE;

    void apply_pwm(uint16_t pwm, Direction dir);
};
