#pragma once
#include <cstdint>
#include "config.h"
#include "shared_state.h"

class MotorControl {
public:
    enum class Direction : uint8_t {
        OFF = 0,
        CW  = 1,
        CCW = 2,
        BRAKE = 3
    };

    void init(const CalibrationState* cal_state);

    // Set minimum PWM needed to overcome static friction (from calibration)
    void set_calibration_zero(uint16_t cw_val, uint16_t ccw_val);

    // Set the target PWM and direction.
    // Handles dead-time, friction compensation, and stall protection.
    // Set physical hardware PWM (-10000 to +10000 scaled internally to FORWARD_MAX_PWM)
    void set_force(int16_t force, int32_t velocity);

    // Set raw PWM for calibration, with stall governor logic
    void set_pwm(uint16_t pwm, Direction dir, int32_t velocity);

    // Immediate stop (coast)
    void stop() { apply_pwm(0, Direction::OFF); }

    // Active stop (short terminals)
    void brake() { apply_pwm(0, Direction::BRAKE); }

    // Applies the hardware limits for the current speed
    uint16_t get_safe_max_pwm(Direction dir, int32_t velocity);

private:
    uint16_t cw_zero_pwm_ = 0;
    uint16_t cw_active_range = 0;
    uint16_t ccw_zero_pwm_ = 0;
    uint16_t ccw_active_range = 0;
    const CalibrationState* cal_state_ = nullptr;
    Direction current_direction_ = Direction::OFF;

    void apply_pwm(uint16_t pwm, Direction dir);
};
