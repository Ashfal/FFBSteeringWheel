#pragma once
#include <cstdint>
#include "config.h"

class AS5600Parser {
public:
    void init();

    // Called from DMA interrupt with raw I2C data.
    // Returns true if data is valid (no hardware error).
    // On hardware error, flags the error and returns false.
    bool update(uint8_t status_reg, uint16_t raw_angle);

    int32_t get_position() const { return accumulated_position_; }
    int32_t get_velocity() const { return velocity_; }
    uint8_t get_error_flags() const { return error_flags_; }

    // Set the center offset (from flash calibration)
    void set_center(int32_t center) { center_offset_ = center; }

private:
    // Position tracking
    int32_t  accumulated_position_ = 0;   // Continuous position from center
    int32_t  center_offset_ = 0;          // Flash-saved center position
    uint16_t last_raw_angle_ = 0;
    bool     first_read_ = true;

    // Velocity calculation
    int32_t  velocity_ = 0;               // Raw counts / ms
    uint64_t last_time_us_ = 0;

    // Error state
    uint8_t  error_flags_ = 0;

    // Handle 12-bit encoder wrap-around with 1:2 gear ratio
    int32_t  compute_delta(uint16_t current, uint16_t previous);
};
