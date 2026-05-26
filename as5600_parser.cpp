// =========================================================================
// AS5600 Parser — Position, Velocity & Error Detection
// =========================================================================
// Processes raw I2C reads from the AS5600 magnetic encoder.
// - Early exit on hardware errors (MH, ML, MD)
// - Handles 12-bit wrap-around with 1:2 gear ratio
// - Calculates velocity, filters impossible physics jumps
// =========================================================================

#include "as5600_parser.h"
#include "pico/time.h"

void AS5600Parser::init() {
    accumulated_position_ = 0;
    velocity_ = 0;
    turn_count_ = 0;
    last_raw_angle_ = 0;
    first_read_ = true;
    error_flags_ = 0;
    last_time_us_ = time_us_64();
}

bool AS5600Parser::update(uint8_t status_reg, uint16_t raw_angle) {
    // ---- EARLY EXIT: Hardware Error Check ----
    // MH (bit 3) = magnet too strong
    // ML (bit 4) = magnet too weak
    // MD (bit 5) = magnet detected
    error_flags_ = 0;

    if (status_reg & AS5600_STATUS_MH) {
        error_flags_ |= 0x01;  // ERR_MAGNET_HIGH
    }
    if (status_reg & AS5600_STATUS_ML) {
        error_flags_ |= 0x02;  // ERR_MAGNET_LOW
    }
    if (!(status_reg & AS5600_STATUS_MD)) {
        error_flags_ |= 0x04;  // ERR_MAGNET_MISSING
    }

    if (error_flags_ != 0) {
        // Hardware error — do NOT update position or velocity
        return false;
    }

    // ---- Parse Position ----
    // Mask to 12 bits (0..4095)
    raw_angle &= 0x0FFF;

    uint64_t now = time_us_64();

    if (first_read_) {
        last_raw_angle_ = raw_angle;
        last_time_us_ = now;
        first_read_ = false;
        
        // At boot, we ALWAYS assume the wheel is at 0 turns relative to the center.
        // We find the shortest path to the center to set the initial position.
        // If center is 4090 and we read 10, the delta is +16, so turn_count_ must be 1.
        int32_t raw_diff = static_cast<int32_t>(raw_angle) - center_offset_;
        if (raw_diff > 2048) {
            turn_count_ = -1;
        } else if (raw_diff < -2048) {
            turn_count_ = 1;
        } else {
            turn_count_ = 0;
        }
        
        accumulated_position_ = (turn_count_ * 4096) + static_cast<int32_t>(raw_angle) - center_offset_;
        return true;
    }

    // Compute raw delta and check for wraps
    int32_t delta = static_cast<int32_t>(raw_angle) - static_cast<int32_t>(last_raw_angle_);
    int32_t wraps = 0;
    
    if (delta > 2048) {
        delta -= 4096;
        wraps = -1;
    } else if (delta < -2048) {
        delta += 4096;
        wraps = 1;
    }

    // ---- Velocity Calculation ----
    uint64_t dt_us = now - last_time_us_;
    if (dt_us == 0) dt_us = 1;  // Prevent division by zero

    // velocity = delta counts per ms = (delta * 1000) / dt_us
    int32_t new_velocity = (delta * 1000) / static_cast<int32_t>(dt_us);

    // ---- Filter Impossible Physics Jumps ----
    int32_t max_delta = (MAX_PHYSICAL_VELOCITY * static_cast<int32_t>(dt_us)) / 1000;

    if (delta > max_delta || delta < -max_delta) {
        // Impossible jump — do NOT update turn_count_ or last_raw_angle_
        // Estimate position from last velocity
        int32_t estimated_delta = (velocity_ * static_cast<int32_t>(dt_us)) / 1000;
        accumulated_position_ += estimated_delta;
        last_time_us_ = now; // Update time so dt_us doesn't grow arbitrarily large
    } else {
        // Valid read
        turn_count_ += wraps;
        accumulated_position_ = (turn_count_ * 4096) + static_cast<int32_t>(raw_angle) - center_offset_;
        velocity_ = new_velocity;
        last_raw_angle_ = raw_angle;
        last_time_us_ = now;
    }

    return true;
}
