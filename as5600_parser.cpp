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
        // On first read, set position relative to center
        accumulated_position_ = static_cast<int32_t>(raw_angle) - center_offset_;
        return true;
    }

    // Compute delta handling 12-bit wrap-around
    int32_t delta = compute_delta(raw_angle, last_raw_angle_);

    // ---- Velocity Calculation ----
    uint64_t dt_us = now - last_time_us_;
    if (dt_us == 0) dt_us = 1;  // Prevent division by zero

    // velocity = delta counts per ms = (delta * 1000) / dt_us
    int32_t new_velocity = (delta * 1000) / static_cast<int32_t>(dt_us);

    // ---- Filter Impossible Physics Jumps ----
    // If delta exceeds MAX_PHYSICAL_VELOCITY worth of counts in this interval,
    // discard this read and estimate using last known velocity
    int32_t max_delta = (MAX_PHYSICAL_VELOCITY * static_cast<int32_t>(dt_us)) / 1000;

    if (delta > max_delta || delta < -max_delta) {
        // Impossible jump — estimate position from last velocity
        int32_t estimated_delta = (velocity_ * static_cast<int32_t>(dt_us)) / 1000;
        accumulated_position_ += estimated_delta;
        // Keep last velocity (don't update with bad data)
    } else {
        // Valid read
        accumulated_position_ += delta;
        velocity_ = new_velocity;
    }

    last_raw_angle_ = raw_angle;
    last_time_us_ = now;

    return true;
}

int32_t AS5600Parser::compute_delta(uint16_t current, uint16_t previous) {
    // Compute shortest-path delta on a 12-bit circular encoder (0..4095)
    int32_t delta = static_cast<int32_t>(current) - static_cast<int32_t>(previous);

    // Handle wrap-around: if delta > 2048, the encoder wrapped backward
    // if delta < -2048, it wrapped forward
    if (delta > 2048) {
        delta -= 4096;
    } else if (delta < -2048) {
        delta += 4096;
    }

    // The gear ratio is 1:2, so each encoder count = 1 wheel count
    // (the encoder sees half the wheel rotation, so ENCODER_COUNTS_PER_REV
    //  maps to WHEEL_COUNTS_PER_REV = 2 * ENCODER_COUNTS_PER_REV)
    // But since the encoder already reads the geared output, the delta
    // in encoder counts IS the delta in wheel encoder counts.
    // The 1:2 mapping is already accounted for in config.h constants.
    return delta;
}
