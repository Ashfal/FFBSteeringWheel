// =========================================================================
// FFB Processor — Force Feedback Effect Calculation Engine
// =========================================================================
// Processes all active effects and returns a combined PWM + direction.
// All calculations use integer math only — no floats.
// Force range: -10000 to +10000 (matches PID spec normalized range).
// =========================================================================

#include "ffb_processor.h"
#include "pico/time.h"

// =========================================================================
// Integer sine LUT — 91 entries for 0..90 degrees (1° steps)
// Values scaled to 0..10000
// =========================================================================

static const int16_t sin_lut[91] = {
        0,   175,   349,   523,   698,   872,  1045,  1219,  1392,  1564,
     1736,  1908,  2079,  2250,  2419,  2588,  2756,  2924,  3090,  3256,
     3420,  3584,  3746,  3907,  4067,  4226,  4384,  4540,  4695,  4848,
     5000,  5150,  5299,  5446,  5592,  5736,  5878,  6018,  6157,  6293,
     6428,  6561,  6691,  6820,  6947,  7071,  7193,  7314,  7431,  7547,
     7660,  7771,  7880,  7986,  8090,  8192,  8290,  8387,  8480,  8572,
     8660,  8746,  8829,  8910,  8988,  9063,  9135,  9205,  9272,  9336,
     9397,  9455,  9511,  9563,  9613,  9659,  9703,  9744,  9781,  9816,
     9848,  9877,  9903,  9925,  9945,  9962,  9976,  9986,  9994,  9998,
    10000
};

int32_t FFBProcessor::int_sin(uint32_t angle_centideg) {
    // Normalize to 0..35999
    angle_centideg = angle_centideg % 36000;

    // Convert centidegrees to degrees (integer, losing sub-degree precision)
    uint32_t deg = angle_centideg / 100;

    // Quadrant decomposition
    int32_t sign = 1;
    if (deg >= 180) {
        sign = -1;
        deg -= 180;
    }
    if (deg > 90) {
        deg = 180 - deg;
    }

    return sign * sin_lut[deg];
}

// =========================================================================
// Main calculation
// =========================================================================

FFBOutput FFBProcessor::calculate(int32_t position, int32_t velocity,
                                  const EffectState& effects) {
    FFBOutput out;
    out.pwm = 0;
    out.direction = Direction::OFF;

    // ---- Electronic End-Stop (Early Exit) ----
    // If wheel is beyond physical limit, apply a proportional reverse spring
    // and skip ALL other effect processing.
    if (position > MAX_HALF_ANGLE_COUNTS || position < -MAX_HALF_ANGLE_COUNTS) {
        int32_t overshoot = (position > 0)
            ? (position - MAX_HALF_ANGLE_COUNTS)
            : (position + MAX_HALF_ANGLE_COUNTS);

        // Proportional spring: force = -overshoot scaled to PWM range
        // Cap at PWM_WRAP
        int32_t force = (-overshoot * PWM_WRAP) / MAX_HALF_ANGLE_COUNTS;
        if (force > PWM_WRAP) force = PWM_WRAP;
        if (force < -static_cast<int32_t>(PWM_WRAP)) force = -static_cast<int32_t>(PWM_WRAP);

        if (force > 0) {
            out.pwm = static_cast<int16_t>(force);
            out.direction = Direction::CW;
        } else if (force < 0) {
            out.pwm = static_cast<int16_t>(-force);
            out.direction = Direction::CCW;
        }
        return out;
    }

    // ---- Check if actuators are enabled and not paused ----
    if (!effects.actuators_enabled || effects.device_paused) {
        return out;
    }

    // ---- Accumulate forces from all active effects ----
    int32_t total_force = 0;
    uint64_t now = time_us_64();

    for (uint8_t i = 0; i < MAX_EFFECTS; i++) {
        const EffectSlot& e = effects.effects[i];
        if (e.state != EffectSlot::STATE_PLAYING) continue;

        // Calculate elapsed time in ms
        uint32_t elapsed_ms = static_cast<uint32_t>((now - e.start_time_us) / 1000);

        // Check duration (0xFFFF or 0x7FFF = infinite)
        uint16_t duration = e.params.duration;
        if (duration != 0xFFFF && duration != 0x7FFF && elapsed_ms > duration) {
            // Effect has expired — but we can't modify effects here (read-only)
            // Core 1 will handle expiry in its loop
            continue;
        }

        // Direction scaling: angle_ratio based on directionX
        // directionX: 0..255 maps to 0..360 degrees
        // For a 1-axis wheel, we primarily care about the X-axis contribution
        uint32_t dir_angle = static_cast<uint32_t>(e.params.directionX) * 36000 / 255;
        int32_t angle_ratio = int_sin(dir_angle);
        // angle_ratio is in -10000..+10000

        int32_t force = 0;

        // Determine effect type from the effectType field
        switch (e.params.effectType) {
            case 1:  // ET Constant Force (0x26)
                force = calc_constant_force(e);
                force = apply_envelope(e, force, elapsed_ms);
                force = (force * angle_ratio) / 10000;
                break;

            case 2:  // ET Ramp (0x27)
                force = calc_ramp_force(e, elapsed_ms);
                force = apply_envelope(e, force, elapsed_ms);
                force = (force * angle_ratio) / 10000;
                break;

            case 3:  // ET Square (0x30)
            case 4:  // ET Sine (0x31)
            case 5:  // ET Triangle (0x32)
            case 6:  // ET Sawtooth Up (0x33)
            case 7:  // ET Sawtooth Down (0x34)
                force = calc_periodic_force(e, elapsed_ms);
                force = apply_envelope(e, force, elapsed_ms);
                force = (force * angle_ratio) / 10000;
                break;

            case 8:  // ET Spring (0x40) — uses position as metric
                force = calc_condition_force(e, position, 0);
                break;

            case 9:  // ET Damper (0x41) — uses velocity as metric
                force = calc_condition_force(e, velocity, 0);
                break;

            case 10: // ET Inertia (0x42) — uses acceleration as metric
                // We don't track acceleration directly; approximate as 0
                force = 0;
                break;

            case 11: // ET Friction (0x43) — uses velocity sign
                force = calc_condition_force(e, velocity, 0);
                break;

            default:
                break;
        }

        // Apply per-effect gain (0..255 → 0..10000)
        force = (force * e.params.gain) / 255;

        total_force += force;
    }

    // Apply device gain
    total_force = (total_force * effects.device_gain) / 255;

    // Clamp to output range
    if (total_force > 10000) total_force = 10000;
    if (total_force < -10000) total_force = -10000;

    // Scale from -10000..+10000 to -PWM_WRAP..+PWM_WRAP
    int32_t pwm = (total_force * PWM_WRAP) / 10000;

    if (pwm > 0) {
        out.pwm = static_cast<int16_t>(pwm);
        out.direction = Direction::CW;
    } else if (pwm < 0) {
        out.pwm = static_cast<int16_t>(-pwm);
        out.direction = Direction::CCW;
    } else {
        out.pwm = 0;
        out.direction = Direction::OFF;
    }

    return out;
}

// =========================================================================
// Individual effect calculators
// =========================================================================

int32_t FFBProcessor::calc_constant_force(const EffectSlot& e) {
    // Magnitude: -10000..+10000 in the descriptor
    return static_cast<int32_t>(e.constant.magnitude);
}

int32_t FFBProcessor::calc_ramp_force(const EffectSlot& e, uint32_t elapsed_ms) {
    uint16_t duration = e.params.duration;
    if (duration == 0) return e.ramp.startMagnitude;

    int32_t start = static_cast<int32_t>(e.ramp.startMagnitude);
    int32_t end   = static_cast<int32_t>(e.ramp.endMagnitude);

    if (elapsed_ms >= duration) return end;

    // Linear interpolation: start + (end - start) * elapsed / duration
    return start + ((end - start) * static_cast<int32_t>(elapsed_ms)) / duration;
}

int32_t FFBProcessor::calc_periodic_force(const EffectSlot& e, uint32_t elapsed_ms) {
    int32_t  magnitude = static_cast<int32_t>(e.periodic.magnitude);
    int32_t  offset    = static_cast<int32_t>(e.periodic.offset);
    uint32_t phase     = static_cast<uint32_t>(e.periodic.phase);  // 0..35999 centidegrees
    uint32_t period    = static_cast<uint32_t>(e.periodic.period); // ms

    if (period == 0) return offset;

    // Calculate phase position within the period
    uint32_t phase_ms = (phase * period) / 36000;
    uint32_t t = (elapsed_ms + phase_ms) % period;

    int32_t force = 0;

    switch (e.params.effectType) {
        case 3: { // Square
            if (t < period / 2) {
                force = offset + magnitude;
            } else {
                force = offset - magnitude;
            }
            break;
        }

        case 4: { // Sine
            // angle = (t / period) * 36000 centidegrees
            uint32_t angle = (t * 36000) / period;
            force = offset + (magnitude * int_sin(angle)) / 10000;
            break;
        }

        case 5: { // Triangle
            int32_t half = static_cast<int32_t>(period / 2);
            int32_t ti = static_cast<int32_t>(t);
            if (half == 0) half = 1;
            if (ti < half) {
                // Rising: -magnitude to +magnitude
                force = offset - magnitude + (2 * magnitude * ti) / half;
            } else {
                // Falling: +magnitude to -magnitude
                force = offset + magnitude - (2 * magnitude * (ti - half)) / half;
            }
            break;
        }

        case 6: { // Sawtooth Up
            force = offset - magnitude +
                    (2 * magnitude * static_cast<int32_t>(t)) / static_cast<int32_t>(period);
            break;
        }

        case 7: { // Sawtooth Down
            force = offset + magnitude -
                    (2 * magnitude * static_cast<int32_t>(t)) / static_cast<int32_t>(period);
            break;
        }
    }

    return force;
}

int32_t FFBProcessor::calc_condition_force(const EffectSlot& e, int32_t metric, uint8_t axis) {
    if (axis >= 2) axis = 0;
    const auto& cond = e.condition[axis];

    int32_t cp_offset   = static_cast<int32_t>(cond.cpOffset);
    int32_t pos_coeff   = static_cast<int32_t>(cond.positiveCoefficient);
    int32_t neg_coeff   = static_cast<int32_t>(cond.negativeCoefficient);
    int32_t pos_sat     = static_cast<int32_t>(cond.positiveSaturation);
    int32_t neg_sat     = static_cast<int32_t>(cond.negativeSaturation);
    int32_t dead_band   = static_cast<int32_t>(cond.deadBand);

    int32_t force = 0;

    if (metric < (cp_offset - dead_band)) {
        // Negative side
        force = (neg_coeff * (metric - (cp_offset - dead_band))) / 10000;
        if (force < -neg_sat) force = -neg_sat;
    } else if (metric > (cp_offset + dead_band)) {
        // Positive side
        force = (pos_coeff * (metric - (cp_offset + dead_band))) / 10000;
        if (force > pos_sat) force = pos_sat;
    }
    // Inside dead band → force = 0

    return -force;  // Negate: condition forces resist the metric
}

int32_t FFBProcessor::apply_envelope(const EffectSlot& e, int32_t force,
                                     uint32_t elapsed_ms) {
    uint16_t duration = e.params.duration;
    uint32_t attack_time = e.envelope.attackTime;
    uint32_t fade_time   = e.envelope.fadeTime;
    int32_t  attack_level = static_cast<int32_t>(e.envelope.attackLevel);
    int32_t  fade_level   = static_cast<int32_t>(e.envelope.fadeLevel);

    // Normalize force magnitude for envelope scaling
    int32_t magnitude = (force >= 0) ? force : -force;

    if (elapsed_ms < attack_time && attack_time > 0) {
        // Attack phase: ramp from attack_level to magnitude
        int32_t level = attack_level +
            ((magnitude - attack_level) * static_cast<int32_t>(elapsed_ms)) /
            static_cast<int32_t>(attack_time);
        return (force >= 0) ? level : -level;
    }

    if (duration != 0xFFFF && duration != 0x7FFF &&
        elapsed_ms > (duration - fade_time) && fade_time > 0) {
        // Fade phase: ramp from magnitude to fade_level
        uint32_t fade_elapsed = elapsed_ms - (duration - fade_time);
        int32_t level = magnitude -
            ((magnitude - fade_level) * static_cast<int32_t>(fade_elapsed)) /
            static_cast<int32_t>(fade_time);
        return (force >= 0) ? level : -level;
    }

    return force;  // Sustain phase
}

int32_t FFBProcessor::lookup_expected_speed(uint16_t pwm, Direction dir) const {
    if (!cal_luts_ || !cal_luts_->valid) return 0;

    const int32_t* lut = (dir == Direction::CW) ? cal_luts_->cw_speed : cal_luts_->ccw_speed;

    // Find the two bracketing PWM levels and interpolate
    for (uint8_t i = 0; i < CAL_PWM_LEVEL_COUNT; i++) {
        if (pwm <= CAL_PWM_LEVELS[i]) {
            if (i == 0) return (static_cast<int32_t>(pwm) * lut[0]) / CAL_PWM_LEVELS[0];
            // Linear interpolation between lut[i-1] and lut[i]
            int32_t pwm_low  = CAL_PWM_LEVELS[i - 1];
            int32_t pwm_high = CAL_PWM_LEVELS[i];
            int32_t spd_low  = lut[i - 1];
            int32_t spd_high = lut[i];
            return spd_low + ((spd_high - spd_low) * (pwm - pwm_low)) / (pwm_high - pwm_low);
        }
    }
    return lut[CAL_PWM_LEVEL_COUNT - 1];
}
