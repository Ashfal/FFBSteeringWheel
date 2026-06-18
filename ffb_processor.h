#pragma once
#include <cstdint>
#include "config.h"
#include "shared_state.h"

struct FFBOutput {
    int32_t force;
};

class FFBProcessor {
public:
    void init(const CalibrationLUTs* luts) { cal_luts_ = luts; }

    // Calculate the combined force output from all active effects.
    // position: wheel position in raw counts from center
    // velocity: raw counts per sec (signed)
    // effects: snapshot of current effect state (caller holds spinlock)
    FFBOutput calculate(int32_t position, int32_t velocity,
                        EffectState& effects, int32_t max_half_angle_counts);

private:
    const CalibrationLUTs* cal_luts_ = nullptr;

    // Effect calculators — all return force in -10000..+10000 range
    int32_t calc_constant_force(const EffectSlot& e);
    int32_t calc_ramp_force(const EffectSlot& e, uint32_t elapsed_ms);
    int32_t calc_periodic_force(const EffectSlot& e, uint32_t elapsed_ms);
    int32_t calc_condition_force(const EffectSlot& e, int32_t metric, uint8_t axis);

    // Envelope application
    int32_t apply_envelope(const EffectSlot& e, int32_t force, uint32_t elapsed_ms);

    // Integer sine approximation (input: 0..35999 in 0.01° steps, output: -10000..10000)
    static int32_t int_sin(uint32_t angle_centideg);

    // Look up expected speed for a given PWM from calibration LUTs
    int32_t lookup_expected_speed(int32_t force) const;
    
    // Look up required holding force for a given velocity from calibration LUTs
    int32_t lookup_required_force(int32_t velocity) const;
};
