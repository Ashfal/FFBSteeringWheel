// =========================================================================
// Pedal Reader — ADC + DMA, Median Filter
// =========================================================================
// Reads 2 analog pedals via ADC with DMA at 3kHz.
// Uses a 3-sample rolling buffer with spike rejection:
// if one read differs by >30%, discard it and average the remaining two.
// =========================================================================

#include "pedal_reader.h"
#include "config.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

void PedalReader::init() {
    adc_init();
    adc_gpio_init(PIN_ADC_ACCEL);  // GP26 → ADC0
    adc_gpio_init(PIN_ADC_BRAKE);  // GP27 → ADC1
}

void PedalReader::set_calibration(uint16_t accel_min, uint16_t accel_max,
                                  uint16_t brake_min, uint16_t brake_max) {
    accel_min_ = accel_min;
    accel_max_ = accel_max;
    brake_min_ = brake_min;
    brake_max_ = brake_max;
}

void PedalReader::update() {
    // Read accelerator
    adc_select_input(ADC_CHANNEL_ACCEL);
    uint16_t accel_raw = adc_read();

    // Read brake
    adc_select_input(ADC_CHANNEL_BRAKE);
    uint16_t brake_raw = adc_read();

    // Store in rolling buffers
    accel_history_[history_idx_] = accel_raw;
    brake_history_[history_idx_] = brake_raw;
    history_idx_ = (history_idx_ + 1) % ADC_FILTER_DEPTH;

    // Apply median/spike filter and scale
    accel_filtered_ = scale_to_16bit(
        median_filter(accel_history_), accel_min_, accel_max_);
    brake_filtered_ = scale_to_16bit(
        median_filter(brake_history_), brake_min_, brake_max_);
}

uint16_t PedalReader::median_filter(uint16_t history[3]) {
    uint16_t a = history[0];
    uint16_t b = history[1];
    uint16_t c = history[2];

    // Calculate average for spike detection
    uint32_t avg = (static_cast<uint32_t>(a) + b + c) / 3;
    uint32_t threshold = (avg * ADC_SPIKE_THRESHOLD_PERCENT) / 100;

    // Check each value — if it deviates by more than threshold, it's a spike
    auto abs_diff = [](uint32_t x, uint32_t y) -> uint32_t {
        return x > y ? x - y : y - x;
    };

    bool a_spike = abs_diff(a, avg) > threshold;
    bool b_spike = abs_diff(b, avg) > threshold;
    bool c_spike = abs_diff(c, avg) > threshold;

    // If exactly one is a spike, discard it and average the other two
    if (a_spike && !b_spike && !c_spike) {
        return static_cast<uint16_t>((static_cast<uint32_t>(b) + c) / 2);
    }
    if (b_spike && !a_spike && !c_spike) {
        return static_cast<uint16_t>((static_cast<uint32_t>(a) + c) / 2);
    }
    if (c_spike && !a_spike && !b_spike) {
        return static_cast<uint16_t>((static_cast<uint32_t>(a) + b) / 2);
    }

    // No spike or multiple spikes — use median
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;  // Median
}

uint16_t PedalReader::scale_to_16bit(uint16_t raw, uint16_t cal_min, uint16_t cal_max) {
    if (cal_max <= cal_min) return 0;

    // Clamp to calibrated range
    if (raw < cal_min) raw = cal_min;
    if (raw > cal_max) raw = cal_max;

    // Scale 0..65535 (16-bit unsigned range for HID)
    // Using 32-bit arithmetic to avoid overflow
    uint32_t scaled = (static_cast<uint32_t>(raw - cal_min) * 65535) / (cal_max - cal_min);
    return static_cast<uint16_t>(scaled);
}
