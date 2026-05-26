#pragma once
#include <cstdint>

class PedalReader {
public:
    void init();
    void update();
    uint16_t get_accel() const { return accel_filtered_; }
    uint16_t get_brake() const { return brake_filtered_; }

    // Calibration: set min/max ADC values for scaling
    void set_calibration(uint16_t accel_min, uint16_t accel_max,
                         uint16_t brake_min, uint16_t brake_max);

private:
    uint16_t median_filter(uint16_t history[3]);
    uint16_t scale_to_16bit(uint16_t raw, uint16_t cal_min, uint16_t cal_max);

    uint16_t accel_history_[3] = {};
    uint16_t brake_history_[3] = {};
    uint8_t  history_idx_ = 0;

    uint16_t accel_filtered_ = 0;
    uint16_t brake_filtered_ = 0;

    // Calibration ranges (ADC values)
    uint16_t accel_min_ = 0;
    uint16_t accel_max_ = 4095;
    uint16_t brake_min_ = 0;
    uint16_t brake_max_ = 4095;
};
