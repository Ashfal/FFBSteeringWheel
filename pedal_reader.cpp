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
    adc_gpio_init(PIN_ADC_VBUS);   // GP28 → ADC2
}

void PedalReader::set_calibration(uint16_t accel_min, uint16_t accel_max,
                                  uint16_t brake_min, uint16_t brake_max) {
    accel_min_ = accel_min;
    accel_max_ = accel_max;
    brake_min_ = brake_min;
    brake_max_ = brake_max;
}

void PedalReader::get_calibration(uint16_t& accel_min, uint16_t& accel_max,
                                  uint16_t& brake_min, uint16_t& brake_max) const {
    accel_min = accel_min_;
    accel_max = accel_max_;
    brake_min = brake_min_;
    brake_max = brake_max_;
}

void PedalReader::read_raw_compensated(uint16_t &accel_comp, uint16_t &brake_comp) {
    // Read raw values as close as possible to compensate spikes
    adc_select_input(ADC_CHANNEL_VBUS);
    uint16_t vbus_raw = adc_read();

    adc_select_input(ADC_CHANNEL_ACCEL);
    uint16_t accel_raw = adc_read();

    adc_select_input(ADC_CHANNEL_BRAKE);
    uint16_t brake_raw = adc_read();

    if (vbus_raw == 0) {
        accel_comp = 0;
        brake_comp = 0;
    } else {
        if (accel_raw > vbus_raw) {
            accel_raw = vbus_raw;
        }
        if (brake_raw > vbus_raw) {
            brake_raw = vbus_raw;
        }
        accel_comp = static_cast<uint16_t>((static_cast<uint32_t>(accel_raw) * 4095) / vbus_raw);
        brake_comp = static_cast<uint16_t>((static_cast<uint32_t>(brake_raw) * 4095) / vbus_raw);
    }
}

void PedalReader::update() {
    uint16_t accel_comp = 0;
    uint16_t brake_comp = 0;
    read_raw_compensated(accel_comp, brake_comp);

    // Store in rolling buffers
    accel_history_[history_idx_] = accel_comp;
    brake_history_[history_idx_] = brake_comp;
    history_idx_ = (history_idx_ + 1) % ADC_FILTER_DEPTH;

    // Apply trimmed mean filter and scale
    accel_filtered_ = scale_to_16bit(
        trimmed_mean_filter(accel_history_, ADC_FILTER_DEPTH), accel_min_, accel_max_);
    brake_filtered_ = scale_to_16bit(
        trimmed_mean_filter(brake_history_, ADC_FILTER_DEPTH), brake_min_, brake_max_);
}

uint16_t PedalReader::trimmed_mean_filter(const uint16_t* history, uint8_t depth) {
    // Requires at least 5 elements to discard 4 outliers
    if (depth < 5) return 0;

    uint16_t max1 = 0, max2 = 0;
    uint16_t min1 = 65535, min2 = 65535;
    uint32_t sum = 0;

    // Single O(N) pass to sum all and track the 2 highest and 2 lowest values
    for (uint8_t i = 0; i < depth; i++) {
        uint16_t val = history[i];
        sum += val;

        if (val > max1) {
            max2 = max1;
            max1 = val;
        } else if (val > max2) {
            max2 = val;
        }

        if (val < min1) {
            min2 = min1;
            min1 = val;
        } else if (val < min2) {
            min2 = val;
        }
    }

    // Discard the 4 outliers
    sum -= (max1 + max2 + min1 + min2);

    // Average the remaining elements
    // When depth=20, depth-4 = 16. The compiler optimizes this division into a `>> 4` bitshift.
    return static_cast<uint16_t>(sum / (depth - 4));
}

int16_t PedalReader::scale_to_16bit(uint16_t raw, uint16_t cal_min, uint16_t cal_max) {
    if (cal_max <= cal_min) return -32767;

    // Clamp to calibrated range
    if (raw < cal_min) raw = cal_min;
    if (raw > cal_max) raw = cal_max;

    // Scale 0..(cal_max-cal_min) to -32767..+32767
    uint32_t range = cal_max - cal_min;
    uint32_t val = raw - cal_min;
    int32_t scaled = -32767 + (static_cast<int32_t>(val) * 65534) / range;
    return static_cast<int16_t>(scaled);
}
