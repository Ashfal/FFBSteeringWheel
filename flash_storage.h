#pragma once
#include <cstdint>

struct FlashCalibrationData {
    uint32_t magic;             // Must be 0xFEEDFACE
    uint32_t version;           // Version number for future upgrades
    int32_t  center_position;   // Absolute raw encoder counts
    uint16_t accel_min;         // Pedal min ADC
    uint16_t accel_max;         // Pedal max ADC
    uint16_t brake_min;
    uint16_t brake_max;
    
    // Motor Calibration LUTs
    uint16_t cw_zero_pwm;
    uint16_t ccw_zero_pwm;
    int32_t  cw_speed[5];       // Note: Size matches CAL_FORCE_LEVEL_COUNT (5)
    int32_t  ccw_speed[5];
    
    uint32_t crc32;             // Integrity check
};

class FlashStorage {
public:
    // Load data from flash. Returns true if data was valid.
    bool load(FlashCalibrationData& out_data);

    // Save data to flash. Uses flash_safe_execute for multicore safety.
    bool save(const FlashCalibrationData& data);

private:
    uint32_t calculate_crc(const FlashCalibrationData& data) const;
};
