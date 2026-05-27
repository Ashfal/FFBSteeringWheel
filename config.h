#pragma once
#include <cstdint>

// =========================================================================
// GPIO PIN ASSIGNMENTS
// =========================================================================

// I2C0 — AS5600 magnetic encoder
constexpr uint8_t PIN_I2C_SDA          = 4;
constexpr uint8_t PIN_I2C_SCL          = 5;

// SPI0 — HCF4021B shift registers (buttons)
constexpr uint8_t PIN_SPI_SCK          = 18;
constexpr uint8_t PIN_SPI_RX           = 16;   // MISO
constexpr uint8_t PIN_SPI_LATCH        = 17;   // Directly wired parallel/serial control

// PWM — BTS7960 H-Bridge
constexpr uint8_t PIN_PWM_LPWM        = 6;
constexpr uint8_t PIN_PWM_RPWM        = 7;
constexpr uint8_t PIN_PWM_EN          = 8;     // Bridged left/right enable

// ADC — Analog pedals (0–3V)
constexpr uint8_t PIN_ADC_ACCEL       = 26;    // ADC0
constexpr uint8_t PIN_ADC_BRAKE       = 27;    // ADC1

// Status LED
constexpr uint8_t PIN_LED             = 25;    // Onboard LED

// =========================================================================
// WHEEL GEOMETRY (Raw Integer Units)
// =========================================================================
// AS5600: 12-bit → 4096 counts per encoder revolution
// Gear ratio 1:2 → 8192 encoder counts per wheel revolution
// All angles expressed in raw encoder counts, NOT degrees.

constexpr int32_t ENCODER_COUNTS_PER_REV  = 4096;
constexpr int32_t WHEEL_COUNTS_PER_REV    = ENCODER_COUNTS_PER_REV * 2;  // 8192

// Physical wheel range: 1080° total → ±540° from center
// 540° = 540/360 * 8192 = 12288 raw counts from center
constexpr int32_t MAX_WHEEL_ANGLE_DEG     = 1080;
constexpr int32_t MAX_HALF_ANGLE_DEG      = 540;
constexpr int32_t MAX_HALF_ANGLE_COUNTS   = (MAX_HALF_ANGLE_DEG * WHEEL_COUNTS_PER_REV) / 360;  // 12288

// =========================================================================
// PWM CONFIGURATION
// =========================================================================
// Target: 20kHz PWM
// RP2040 system clock: 125 MHz
// Edge-aligned: TOP = 125_000_000 / 20_000 - 1 = 6249
// Duty cycle range: 0 to PWM_WRAP (6249)

constexpr uint32_t PWM_FREQ_HZ           = 20000;
constexpr uint16_t PWM_WRAP              = 6249;   // TOP value, duty 0..6249
constexpr uint16_t FORWARD_MAX_PWM       = 6249;   // Software limit for FFB max force

// =========================================================================
// BTS7960 MOTOR SAFETY
// =========================================================================

// Dead-time between direction changes to prevent shoot-through (microseconds)
constexpr uint16_t DEAD_TIME_US           = 50;

// Stall protection governor:
// Differentiates between moving forward (with the motor) and backwards (against the motor).
constexpr uint16_t STALL_PWM_MAX                = 1250;
constexpr int32_t  FORWARD_VELOCITY_THRESHOLD   = 200;  // Raw counts/ms
constexpr int32_t  BACKWARDS_VELOCITY_THRESHOLD = 200;  // Raw counts/ms
constexpr uint16_t BACKWARDS_PWM_MAX            = 500;

// =========================================================================
// AS5600 SENSOR
// =========================================================================

constexpr uint8_t  AS5600_I2C_ADDR       = 0x36;
constexpr uint8_t  AS5600_REG_STATUS     = 0x0B;
constexpr uint8_t  AS5600_REG_RAW_ANGLE_H = 0x0C;
constexpr uint8_t  AS5600_REG_RAW_ANGLE_L = 0x0D;

constexpr uint32_t I2C_FREQ_HZ           = 100000;  // 100 kHz

// AS5600 status register bits
constexpr uint8_t  AS5600_STATUS_MH      = 0x08;    // Magnet too strong
constexpr uint8_t  AS5600_STATUS_ML      = 0x10;    // Magnet too weak
constexpr uint8_t  AS5600_STATUS_MD      = 0x20;    // Magnet detected

// Maximum physically possible velocity (raw counts per ms).
// Any delta exceeding this is a sensor glitch and must be discarded.
constexpr int32_t  MAX_PHYSICAL_VELOCITY  = 2000;

// =========================================================================
// BUTTON READING (SPI)
// =========================================================================

constexpr uint32_t SPI_FREQ_HZ           = 200000;  // 200 kHz
constexpr uint8_t  BUTTON_COUNT          = 16;       // 2x HCF4021B = 16 bits
constexpr uint8_t  DEBOUNCE_READS        = 3;        // Rolling buffer depth

// =========================================================================
// PEDAL READING (ADC)
// =========================================================================

constexpr uint32_t ADC_SAMPLE_FREQ_HZ    = 3000;    // 3 kHz
constexpr uint8_t  ADC_CHANNEL_ACCEL     = 0;       // GP26
constexpr uint8_t  ADC_CHANNEL_BRAKE     = 1;       // GP27
constexpr int32_t  ADC_SPIKE_THRESHOLD_PERCENT = 30; // Spike rejection threshold
constexpr uint8_t  ADC_FILTER_DEPTH      = 3;       // Rolling buffer depth

// =========================================================================
// LED STATUS CONTROLLER
// =========================================================================

constexpr uint32_t LED_FLASH_ON_MS       = 200;
constexpr uint32_t LED_FLASH_OFF_MS      = 500;
constexpr uint32_t LED_PAUSE_MS          = 2000;

// =========================================================================
// FFB CONFIGURATION
// =========================================================================

constexpr uint8_t  MAX_EFFECTS           = 40;

// =========================================================================
// CALIBRATION
// =========================================================================

constexpr uint32_t LONG_PRESS_MS         = 5000;    // Hold > 5s for Flash cal

// Force levels for speed LUT calibration sweeps (raw -10000 to +10000 scale)
// Corresponds to 10%, 25%, 50%, 75%, 100% force
constexpr int32_t  CAL_FORCE_LEVELS[]    = {
    1000,
    2500,
    5000,
    7500,
    10000
};
constexpr uint8_t  CAL_FORCE_LEVEL_COUNT = 5;

// Minimum sweep distance during calibration (180 degrees in raw counts)
constexpr int32_t  CAL_MIN_SWEEP_COUNTS  = WHEEL_COUNTS_PER_REV / 2;  // 4096

// =========================================================================
// I2C DMA / WATCHDOG TIMING
// =========================================================================

constexpr uint32_t I2C_READ_INTERVAL_US  = 1000;    // 1 ms
constexpr uint32_t I2C_WATCHDOG_TIMEOUT_US = 2000;  // 2 ms — motor kill if missed

// =========================================================================
// USB
// =========================================================================

constexpr uint16_t USB_VID               = 0xCAFE;
constexpr uint16_t USB_PID               = 0x4003;

// =========================================================================
// STATUS / ERROR CODES  (LED flash code = enum value)
// =========================================================================
// Priority: highest numeric value wins.

enum class SystemStatus : uint8_t {
    Normal           = 0,   // Solid ON — no flash code
    ReadyForCal      = 1,   // Awaiting button press for calibration
    StartupCalActive = 2,   // Startup calibration sweep in progress
    PedalCalActive   = 3,   // Pedal calibration active
    FlashCalMissing  = 5,   // No valid flash calibration data
    MagnetHigh       = 7,   // AS5600: magnet too strong (MH=1)
    MagnetLow        = 8,   // AS5600: magnet too weak (ML=1)
    MagnetMissing    = 9,   // AS5600: no magnet detected (MD=0)
    I2CWatchdogFired = 10,  // I2C DMA watchdog expired — motor killed
};
