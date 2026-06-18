#pragma once
#include <cstdint>
#include <atomic>
#include "config.h"
#include "usb_ffb_descriptors.h"
#include "hardware/sync.h"
// =========================================================================
// Core 1 → Core 0  (Written ONLY by Core 1, read by Core 0)
// Uses atomics — no lock needed.
// =========================================================================

struct SensorState {
    std::atomic<int32_t>  wheel_position{0};     // Accumulated raw counts from center
    std::atomic<int32_t>    wheel_velocity{0};     // Filtered counts / sec (signed)
    std::atomic<int32_t>  absolute_raw_angle{0}; // Total absolute raw counts ignoring center offset
    std::atomic<uint32_t> loop_time_avg_us{0};   // Exponential moving average of FFB loop execution time
    std::atomic<uint8_t>  error_flags{0};        // Bit 0: MH, Bit 1: ML, Bit 2: MD missing

    // Error flag bits
    static constexpr uint8_t ERR_MAGNET_HIGH    = 0x01;
    static constexpr uint8_t ERR_MAGNET_LOW     = 0x02;
    static constexpr uint8_t ERR_MAGNET_MISSING = 0x04;
    static constexpr uint8_t ERR_I2C_WATCHDOG   = 0x08;
    static constexpr uint8_t ERR_DESYNC         = 0x10;
    static constexpr uint8_t ERR_RECOVERY_DESYNC = 0x20;
};

// =========================================================================
// FFB Effect Runtime State
// =========================================================================

// Active effect state — one per effect slot
struct EffectSlot {
    // From Set Effect Report (Report ID 1 output)
    USB_FFBReport_SetEffect_Output_Data_t    params;

    // Type-specific parameter blocks (one active at a time per effect)
    USB_FFBReport_SetEnvelope_Output_Data_t  envelope;
    USB_FFBReport_SetCondition_Output_Data_t condition[2];   // Up to 2 axes
    USB_FFBReport_SetPeriodic_Output_Data_t  periodic;
    USB_FFBReport_SetConstantForce_Output_Data_t constant;
    USB_FFBReport_SetRampForce_Output_Data_t ramp;

    uint8_t  conditionBlockCount;  // Number of condition blocks received (1 or 2)

    // Runtime state
    uint8_t  state;                // 0=free, 1=allocated, 2=playing
    uint64_t start_time_us;        // Timestamp when effect started playing
    uint8_t  loop_count;           // Remaining loops (0xFF = infinite)

    static constexpr uint8_t STATE_FREE      = 0;
    static constexpr uint8_t STATE_ALLOCATED = 1;
    static constexpr uint8_t STATE_PLAYING   = 2;
};

// =========================================================================
// Core 0 → Core 1  (Written ONLY by Core 0, read by Core 1)
// Protected by hardware spinlock.
// =========================================================================

struct EffectState {
    EffectSlot   effects[MAX_EFFECTS];
    uint8_t      device_gain;            // 0..255, from Device Gain Report
    bool         actuators_enabled;
    bool         device_paused;

    spin_lock_t* lock;

    void init() {
        int lock_num = spin_lock_claim_unused(true);
        lock = spin_lock_init(lock_num);
        device_gain = 255;
        actuators_enabled = true;
        device_paused = false;
        for (auto& e : effects) {
            e.state = EffectSlot::STATE_FREE;
            e.conditionBlockCount = 0;
        }
    }
};

// =========================================================================
// Calibration Data (shared, written by Core 1 calibration routine)
// =========================================================================

struct CalibrationLUTs {
    // Speed LUT: expected maximum raw velocity at each force level
    // Index corresponds to CAL_FORCE_LEVELS[]
    int32_t  cw_speed[CAL_FORCE_LEVEL_COUNT];
    int32_t  ccw_speed[CAL_FORCE_LEVEL_COUNT];

    // Minimum PWM to overcome static friction (independently per direction)
    uint16_t cw_zero_pwm;
    uint16_t ccw_zero_pwm;

    bool     valid = false;
};

// =========================================================================
// LED Status (shared, written by both cores)
// =========================================================================

struct StatusState {
    std::atomic<uint8_t> status{0};
    std::atomic<uint8_t> min_cycles_remaining{0};
    std::atomic<bool>    clear_pending{false};

    void set(SystemStatus s) {
        uint8_t val = static_cast<uint8_t>(s);
        // Only upgrade priority (higher numeric = higher priority)
        uint8_t current = status.load();
        while (val > current) {
            if (status.compare_exchange_weak(current, val)) {
                // Set minimum display cycles so the LED flashes long enough to see
                // Exclude status codes 1 to 3 (BootWait, MotorSweepsActive, PedalCalActive)
                if (val > 3) {
                    min_cycles_remaining.store(LED_MIN_DISPLAY_CYCLES);
                }
                clear_pending.store(false);
                break;
            }
        }
    }

    void clear(SystemStatus s) {
        uint8_t expected = static_cast<uint8_t>(s);
        
        if (min_cycles_remaining.load() > 0) {
            // Must finish minimum flash. If this is the active code, mark it to be cleared later.
            if (status.load() == expected) {
                clear_pending.store(true);
            }
            return; 
        }
        
        // Only clear if this exact status is currently active
        status.compare_exchange_strong(expected, 0);
    }

    // Called by LED controller at the end of each complete flash cycle
    void decrement_display_cycle() {
        uint8_t c = min_cycles_remaining.load();
        if (c > 0) {
            min_cycles_remaining.store(c - 1);
            // If we just finished the minimum cycles and a clear is pending, clear it now
            if (c - 1 == 0 && clear_pending.load()) {
                status.store(0);
                clear_pending.store(false);
            }
        }
    }

    void force(SystemStatus s) {
        status.store(static_cast<uint8_t>(s));
        min_cycles_remaining.store(0);
        clear_pending.store(false);
    }

    SystemStatus get() const {
        return static_cast<SystemStatus>(status.load());
    }
};

// =========================================================================
// Global Shared State  (instantiated once in main.cpp)
// =========================================================================

struct SharedState {
    SensorState      sensor;
    EffectState      ffb;
    CalibrationLUTs  cal_luts;
    StatusState      led_status;

    std::atomic<int32_t> center_offset{0}; // Read by Core 1 on boot

    // Button state (Core 0 only writes, Core 0 only reads for USB)
    std::atomic<uint16_t> buttons{0};

    // Pedal values (Core 0 only, signed 16-bit for USB HID report)
    std::atomic<int16_t> pedal_accel{-32767};
    std::atomic<int16_t> pedal_brake{-32767};

    // Debug serial: one-shot AGC register read request
    std::atomic<bool>    request_agc_read{false};
    std::atomic<uint8_t> agc_value{0};
};
