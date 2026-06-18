// =========================================================================
// Debug Serial Console — CDC ACM Single-Char Command Interface
// =========================================================================
// Provides a simple debug console over USB CDC for diagnostics.
// Commands:
//   'c' - Print flash calibration data
//   's' - Print live status (position, velocity, pedals, AGC)
//   'e' - Print and clear the error log
// =========================================================================

#include "debug_serial.h"
#include "tusb.h"
#include "config.h"
#include "shared_state.h"
#include "flash_storage.h"
#include "pico/time.h"
#include <cstdio>
#include <cstring>

// =========================================================================
// Error Log — Lock-free ring buffer (single writer from ISR, single reader)
// =========================================================================

struct ErrorLogEntry {
    uint64_t timestamp_us;
    SystemStatus error_code;
};

static constexpr uint8_t ERROR_LOG_SIZE = 16;
static ErrorLogEntry g_error_log[ERROR_LOG_SIZE];
static volatile uint8_t g_error_log_write_idx = 0;
static uint8_t g_error_log_read_idx = 0;

static SharedState* g_dbg_state = nullptr;

// =========================================================================
// Public API
// =========================================================================

void debug_serial_init(SharedState& state) {
    g_dbg_state = &state;
    memset(g_error_log, 0, sizeof(g_error_log));
    g_error_log_write_idx = 0;
    g_error_log_read_idx = 0;
}

void debug_log_error(SystemStatus error_code) {
    uint8_t idx = g_error_log_write_idx;
    uint8_t next_idx = (idx + 1) % ERROR_LOG_SIZE;
    
    // If full, drop the oldest entry
    if (next_idx == g_error_log_read_idx) {
        g_error_log_read_idx = (g_error_log_read_idx + 1) % ERROR_LOG_SIZE;
    }

    g_error_log[idx].timestamp_us = time_us_64();
    g_error_log[idx].error_code = error_code;
    g_error_log_write_idx = next_idx;
}

// =========================================================================
// Helper: Write a string to CDC
// =========================================================================

static void cdc_print(const char* str) {
    uint32_t len = strlen(str);
    uint32_t sent = 0;
    while (sent < len) {
        uint32_t avail = tud_cdc_write_available();
        if (avail == 0) {
            tud_cdc_write_flush();
            // Brief yield to let USB send
            tud_task();
            continue;
        }
        uint32_t chunk = len - sent;
        if (chunk > avail) chunk = avail;
        tud_cdc_write(str + sent, chunk);
        sent += chunk;
    }
    tud_cdc_write_flush();
}

// =========================================================================
// Helper: Integer to string (avoids printf/snprintf overhead)
// =========================================================================

static char* int_to_str(int32_t val, char* buf) {
    if (val < 0) {
        *buf++ = '-';
        // Handle INT32_MIN edge case
        if (val == -2147483647 - 1) {
            strcpy(buf, "2147483648");
            return buf + 10;
        }
        val = -val;
    }
    // Write digits in reverse
    char tmp[11];
    int i = 0;
    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0) {
            tmp[i++] = '0' + (val % 10);
            val /= 10;
        }
    }
    // Reverse into buf
    for (int j = i - 1; j >= 0; j--) {
        *buf++ = tmp[j];
    }
    *buf = '\0';
    return buf;
}

static char* uint_to_str(uint32_t val, char* buf) {
    char tmp[11];
    int i = 0;
    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0) {
            tmp[i++] = '0' + (val % 10);
            val /= 10;
        }
    }
    for (int j = i - 1; j >= 0; j--) {
        *buf++ = tmp[j];
    }
    *buf = '\0';
    return buf;
}

static char* uint64_to_str(uint64_t val, char* buf) {
    char tmp[21];
    int i = 0;
    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0) {
            tmp[i++] = '0' + (val % 10);
            val /= 10;
        }
    }
    for (int j = i - 1; j >= 0; j--) {
        *buf++ = tmp[j];
    }
    *buf = '\0';
    return buf;
}

// =========================================================================
// Command: 'c' — Print calibration data
// =========================================================================

static void cmd_calibration() {
    if (!g_dbg_state) {
        cdc_print("ERR: No state\r\n");
        return;
    }

    FlashStorage flash;
    FlashCalibrationData data;
    bool valid = flash.load(data);

    if (!valid) {
        cdc_print("=== CALIBRATION: No valid flash data ===\r\n");
        return;
    }

    char buf[64];
    char* p;

    cdc_print("=== CALIBRATION DATA ===\r\n");

    p = buf; strcpy(p, "Center: "); p += 8;
    p = int_to_str(data.center_position, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Accel: "); p += 7;
    p = uint_to_str(data.accel_min, p); *p++ = '-';
    p = uint_to_str(data.accel_max, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Brake: "); p += 7;
    p = uint_to_str(data.brake_min, p); *p++ = '-';
    p = uint_to_str(data.brake_max, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "CW Zero PWM: "); p += 13;
    p = uint_to_str(data.cw_zero_pwm, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "CCW Zero PWM: "); p += 14;
    p = uint_to_str(data.ccw_zero_pwm, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    cdc_print("CW Speed LUT:");
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        p = buf; *p++ = ' ';
        p = int_to_str(data.cw_speed[i], p);
        *p = '\0';
        cdc_print(buf);
    }
    cdc_print("\r\n");

    cdc_print("CCW Speed LUT:");
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        p = buf; *p++ = ' ';
        p = int_to_str(data.ccw_speed[i], p);
        *p = '\0';
        cdc_print(buf);
    }
    cdc_print("\r\n");
}

// =========================================================================
// Command: 's' — Print live status
// =========================================================================

static void cmd_status() {
    if (!g_dbg_state) {
        cdc_print("ERR: No state\r\n");
        return;
    }

    char buf[64];
    char* p;

    cdc_print("=== LIVE STATUS ===\r\n");

    p = buf; strcpy(p, "Position: "); p += 10;
    p = int_to_str(g_dbg_state->sensor.wheel_position.load(), p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    int32_t vel_cps = g_dbg_state->sensor.wheel_velocity.load();
    p = buf; strcpy(p, "Velocity (cps): "); p += 16;
    p = int_to_str(vel_cps, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Buttons: 0x"); p += 11;
    uint16_t btns = g_dbg_state->buttons.load();
    // Simple hex output
    const char hex[] = "0123456789ABCDEF";
    *p++ = hex[(btns >> 12) & 0xF];
    *p++ = hex[(btns >> 8) & 0xF];
    *p++ = hex[(btns >> 4) & 0xF];
    *p++ = hex[btns & 0xF];
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Accel: "); p += 7;
    p = int_to_str(g_dbg_state->pedal_accel.load(), p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Brake: "); p += 7;
    p = int_to_str(g_dbg_state->pedal_brake.load(), p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Err flags: "); p += 11;
    p = uint_to_str(g_dbg_state->sensor.error_flags.load(), p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "LED status: "); p += 12;
    p = uint_to_str(static_cast<uint8_t>(g_dbg_state->led_status.get()), p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Loop time (EMA): "); p += 17;
    p = uint_to_str(g_dbg_state->sensor.loop_time_avg_us.load(), p);
    strcpy(p, " us\r\n");
    cdc_print(buf);

    // Request AGC register read from Core 1
    g_dbg_state->request_agc_read.store(true);

    // Poll for completion (timeout after 10ms)
    uint64_t start = time_us_64();
    while (g_dbg_state->request_agc_read.load()) {
        if (time_us_64() - start > 10000) {
            cdc_print("AGC: timeout\r\n");
            g_dbg_state->request_agc_read.store(false);
            return;
        }
        tud_task(); // Keep USB alive while waiting
    }

    p = buf; strcpy(p, "AGC: "); p += 5;
    p = uint_to_str(g_dbg_state->agc_value.load(), p);
    strcpy(p, "\r\n");
    cdc_print(buf);
}

// =========================================================================
// Command: 'e' — Print and clear error log
// =========================================================================

static const char* error_name(SystemStatus code) {
    switch (code) {
        case SystemStatus::FlashCalMissing: return "FlashCalMissing";
        case SystemStatus::MagnetHigh:      return "MagnetHigh";
        case SystemStatus::MagnetLow:       return "MagnetLow";
        case SystemStatus::MagnetMissing:   return "MagnetMissing";
        case SystemStatus::I2CWatchdogFired:return "I2CWatchdog";
        case SystemStatus::EncoderDesync:   return "Desync";
        case SystemStatus::DesyncAfterRecovery: return "RecoveryDesync";
        case SystemStatus::FlashWriteFailed:return "FlashWriteFailed";
        default: return "Unknown";
    }
}

static void cmd_errors() {
    uint8_t write_idx = g_error_log_write_idx;
    uint8_t read_idx = g_error_log_read_idx;

    if (read_idx == write_idx) {
        cdc_print("=== ERROR LOG: Empty ===\r\n");
        return;
    }

    cdc_print("=== ERROR LOG ===\r\n");

    char buf[64];
    char* p;
    int count = 0;

    while (read_idx != write_idx && count < ERROR_LOG_SIZE) {
        ErrorLogEntry& e = g_error_log[read_idx];
        p = buf;
        *p++ = '[';
        p = int_to_str(static_cast<uint32_t>(e.timestamp_us / 1000), p);
        strcpy(p, "ms | Code "); p += 10;
        p = uint_to_str(static_cast<uint8_t>(e.error_code), p);
        strcpy(p, " ("); p += 2;
        strcpy(p, error_name(e.error_code)); p += strlen(error_name(e.error_code));
        strcpy(p, ")\r\n"); p += 3;
        cdc_print(buf);

        read_idx = (read_idx + 1) % ERROR_LOG_SIZE;
        count++;
    }

    // Clear the log
    g_error_log_read_idx = write_idx;

    p = buf; strcpy(p, "Total: "); p += 7;
    p = int_to_str(count, p);
    strcpy(p, " entries (cleared)\r\n");
    cdc_print(buf);
}

// =========================================================================
// Main update — called from Core 0 main loop
// =========================================================================

void debug_serial_update() {
    if (!tud_cdc_connected() || !tud_cdc_available()) return;

    char c = (char)tud_cdc_read_char();

    switch (c) {
        case 'c': case 'C': cmd_calibration(); break;
        case 's': case 'S': cmd_status(); break;
        case 'e': case 'E': cmd_errors(); break;
        default:
            cdc_print("Commands: [c]alibration [s]tatus [e]rrors\r\n");
            break;
    }
}
