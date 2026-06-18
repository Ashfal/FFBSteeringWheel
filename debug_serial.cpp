// =========================================================================
// Debug Serial Console — CDC ACM Line-Buffered CLI
// =========================================================================
// Provides a simple debug console over USB CDC for diagnostics.
// =========================================================================

#include "debug_serial.h"
#include "tusb.h"
#include "config.h"
#include "shared_state.h"
#include "flash_storage.h"
#include "pedal_reader.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

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
static PedalReader* g_dbg_pedals = nullptr;
static FlashStorage* g_dbg_flash = nullptr;

static char g_line_buf[64];
static uint8_t g_line_len = 0;
static bool g_prompt_printed = false;

// =========================================================================
// Public API
// =========================================================================

void debug_serial_init(SharedState& state, PedalReader& pedals, FlashStorage& flash) {
    g_dbg_state = &state;
    g_dbg_pedals = &pedals;
    g_dbg_flash = &flash;
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

// =========================================================================
// Command: Print live calibration data
// =========================================================================

static void cmd_calibration() {
    if (!g_dbg_state || !g_dbg_pedals) {
        cdc_print("ERR: No state\r\n");
        return;
    }

    char buf[64];
    char* p;

    cdc_print("=== LIVE CALIBRATION DATA ===\r\n");

    p = buf; strcpy(p, "Center: "); p += 8;
    p = int_to_str(g_dbg_state->center_offset.load(), p);
    strcpy(p, "\r\n");
    cdc_print(buf);
    
    p = buf; strcpy(p, "Wheel Angle (deg): "); p += 19;
    p = int_to_str(g_dbg_state->wheel_angle_deg.load(), p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    uint16_t amin, amax, bmin, bmax;
    g_dbg_pedals->get_calibration(amin, amax, bmin, bmax);

    p = buf; strcpy(p, "Accel: "); p += 7;
    p = uint_to_str(amin, p); *p++ = '-';
    p = uint_to_str(amax, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "Brake: "); p += 7;
    p = uint_to_str(bmin, p); *p++ = '-';
    p = uint_to_str(bmax, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "CW Zero PWM: "); p += 13;
    p = uint_to_str(g_dbg_state->cal_luts.cw_zero_pwm, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    p = buf; strcpy(p, "CCW Zero PWM: "); p += 14;
    p = uint_to_str(g_dbg_state->cal_luts.ccw_zero_pwm, p);
    strcpy(p, "\r\n");
    cdc_print(buf);

    cdc_print("CW Speed LUT:");
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        p = buf; *p++ = ' ';
        p = int_to_str(g_dbg_state->cal_luts.cw_speed[i], p);
        *p = '\0';
        cdc_print(buf);
    }
    cdc_print("\r\n");

    cdc_print("CCW Speed LUT:");
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        p = buf; *p++ = ' ';
        p = int_to_str(g_dbg_state->cal_luts.ccw_speed[i], p);
        *p = '\0';
        cdc_print(buf);
    }
    cdc_print("\r\n");
}

// =========================================================================
// Command: Save calibration data
// =========================================================================

static void cmd_save_calibration() {
    if (!g_dbg_state || !g_dbg_pedals || !g_dbg_flash) return;
    
    cdc_print("Suspending Core 1...\r\n");
    multicore_fifo_push_blocking(CORE1_CMD_SUSPEND);
    uint32_t ack = multicore_fifo_pop_blocking();
    if (ack != CORE1_CMD_ACK) {
        cdc_print("ERR: Core 1 did not ACK suspend\r\n");
        return;
    }

    cdc_print("Saving to flash...\r\n");
    FlashCalibrationData data;
    data.center_position = g_dbg_state->center_offset.load();
    data.wheel_angle_deg = g_dbg_state->wheel_angle_deg.load();
    g_dbg_pedals->get_calibration(data.accel_min, data.accel_max, data.brake_min, data.brake_max);
    data.cw_zero_pwm = g_dbg_state->cal_luts.cw_zero_pwm;
    data.ccw_zero_pwm = g_dbg_state->cal_luts.ccw_zero_pwm;
    for (int i = 0; i < CAL_FORCE_LEVEL_COUNT; i++) {
        data.cw_speed[i] = g_dbg_state->cal_luts.cw_speed[i];
        data.ccw_speed[i] = g_dbg_state->cal_luts.ccw_speed[i];
    }

    // Core 1 is spinning, but we still pass core1_running=true so flash_safe_execute 
    // puts Core 1 into a lockout state cleanly before pausing Core 0 interrupts.
    bool ok = g_dbg_flash->save(data, true);

    cdc_print(ok ? "Save OK\r\n" : "Save FAILED\r\n");

    cdc_print("Resuming Core 1...\r\n");
    multicore_fifo_push_blocking(CORE1_CMD_RESUME);
    multicore_fifo_pop_blocking(); // Wait for ACK
    cdc_print("Done.\r\n");
}

// =========================================================================
// Command: Print live status
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
// Command: Print and clear error log
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
// Command Parser
// =========================================================================

static void print_help() {
    cdc_print("Commands:\r\n");
    cdc_print("  s               - Print live status\r\n");
    cdc_print("  c               - Print live calibration data\r\n");
    cdc_print("  e               - Print error log\r\n");
    cdc_print("  cs <var> <val>  - Set cal variable (amin, amax, bmin, bmax, cwz, ccwz, center, angle)\r\n");
    cdc_print("  cs <lut> <idx> <val> - Set LUT value (lut: cw, ccw; idx: 0-4)\r\n");
    cdc_print("  cw              - Write live calibration data to flash\r\n");
    cdc_print("  help            - Print this help\r\n");
}

static void process_command(char* cmd) {
    char* argv[5];
    int argc = 0;
    char* p = cmd;
    while (*p) {
        while (*p == ' ') p++;
        if (!*p) break;
        argv[argc++] = p;
        if (argc >= 5) break;
        while (*p && *p != ' ') p++;
        if (*p) {
            *p = '\0';
            p++;
        }
    }

    if (argc == 0) return;

    if (strcmp(argv[0], "s") == 0) {
        cmd_status();
    } else if (strcmp(argv[0], "c") == 0) {
        cmd_calibration();
    } else if (strcmp(argv[0], "e") == 0) {
        cmd_errors();
    } else if (strcmp(argv[0], "help") == 0) {
        print_help();
    } else if (strcmp(argv[0], "cw") == 0) {
        cmd_save_calibration();
    } else if (strcmp(argv[0], "cs") == 0) {
        if (argc >= 3) {
            const char* var = argv[1];
            if (strcmp(var, "cw") == 0 || strcmp(var, "ccw") == 0) {
                if (argc == 4) {
                    int idx = atoi(argv[2]);
                    int32_t val = atoi(argv[3]);
                    if (idx >= 0 && idx < CAL_FORCE_LEVEL_COUNT) {
                        if (strcmp(var, "cw") == 0) g_dbg_state->cal_luts.cw_speed[idx] = val;
                        else g_dbg_state->cal_luts.ccw_speed[idx] = val;
                        cdc_print("LUT updated.\r\n");
                    } else {
                        cdc_print("ERR: Invalid index\r\n");
                    }
                } else {
                    cdc_print("ERR: Usage: cs <lut> <idx> <val>\r\n");
                    print_help();
                }
            } else {
                int32_t val = atoi(argv[2]);
                if (strcmp(var, "cwz") == 0) g_dbg_state->cal_luts.cw_zero_pwm = val;
                else if (strcmp(var, "ccwz") == 0) g_dbg_state->cal_luts.ccw_zero_pwm = val;
                else if (strcmp(var, "center") == 0) g_dbg_state->center_offset.store(val);
                else if (strcmp(var, "angle") == 0) {
                    g_dbg_state->wheel_angle_deg.store(val);
                    int32_t half_deg = val / 2;
                    int32_t max_half_angle_counts = (half_deg * WHEEL_COUNTS_PER_REV) / 360;
                    g_dbg_state->max_half_angle_counts.store(max_half_angle_counts);
                } else if (strcmp(var, "amin") == 0 || strcmp(var, "amax") == 0 || 
                           strcmp(var, "bmin") == 0 || strcmp(var, "bmax") == 0) {
                    uint16_t amin, amax, bmin, bmax;
                    g_dbg_pedals->get_calibration(amin, amax, bmin, bmax);
                    if (strcmp(var, "amin") == 0) amin = val;
                    else if (strcmp(var, "amax") == 0) amax = val;
                    else if (strcmp(var, "bmin") == 0) bmin = val;
                    else if (strcmp(var, "bmax") == 0) bmax = val;
                    g_dbg_pedals->set_calibration(amin, amax, bmin, bmax);
                } else {
                    cdc_print("ERR: Unknown variable\r\n");
                    print_help();
                    return;
                }
                cdc_print("Updated.\r\n");
            }
        } else {
            cdc_print("ERR: Usage: cs <var> <val>\r\n");
            print_help();
        }
    } else {
        cdc_print("ERR: Unknown command.\r\n");
        print_help();
    }
}

// =========================================================================
// Main update — called from Core 0 main loop
// =========================================================================

static uint64_t g_connected_timestamp = 0;
static bool g_was_connected = false;

void debug_serial_update() {
    bool connected = tud_cdc_connected();
    if (!connected) {
        g_prompt_printed = false;
        g_line_len = 0;
        g_was_connected = false;
        return;
    }

    if (!g_was_connected) {
        g_was_connected = true;
        g_connected_timestamp = time_us_64();
    }

    if (!g_prompt_printed) {
        // Wait 500ms after connection before sending the first prompt
        // to give the host terminal software time to initialize.
        if (time_us_64() - g_connected_timestamp > 500000) {
            cdc_print("\r\nffbserial: ");
            g_prompt_printed = true;
        }
    }

    if (!tud_cdc_available()) return;

    char c = (char)tud_cdc_read_char();

    if (c == '\r' || c == '\n') {
        cdc_print("\r\n");
        g_line_buf[g_line_len] = '\0';
        if (g_line_len > 0) {
            process_command(g_line_buf);
            g_line_len = 0;
        }
        if (g_prompt_printed) {
            cdc_print("ffbserial: ");
        }
    } else if (c == '\b' || c == 0x7F) { // Backspace or DEL
        if (g_line_len > 0) {
            g_line_len--;
            cdc_print("\b \b");
        }
    } else if (g_line_len < sizeof(g_line_buf) - 1) {
        // Allow alphanumeric, space, minus
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || 
            (c >= '0' && c <= '9') || c == ' ' || c == '-') {
            g_line_buf[g_line_len++] = c;
            char echo[2] = {c, '\0'};
            cdc_print(echo);
        }
    }
}
