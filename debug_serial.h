#pragma once
#include <cstdint>
#include "config.h"

// Forward declaration
struct SharedState;

// Initialize the debug serial module
void debug_serial_init(SharedState* state);

// Call from Core 0 main loop — checks for incoming CDC bytes and responds
void debug_serial_update();

// Log an error from any context (ISR-safe, lock-free)
void debug_log_error(SystemStatus error_code);
