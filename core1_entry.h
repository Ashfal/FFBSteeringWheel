#pragma once
#include "shared_state.h"

// Entry point for Core 1
void core1_main();

// Initialize Core 1 hardware (called from main before launching core 1)
void core1_hw_init();

// Start Core 1 background loop (enables interrupts and alarms)
void core1_start_loop();

// Set the global shared state pointer for Core 1
void core1_set_shared_state(SharedState* state);
