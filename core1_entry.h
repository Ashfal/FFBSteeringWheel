#pragma once
#include "shared_state.h"

// Entry point for Core 1
void core1_main();

// Initialize Core 1 hardware (called from main before launching core 1)
void core1_init();

// Set the global shared state pointer for Core 1
void core1_set_shared_state(SharedState* state);
