#pragma once
#include "shared_state.h"

// Entry point for Core 1
void core1_main();

// Initialize Core 1 hardware (called from main before launching core 1)
void core1_hw_init();

// Initialize Core 1 background interrupts and alarms
void core1_init_interrupts();

// Set the global shared state pointer for Core 1
void core1_set_shared_state(SharedState* state);
