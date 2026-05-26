#pragma once
#include "shared_state.h"

// Initialize the USB HID subsystem with a reference to shared state
void usb_hid_init(SharedState& state);

// Send the joystick input report (call from Core 0 main loop)
void usb_hid_send_input_report(SharedState& state);
