#pragma once

// =========================================================================
// TinyUSB Configuration for FFB Steering Wheel
// =========================================================================

#define CFG_TUSB_RHPORT0_MODE    OPT_MODE_DEVICE
#define CFG_TUSB_OS              OPT_OS_PICO

// Enable Device stack
#define CFG_TUD_ENABLED          1

// Class drivers
#define CFG_TUD_HID              1
#define CFG_TUD_CDC              0
#define CFG_TUD_MSC              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0

// HID endpoint buffer size — must accommodate largest report
#define CFG_TUD_HID_EP_BUFSIZE   64

// Endpoint 0 size
#define CFG_TUD_ENDPOINT0_SIZE   64
