# Role
You are an expert embedded C++ developer specializing in USB HID protocols, DirectInput Force Feedback (PID Page 0x0F), and the Raspberry Pi Pico C++ SDK (TinyUSB).

# Task
I am building a Force Feedback Steering Wheel using an RP2040. I need you to extract the raw HID report descriptor and the corresponding PID Force Feedback structs from two reference projects and synthesize them into a single, clean `usb_ffb_descriptors.h` header file.

# Reference Material
Please analyze the following local files from my cloned repositories:
1. **ArduinoJoystickWithFFBLibrary Source:** `/home/matias/GitHub/ArduinoJoystickWithFFBLibrary/src/FFBDescriptor.h`
2. **picowinder USB Implementation:** `/home/matias/GitHub/picowinder/`

# Output Requirements
Generate the complete `usb_ffb_descriptors.h` file following these strict rules:

1. **Zero Hex Modification:** Extract the massive HID report descriptor array (the one mapping the Generic Desktop Multi-axis Controller and the PID Page 0x0F effects) exactly as it exists in the source. Do not alter, optimize, or recalculate any hex values, logical minimums/maximums, or report counts.
2. **Standardize for RP2040/TinyUSB:** Remove all AVR-specific or Arduino-specific macros (e.g., `PROGMEM`). Store the descriptor array as standard `const uint8_t hid_report_descriptor[]`.
3. **Strict Struct Padding:** Extract the struct definitions for every PID FFB effect report. You MUST enforce strict memory alignment by appending `__attribute__((packed))` to every single struct. 
4. **Report IDs:** Clearly define an `enum` or a list of `constexpr uint8_t` for all the Report IDs mapped in the descriptor to be used later in the TinyUSB `tud_hid_set_report_cb` callback.
5. **Preserve Critical Knowledge (The Comment Filter):** Scan the source files for developer comments and warnings. 
    - **KEEP:** Extract any comments regarding DirectInput quirks, HID byte alignment, data ranges, or specific FFB effect limitations. Format these as clean C++ block comments directly above the relevant structs or hex lines.
    - **DISCARD:** Ignore any comments specifically complaining about AVR/Arduino memory limits, hardware bugs, or library legacy issues, as these do not apply to the RP2040.
6. **Prune the Fat:** Only include the includes (e.g., `<stdint.h>`), the descriptor array, the Report ID definitions, the packed structs, and the filtered comments. Strip out any unrelated boilerplate or implementation logic. Use standard `#pragma once` at the top.