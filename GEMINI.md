# FFB Steering Wheel — Project Rules

## Project Overview

Firmware for a Force Feedback (FFB) steering wheel controller targeting Euro Truck Simulator 2. Runs on a Raspberry Pi Pico (RP2040, dual-core Cortex-M0+, no FPU, 264KB RAM, 2MB Flash). Implements USB HID with PID (Physical Interface Device) force feedback over TinyUSB.

## Hardware

- **Motor:** 775 DC motor via BTS7960 H-Bridge (20kHz PWM). Pins: EN (bridged), LPWM, RPWM.
- **Encoder:** AS5600 magnetic encoder on I2C at 100kHz. 1:2 gear ratio (8192 counts/wheel rev). CONF register initialized on boot for low-latency operation (PM=00, SF=10, FTH=100, WD=0).
- **Buttons:** 16 buttons via 2× HCF4021B shift registers on SPI at 100kHz (pull-up: 0 = pressed).
- **Pedals:** 2 analog pedals (0–3V) on ADC (compensated ratiometrically via a VBUS reference on ADC2).
- **LED:** Single LED for status/error flash codes.

**Peripheral mapping:** I2C instance, SPI instance, and ADC channels are derived at compile time from pin numbers in `config.h` using RP2040 GPIO function table formulas. Source files resolve these via `I2C_PORT` / `SPI_PORT` macros — no hardcoded peripheral instances.


## Dual-Core Architecture

```
Core 0 (main.cpp)                    Core 1 (core1_entry.cpp)
├── USB HID (TinyUSB) >1kHz          ├── 1ms alarm → I2C DMA read (AS5600)
├── Decoupled input polling:         ├── DMA ISR → AS5600Parser → FFBProcessor → MotorControl
│   ├── Pedals at 2000Hz (ADC)       ├── Calibration (blocking, runs before ISR enabled)
│   ├── Buttons at 500Hz (SPI DMA)   └── Background watchdog loop
│   └── LED status at 100Hz
├── USB HID reports at 100Hz
├── Boot button wait & flash cal
└── Flash storage (flash_safe_execute)

     SharedState (shared_state.h)
     ├── SensorState      — Core 1 writes (atomics), Core 0 reads
     ├── EffectState       — Core 0 writes (spinlock), Core 1 reads
     ├── CalibrationLUTs   — Core 1 writes during calibration only
     ├── StatusState       — Both cores write (atomic CAS), tracks min display cycles
     └── Inputs            — Core 0 writes (atomics): buttons, pedals, center_offset
     └── DebugSerial       — AGC read request flag and ring buffer for error logs
```

### Data Flow

1. `FFBProcessor` (Physics/Force) → computes a force value in `-10000..+10000`
2. `SharedState` (Memory) → cross-core communication via atomics and spinlocks
3. `MotorControl` (Hardware) → maps force to raw PWM, applies stall governor + protection envelope + static friction compensation

### Boot Sequence

```
Core 0 (main.cpp)                           Core 1 (core1_entry.cpp)
──────────────────                           ──────────────────────
1. board_init, stdio_init_all                (waiting to be launched)
2. Init LED, buttons, pedals, USB HID
3. Load flash calibration data
4. Launch Core 1 ──────────────────────────→ 5. Init I2C (+ AS5600 CONF), motor, parser
6. Wait for button press:                    7. Block on FIFO (waiting for command)
   - Short press → CMD_BOOT_NORMAL (0)
   - Long press / no flash → CMD_RUN_FLASH_CAL (1)

If CMD_BOOT_NORMAL:
   Core 0 sends 0 via FIFO ───────────────→ 8. Apply flash center offset
                                              9. Init FFBProcessor with CalibrationLUTs
                                             10. Apply friction compensation to motor
                                             11. Enable DMA IRQ, create alarm pool
                                             12. Start 1ms alarm + first I2C read
                                             13. Enter watchdog loop

If CMD_RUN_FLASH_CAL:
   Core 0 sends 1 via FIFO ───────────────→ 8. Capture center, run motor sweeps
   Core 0 waits for Ack                     9. Ack via FIFO, then halt
   Core 0 runs pedal calibration
   Core 0 saves all data to flash
   Core 0 triggers watchdog reboot
```

**Critical:** DMA interrupts must NOT be enabled before calibration completes. The calibration routine uses synchronous polling of the DMA completion flag, and the ISR would consume those flags.

### Core 0 Main Loop Timing

- `tud_task()` runs every iteration (>1kHz USB polling)
- Pedal sampling (ADC) at 2000Hz (every 0.5ms)
- Button sampling (SPI DMA) at 500Hz (every 2ms) — provides 8ms debounce window (4-read)
- LED status updates at 100Hz (every 10ms)
- USB HID input reports at 100Hz (every 10ms)
- `sleep_us(250)` per iteration to prevent pegging Core 0

## Core Development Rules

### 1. Integer Math & Raw Units Only

The RP2040 has no FPU. **Do not use floats.** All physical quantities must be expressed in raw hardware units:
- Angles: raw encoder counts (4096/encoder rev, 8192/wheel rev)
- PWM: raw duty cycle values (0 to `PWM_WRAP`, currently 6249)
- Force: normalized integer range `-10000..+10000`
- Velocity: raw encoder counts per millisecond

Convert all human-readable concepts (degrees, RPM, percentages) into `constexpr` raw units in `config.h`.

### 2. Memory Safety

- **Single writer per data path.** Each shared variable has exactly one core that writes to it.
- **Core 1 → Core 0:** Use `std::atomic` (no lock needed for single-writer scalars on Cortex-M0+).
- **Core 0 → Core 1:** Use hardware spinlocks (`spin_lock_blocking` / `spin_unlock`) for multi-field structs like `EffectState`.
- **Inter-core commands:** Use Pico SDK FIFO queues (`multicore_fifo_push/pop_blocking`).
- **LED status (both cores):** Use `compare_exchange_weak` for priority-based atomic updates.

### 3. Global Configuration (`config.h`)

All tunable parameters live in `config.h`. This includes:
- `constexpr` GPIO pin assignments for all peripherals
- PWM frequency, wrap values, and software limits
- Motor safety constants (dead-time, stall thresholds, PWM caps, protection envelope)
- Sensor parameters (I2C address, register map, CONF values, glitch thresholds)
- Calibration force levels and sweep distances
- Dynamic damping tuning parameters
- `enum class SystemStatus` defining all LED flash codes

**Do not scatter magic numbers in implementation files.** If a value is tunable, it belongs in `config.h`.

### 4. USB HID Descriptors

**Do NOT generate or modify** the HID report descriptor or PID FFB structs. Use `#include "usb_ffb_descriptors.h"` which contains the exact `hid_report_descriptor[]` and all `__attribute__((packed))` structs required for DirectInput compliance.

### 5. Build Verification

**The project must compile after every change.** After modifying any source file, build the project and fix all compilation errors before considering the change complete. A change that doesn't compile is not a valid change.

### 6. Documentation Currency

**Any changes to the overall architecture of the project must be reflected in this file (`GEMINI.md`).** This includes changes to data flow, boot sequence, safety invariants, timing, shared state structures, new files, or removed files. This file must always accurately describe the current state of the firmware.

## Key Files

| File | Responsibility |
|------|---------------|
| `config.h` | All constants, pin assignments, enums |
| `shared_state.h` | Cross-core shared memory structures |
| `core1_entry.cpp/.h` | Core 1 boot, DMA ISR (FFB loop), watchdog |
| `main.cpp` | Core 0 boot, USB task, inputs, LED, boot/cal logic |
| `motor_control.cpp/.h` | BTS7960 driver, dead-time, stall governor, protection envelope, friction compensation |
| `ffb_processor.cpp/.h` | FFB effect engine (integer-only math), LUT-based damper/friction, dynamic damping |
| `i2c_dma.cpp/.h` | Non-blocking I2C DMA driver, AS5600 CONF init, bus recovery |
| `as5600_parser.cpp/.h` | Encoder position/velocity, glitch filter, dead-reckoning, desync detection |
| `calibration.cpp/.h` | Static friction + speed LUT calibration sweeps, center capture |
| `usb_hid.cpp/.h` | TinyUSB HID callbacks, PID report routing, effect management |
| `usb_descriptors.cpp` | USB device/config/HID/string descriptors, composite HID+CDC assembly |
| `usb_ffb_descriptors.h` | Raw PID HID descriptor + packed structs (DO NOT EDIT) |
| `debug_serial.cpp/.h` | USB CDC single-char command CLI for live status, error logs, and calibration dumps |
| `pedal_reader.cpp/.h` | ADC reading, VBUS ratiometric compensation, spike rejection, signed 16-bit scaling |
| `button_reader.cpp/.h` | SPI DMA reading, 4-read debounce |
| `led_controller.cpp/.h` | Non-blocking LED flash code state machine with minimum display guarantees |
| `flash_storage.cpp/.h` | Flash read/write with CRC32, `flash_safe_execute` for multicore safety |
| `tusb_config.h` | TinyUSB configuration (endpoint sizes, HID buffer) |

## Safety Invariants

1. **`MotorControl` is the sole owner of the BTS7960 pins.** No other code may touch PWM or EN GPIOs.
2. **Dead-time insertion** is mandatory on every direction change to prevent H-bridge shoot-through. Direction memory is preserved across zero-force intervals so dead-time fires correctly on reversal.
3. **Stall protection governor** dynamically clamps PWM based on velocity and direction:
   - **Stalled (v=0):** Capped at `STALL_PWM_MAX` (~36% duty).
   - **Moving forward (with motor):** Linearly scales from `STALL_PWM_MAX` to `FORWARD_MAX_PWM` over `0..FORWARD_VELOCITY_THRESHOLD`.
   - **Moving backward (fighting motor):** Linearly decreases from `STALL_PWM_MAX` to `BACKWARDS_PWM_MAX` (0) over `0..BACKWARDS_VELOCITY_THRESHOLD`.
4. **Protection envelope (soft speed limiter)** prevents dangerous wheel speeds. When velocity exceeds `VELOCITY_FADE_START` (~110 RPM), forward PWM fades linearly to 0 at `MAX_SAFE_VELOCITY` (~140 RPM). Braking/damping forces are not limited.
5. **Static friction compensation** adds a calibrated `cw_zero_pwm` / `ccw_zero_pwm` offset to all force commands so the motor breaks static friction at any non-zero force level.
6. **Electronic end-stops** in `FFBProcessor` apply a proportional reverse spring force when the wheel exceeds `MAX_HALF_ANGLE_COUNTS`, short-circuiting all other effect processing.
7. **Dynamic damping (overpower detection)** in `FFBProcessor`: if the wheel velocity exceeds the expected speed for the commanded force (from calibration LUTs + safety margin), a damping counter-force is applied to prevent the wheel from overshooting.
8. **I2C watchdog** stops the motor and attempts aggressive bus recovery (SCL bit-bang + peripheral reinit + AS5600 CONF rewrite) if the 1ms DMA loop stalls. The 1ms alarm is cancelled during bus recovery to prevent `start_read()` racing with the reset.
9. **Sensor error tolerance:** Transient I2C/EMI dropouts (missing MD bit) are ignored for up to `MAGNET_ERROR_TOLERANCE_FRAMES`. If the error persists, the DMA ISR stops the motor, skips FFB processing, logs to the debug buffer, and sets the LED error code.
10. **LED visibility guarantees:** Transient status codes are guaranteed to flash for at least `LED_MIN_DISPLAY_CYCLES` complete sequences before a self-clearing condition can turn the LED back to solid ON.
11. **Glitch filter with dead-reckoning:** If the AS5600 reports an impossible position jump (delta > `MAX_PHYSICAL_DELTA`), the parser extrapolates position from the last known velocity instead of accepting the bad data. After 10 consecutive impossible jumps, a fatal `EncoderDesync` error stops the motor and halts the I2C loop.
12. **Recovery desync detection:** On the first read after an I2C watchdog recovery, if the delta exceeds `MAX_PHYSICAL_DELTA`, a fatal `DesyncAfterRecovery` error is raised rather than trusting potentially stale data.
13. **Flash writes use `flash_safe_execute`** to safely pause Core 1 and prevent XIP conflicts during erase/program operations.

## Timing Constraints

- **1ms:** I2C DMA read cycle (alarm-triggered)
- **2ms:** Watchdog timeout (motor kill if DMA ISR hasn't fired)
- **0.5ms / 2ms:** Decoupled input sampling intervals (pedals at 2000Hz, buttons at 500Hz)
- **10ms:** USB HID input report rate (100Hz)
- **50µs:** Dead-time between H-bridge direction changes
- **250µs:** Core 0 main loop sleep (keeps `tud_task()` responsive at >1kHz)
- Use `time_us_64()` for all duration/delta calculations (never `time_us_32()` — wraps at 71.5 minutes)
