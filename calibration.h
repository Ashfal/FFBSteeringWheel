#pragma once
#include "shared_state.h"
#include "i2c_dma.h"
#include "motor_control.h"
#include "as5600_parser.h"

// Run the startup calibration sweep.
// This is called synchronously by Core 1 before starting the real-time loop.
// It will sweep the wheel CW and CCW at various PWM levels to measure
// maximum speed, populate the LUTs, and find static friction zero points.
void run_calibration(SharedState* state, I2CDMA& i2c, MotorControl& motor, AS5600Parser& parser);
