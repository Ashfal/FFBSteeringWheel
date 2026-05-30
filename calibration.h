#pragma once
#include "shared_state.h"
#include "i2c_dma.h"
#include "motor_control.h"
#include "as5600_parser.h"

// Runs the full flash calibration logic (captures center, then runs sweeps)
void run_calibration(SharedState* state, I2CDMA& i2c, MotorControl& motor, AS5600Parser& parser);
