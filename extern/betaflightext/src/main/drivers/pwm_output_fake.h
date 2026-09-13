#pragma once

#include <stdint.h>

//#include "drivers/pwm_output.h"

//extern int16_t motorsPwm[MAX_SUPPORTED_MOTORS];

// Emulated bidirectional DSHOT telemetry (implemented in
// target/simITL/target.c next to the simulated motor device).
// Report one motor's electrical RPM (eRPM) telemetry frame; call
// sitlDshotTelemetryFeedDone() once all motors have been reported to mark the
// frame ready for updateDshotTelemetry().
void sitlDshotTelemetryFeedErpm(uint8_t motorIndex, float erpm);
void sitlDshotTelemetryFeedDone(void);
