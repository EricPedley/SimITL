// In-firmware MPC for the simITL target: the symforce-generated quad_mpc
// controller runs inside the Betaflight motor-write path whenever the
// BOXMPC ("MPC CONTROL") mode is active (assign it to an AUX channel like any
// other mode).  Feedback uses the same signals a real build could produce: BF's
// filtered gyro, the PID setpoint, and bidirectional-DShot eRPM telemetry.
// The ESC transport delay + 100 Hz filtered-PWM states are modelled locally
// from our own command history since real firmware cannot observe them.

#include "sim_mpc.h"

extern "C" {
#include "platform.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/pid.h"
#include "flight/failsafe.h"
#include "sensors/gyro.h"
#include "rx/rx.h"
#include "drivers/dshot.h"
#include "drivers/motor.h"
}

#include <array>
#include <chrono>
#include <cmath>

#include "quad_mpc.h"

// Defined in target.c; 0..1000 normalized motor output the physics reads.
extern int16_t motorsPwm[MAX_SUPPORTED_MOTORS];

namespace {

quad_mpc::Mpc mpc;
bool initialized = false;
bool active = false;
int cycle = 0;
double lastSolveUs = 0.;
double lastCmd[4] = {0., 0., 0., 0.};
double filtEst[4] = {0., 0., 0., 0.};
double lastColl = 0.;

// Motor-write cycles per MPC solve: the pid loop runs every 125 us sim tick,
// so 16 cycles = the 2 ms control period the plant model was built for.
constexpr int kSolveEvery = 16;
// 100 Hz first-order ESC filter integrated over one 2 ms window:
// 1 - exp(-2*pi*100*0.002) ~ 0.7153
constexpr double kEscLpfAlpha = 0.7153315729789778;
constexpr double kDegToRad = 3.14159265358979323846 / 180.;

}  // namespace

extern "C" void simMpcUpdate(void) {
  if (!initialized) {
    // Probe passes + Riccati table build take ~3ms - do it on the first
    // tick (disarmed/sim start) instead of hitching the flight loop when
    // the mode switch flips on mid-air.
    mpc.init();
    initialized = true;
  }
  active = IS_RC_MODE_ACTIVE(BOXMPC) && ARMING_FLAG(ARMED) &&
           !failsafeIsActive();
  if (!active) {
    // Track the commands another controller is actually applying so a
    // mid-flight switch into MPC is bumpless.
    cycle = 0;
    double applied[4];
    for (int i = 0; i < 4; ++i) {
      applied[i] = motorsPwm[i] / 1000.;
      filtEst[i] = applied[i];
    }
    if (initialized) mpc.trackCommands(applied);
    return;
  }
  if (cycle++ % kSolveEvery == 0) {
    double gyroRadS[3], refRadS[3], omega[4];
    for (int i = 0; i < 3; ++i) {
      gyroRadS[i] = gyro.gyroADCf[i] * kDegToRad;
      refRadS[i] = getSetpointRate(i) * kDegToRad;
    }
    for (int i = 0; i < 4; ++i) {
      omega[i] = getDshotRpm(i) * quad_mpc::kRpmToRadS;
      filtEst[i] += kEscLpfAlpha * (mpc.delayedCommand(i) - filtEst[i]);
    }
    // Throttle stick -> collective thrust target.  Quadratic map normalized so
    // the stick's hover position is the model's trim command: stick=hover ->
    // coll=1, full stick -> kCollMax (~4x hover, saturates the motors).
    // rcCommand[THROTTLE] is post-channel-map microseconds, so this follows
    // whatever channel ordering the aircraft/eeprom is configured with.
    const double throttle = std::min(
        1., std::max(0., (rcCommand[THROTTLE] - 1000.) / 1000.));
    const double stick_ratio = throttle / quad_mpc::kHoverStick;
    const double collective =
        std::min(quad_mpc::kCollMax, stick_ratio * stick_ratio);
    lastColl = collective;
    const auto t0 = std::chrono::steady_clock::now();
    const std::array<double, 4> cmd =
        mpc.update(gyroRadS, omega, filtEst, refRadS, collective);
    lastSolveUs = std::chrono::duration<double, std::micro>(
                      std::chrono::steady_clock::now() - t0)
                      .count();
    for (int i = 0; i < 4; ++i) lastCmd[i] = cmd[i];
  }
  // Hold the last MPC command every write cycle so the PID output never
  // leaks through between solves.
  for (int i = 0; i < 4; ++i) {
    motorsPwm[i] = static_cast<int16_t>(
        lrint(std::min(1., std::max(0., lastCmd[i])) * 1000.));
  }
}

extern "C" int simMpcActive(void) { return active ? 1 : 0; }
extern "C" double simMpcLastSolveUs(void) { return lastSolveUs; }
extern "C" double simMpcBias(int axis) {
  return initialized && axis >= 0 && axis < 3 ? mpc.bias(axis) : 0.;
}
extern "C" double simMpcColl(void) { return lastColl; }
extern "C" double simMpcUref(int i) {
  return initialized && i >= 0 && i < 4
             ? quad_mpc::kTrimCommand[i] + mpc.last_u_ref()(i, 0)
             : 0.;
}
