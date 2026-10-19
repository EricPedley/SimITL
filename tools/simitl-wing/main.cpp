/*
 * simitl-wing: headless runner/test harness for the tailsitter wing SITL.
 *
 * Modes:
 *   simitl-wing idle --wing <as_built.json> [--eeprom name.bin] [--dur 12]
 *     Runs the closed-loop sim with a scripted RC schedule and prints a CSV
 *     of motor outputs, servo outputs, elevon deflections, attitude and
 *     rates.  Used for the M1 mixer-sign verification and sanity flights.
 *
 *   simitl-wing forces --wing <as_built.json> --grid <grid.csv> --out <out.csv>
 *     Evaluates the aerodynamic force/moment model on a grid of states for
 *     the C++ <-> Python equivalence test.  No Betaflight, no eeprom.
 *
 *     grid.csv columns:
 *       u,v,w,p,q,r,dl,dr,Tl,Tr
 *     body-axis velocity (u fwd, v right, w down) and rates (rad/s) in the
 *     aerospace frame, elevon deflections in deg (positive TE down),
 *     motor thrusts in N.  Output: Fx,Fy,Fz,Mx,My,Mz in the same frame.
 */

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fmt/format.h>

#include "simitl.h"
#include "sim/tailsitter.h"
#include "sim/state.h"
#include "util/vector_math.h"
#include "wingcfg.h"

using namespace SimITL;

static volatile bool running = true;
static void onSigint(int) { running = false; }

// sim body (x=port, y=dorsal, z=nose) -> aero (x fwd, y right, z down)
// basis that makes the aero frame coincide with the world frame:
// basisA = basis * M^T = I  ->  basis = M
static const mat3 M_SIM_TO_AERO = {
  vec3{ 0.0f, 0.0f, 1.0f},
  vec3{-1.0f, 0.0f, 0.0f},
  vec3{ 0.0f,-1.0f, 0.0f},
};

// ---------------------------------------------------------------------------

static int runForces(const std::string& wingPath, const std::string& gridPath,
                     const std::string& outPath) {
  StateInit si{};
  loadWingConfigInto(wingPath, si);

  SimState ss{};
  TailsitterPhysics tp;
  tp.setSimState(&ss);
  tp.initState(si);   // builds strips, inertia; needs no Betaflight

  std::ifstream in(gridPath);
  if (!in.is_open()) {
    fmt::print(stderr, "cannot open grid file {}\n", gridPath);
    return 1;
  }
  std::ofstream out(outPath);
  out << "u,v,w,p,q,r,dl,dr,Tl,Tr,Fx,Fy,Fz,Mx,My,Mz\n";
  out.precision(10);

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ls(line);
    double u, v, w, p, q, r, dl, dr, Tl, Tr;
    if (!(ls >> u >> v >> w >> p >> q >> r >> dl >> dr >> Tl >> Tr))
      continue;

    vec3 velWorld{(float)u, (float)v, (float)w};
    vec3 omWorld{(float)p, (float)q, (float)r};
    float thrust[2] = {(float)Tr, (float)Tl};   // thrustN[] is {right,left}

    vec3 F, M;
    tp.computeForces(velWorld, omWorld, M_SIM_TO_AERO,
                     (float)dl, (float)dr, thrust, F, M);

    // add the thrust force + moment arms, same as updatePhysics, so the
    // emitted F/M is the total like the python reference produces
    const auto& tsc = ss.stateInit.tailsitter;
    const float motorXAero = tsc.cgX - tsc.motorXAft;
    const vec3 arm[2] = {
      {motorXAero,  tsc.motorY, tsc.motorZ},
      {motorXAero, -tsc.motorY, tsc.motorZ},
    };
    for (int i = 0; i < 2; i++){
      const vec3 fT{thrust[i], 0.0f, 0.0f};
      F = F + fT;
      M = M + cross(arm[i], fT);
    }
    out << u << ',' << v << ',' << w << ',' << p << ',' << q << ',' << r
        << ',' << dl << ',' << dr << ',' << Tl << ',' << Tr
        << ',' << F[0] << ',' << F[1] << ',' << F[2]
        << ',' << M[0] << ',' << M[1] << ',' << M[2] << '\n';
  }
  fmt::print("wrote {}\n", outPath);
  return 0;
}

// ---------------------------------------------------------------------------

// Scripted RC schedule for mixer-sign verification and sanity flight:
//   t < 5.5    disarmed, sticks centred, throttle low (boot grace + cal)
//   5.5-6.5    arm (AUX1 high), throttle still low
//   6.5+       throttle 0.35
//   7.5-8.5    + roll pulse
//   9.0-10.0   + pitch pulse
//   10.5-11.5  + yaw pulse
// The arm switch must be low through boot grace or BF latches
// ARMING_DISABLED_ARM_SWITCH until it toggles low again.
// gThrottle / gPulses let CLI flags retune the schedule for hover-hold
// runs without editing code.
static float gThrottle = 0.35f;
static bool  gPulses = true;
static bool  gProbe = false;
static float gHoldUntil = -1.0f;   // freeze pose until t > this (clean start)
static float gAux2 = 0.0f;         // --aux2: AUX2 (ch6), e.g. 1 = ANGLE mode
static vec3  gKick{0.0f, 0.0f, 0.0f};   // --kick: world rate (rad/s) at release
static bool  gArm = true;
static bool  gLevel = false;       // --level: start in level flight, nose +x
static float gVel = 0.0f;          // --vel: airspeed (m/s) along the nose at release
static float gThr2 = -2.0f;        // --thr2/--thr2-at: throttle step for speed changes
static float gThr2At = 1e9f;          // --noarm: never raise AUX1 (Betaflight
                                   // refuses CLI/MSP config while armed)

static void scheduleRc(float t, StateInput& in) {
  for (int i = 0; i < 4; i++) in.rcData[i] = 0.0f;
  in.rcData[4] = 0.0f;   // aux1
  in.rcData[5] = gAux2;
  in.rcData[6] = 0.0f;
  in.rcData[7] = 0.0f;
  in.rcData[2] = -1.0f;  // throttle low

  if (t >= 5.5f && gArm) in.rcData[4] = 1.0f;   // arm
  if (t >= 6.5f) in.rcData[2] = gThrottle;   // some throttle
  if (t >= gThr2At && gThr2 >= -1.0f) in.rcData[2] = gThr2;
  if (gProbe) {
    // distinctive per-channel values to map the FC channel order
    // ch0=+0.9, ch1=-0.7, ch2=-0.9, ch3=+0.5, ch4=+0.2, ch5=-0.3
    static const float probe[8] = {0.9f, -0.7f, -0.9f, 0.5f, 0.2f, -0.3f, 0.0f, 0.0f};
    for (int i = 0; i < 8; i++) in.rcData[i] = probe[i];
  }
  if (!gPulses) return;
  if (t >= 7.5f && t < 8.5f) in.rcData[0] = 0.5f;   // roll
  if (t >= 9.0f && t < 10.0f) in.rcData[1] = 0.5f;  // pitch
  if (t >= 10.5f && t < 11.5f) in.rcData[3] = 0.5f; // yaw
}

// --frozen: do not integrate; the vehicle hangs at a fixed pose so the BF
// mixer/PID response to sticks and injected rates can be read directly
// without dynamic coupling.  --inj x,y,z injects a constant world-frame
// angular velocity to check rate-loop signs.
static int runIdle(const std::string& wingPath, const std::string& eeprom,
                   float duration, bool verbose, bool frozen,
                   const vec3& injOmega) {
  StateInit si{};
  loadWingConfigInto(wingPath, si);
  if (!eeprom.empty()) {
    std::fill(si.eepromName, si.eepromName + 512, 0);
    memcpy(si.eepromName, eeprom.c_str(), std::min(eeprom.size(), (size_t)511));
    si.eepromName[511] = '\0';
  }

  StateInput in{};
  in.delta = 0.001f;         // 1 ms host-side steps
  in.openLoop = 0;
  in.vbat = 4.2f;
  // Start nose-up (hover pose). mat3 columns are the sim body axes in world
  // coords; sim body is x=port, y=dorsal, z=nose. Put nose up:
  //   col0 (port)   = +x east
  //   col1 (dorsal) = -z       (port x dorsal = nose holds: x0 x (0,0,-1) = +y)
  //   col2 (nose)   = +y up
  // rows of the stored matrix:
  in.rotation[0] = { 1.0f, 0.0f, 0.0f};
  in.rotation[1] = { 0.0f, 0.0f, 1.0f};
  in.rotation[2] = { 0.0f,-1.0f, 0.0f};
  if (gLevel) {   // level flight: nose +x, top up, port -z
    in.rotation[0] = { 0.0f, 0.0f, 1.0f};
    in.rotation[1] = { 0.0f, 1.0f, 0.0f};
    in.rotation[2] = {-1.0f, 0.0f, 0.0f};
    in.position.y = 50.0f;
  }
  in.position = {0.0f, 2.0f, 0.0f};       // 2 m up

  simitl_init(si);

  fmt::print("t,rcR,rcP,rcT,rcY,armed,disFlags,m0,m1,sv0,sv1,sv2,sv3,"
             "elevL,elevR,wx,wy,wz,vx,vy,vz,batt\n");

  const float dt = in.delta;
  float t = 0.0f;
  while (running && t < duration) {
    scheduleRc(t, in);
    const bool holding = frozen || (gHoldUntil >= 0.0f && t < gHoldUntil);
    const bool wasHolding = !frozen && gHoldUntil >= 0.0f && t - dt < gHoldUntil
                            && !holding;
    if (holding) {
      // inject only after boot/cal/arm so gyro calibration sees zero rate
      const bool inj = frozen && (t > 6.5f);
      in.angularVelocity = {inj ? injOmega[0] : 0.0f,
                            inj ? injOmega[1] : 0.0f,
                            inj ? injOmega[2] : 0.0f};
      in.linearVelocity = {0.0f, 0.0f, 0.0f};
    }
    simitl_update(in);
    const StateOutput& o = simitl_get_state();

    if (holding) {
      // keep pose; gyro injection stays
    } else if (wasHolding) {
      // first free step: start from the pose with the --kick disturbance
      in.angularVelocity = {gKick[0], gKick[1], gKick[2]};
      // launch speed along the nose (sim body z = column 2 of the rows)
      in.linearVelocity = {in.rotation[0].z * gVel, in.rotation[1].z * gVel,
                           in.rotation[2].z * gVel};
    } else {
      // feed back orientation + velocities, integrate position host-side
      const quat q{o.orientation.x, o.orientation.y,
                   o.orientation.z, o.orientation.w};
      const mat3 rot = quat_to_mat3(q);
      copy(in.rotation, rot);
      in.angularVelocity = o.angularVelocity;
      in.linearVelocity  = o.linearVelocity;
      in.position.x += o.linearVelocity.x * dt;
      in.position.y += o.linearVelocity.y * dt;
      in.position.z += o.linearVelocity.z * dt;
    }

    if (verbose || (((int)(t * 1000.0f)) % 50) == 0) {
      fmt::print("{:.3f},{:.2f},{:.2f},{:.2f},{:.2f},{},0x{:08x},"
                 "{:.3f},{:.3f},{:.0f},{:.0f},{:.0f},{:.0f},{:.2f},{:.2f},"
                 "{:.2f},{:.2f},{:.2f},{:.3f},{:.3f},{:.3f},{:.2f},"
                 "{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},"
                 "{:.0f},{:.0f},{:.0f},{:.0f},{:.0f},{:.0f},{:.0f},"
                 "{:.0f},{:.0f},{:.0f},{:.0f},"
                 "{:.0f},{:.0f},{:.0f},{:.0f},"
                 "{:.3f},{:.1f}\n",
                 t,
                 in.rcData[0], in.rcData[1], in.rcData[2], in.rcData[3],
                 (int)o.armed, (uint32_t)o.armingDisableFlags,
                 o.motorOutput[0], o.motorOutput[1],
                 o.servoOutput[0], o.servoOutput[1],
                 o.servoOutput[2], o.servoOutput[3],
                 o.elevonDeg[0], o.elevonDeg[1],
                 o.angularVelocity.x, o.angularVelocity.y, o.angularVelocity.z,
                 o.linearVelocity.x, o.linearVelocity.y, o.linearVelocity.z,
                 o.batteryVoltage,
                 o.fcGyro[0], o.fcGyro[1], o.fcGyro[2],
                 o.fcPidSum[0], o.fcPidSum[1], o.fcPidSum[2],
                 o.fcRcCommand[0], o.fcRcCommand[1],
                 o.fcRcCommand[2], o.fcRcCommand[3],
                 o.fcAttitude[0], o.fcAttitude[1], o.fcAttitude[2],
                 o.fcRcData[0], o.fcRcData[1],
                 o.fcRcData[2], o.fcRcData[3],
                 o.fcRcData[4], o.fcRcData[5],
                 o.fcRcData[6], o.fcRcData[7],
                 // height, and the nose's tilt from vertical (deg): sim body
                 // z (nose) in world is column 2 of the row-stored rotation
                 in.position.y,
                 std::acos(std::clamp(in.rotation[1].z, -1.0f, 1.0f)) * 57.2958f);
    }
    t += dt;
  }

  simitl_stop();
  return 0;
}

// --replay <file>: fly the sim on a recorded stick timeline.
// CSV rows: t_seconds,rc0,rc1,...,rc7 with rc in -1..1 in the sim channel
// order (0=roll 1=pitch 2=throttle 3=yaw 4=arm ...).  Replay starts after
// the FC boot grace: sim t=0..5.5 arms like the scripted schedule, then
// sticks come from the file at (t - 5.5) seconds.
static int runReplay(const std::string& wingPath, const std::string& eeprom,
                     const std::string& sticksPath) {
  StateInit si{};
  loadWingConfigInto(wingPath, si);
  if (!eeprom.empty()) {
    std::fill(si.eepromName, si.eepromName + 512, 0);
    memcpy(si.eepromName, eeprom.c_str(), std::min(eeprom.size(), (size_t)511));
    si.eepromName[511] = '\0';
  }

  std::vector<std::array<float, 9>> rows;   // t + rc0..7
  {
    std::ifstream in(sticksPath);
    if (!in.is_open()) {
      fmt::print(stderr, "cannot open sticks file {}\n", sticksPath);
      return 1;
    }
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#' || !isdigit((unsigned char)line[0]))
        continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream ls(line);
      std::array<float, 9> r{};
      if (!(ls >> r[0] >> r[1] >> r[2] >> r[3] >> r[4])) continue;
      for (int i = 5; i < 9; i++) if (!(ls >> r[i])) r[i] = 0.0f;
      rows.push_back(r);
    }
  }
  if (rows.empty()) { fmt::print(stderr, "empty sticks file\n"); return 1; }

  StateInput in{};
  in.delta = 0.001f;
  in.vbat = 4.2f;
  // level cruise start high up, 12 m/s along +x
  in.rotation[0] = { 0.0f, 0.0f, 1.0f};
  in.rotation[1] = { 0.0f, 1.0f, 0.0f};
  in.rotation[2] = {-1.0f, 0.0f, 0.0f};
  in.position = {0.0f, 200.0f, 0.0f};
  const float v0 = (gVel > 0.0f) ? gVel : 12.0f;   // --vel overrides
  in.linearVelocity = {v0, 0.0f, 0.0f};

  simitl_init(si);

  fmt::print("t,armed,m0,m1,sv0,sv1,elevL,elevR,wx,wy,wz,vx,vy,vz,posx,posy,posz,"
             "gyro0,gyro1,gyro2,pid0,pid1,pid2,rcCmd0,rcCmd1,rcCmd2,rcCmd3,"
             "r00,r01,r02,r10,r11,r12,r20,r21,r22,rpm0,rpm1\n");

  const float dt = in.delta;
  float t = 0.0f;
  bool wasArmed = false;
  const float dur = rows.back()[0] + 6.0f;
  while (running && t < dur) {
    for (int i = 0; i < 8; i++) in.rcData[i] = 0.0f;
    in.rcData[2] = -1.0f;
    if (t >= 5.5f) {
      // stick time = sim time - boot grace; linear interpolate rows
      const float st = t - 5.5f;
      size_t lo = 0;
      while (lo + 1 < rows.size() && rows[lo + 1][0] < st) lo++;
      size_t hi = std::min(lo + 1, rows.size() - 1);
      const float span = rows[hi][0] - rows[lo][0];
      const float w = (span > 1e-6f)
                      ? (st - rows[lo][0]) / span : 1.0f;
      for (int i = 0; i < 8; i++)
        in.rcData[i] = rows[lo][i + 1] + (rows[hi][i + 1] - rows[lo][i + 1])
                       * std::min(std::max(w, 0.0f), 1.0f);
      in.rcData[4] = 1.0f;   // armed for the whole replay
      if (!wasArmed) in.rcData[2] = -1.0f;  // BF only arms at throttle-low
    }
    // hold the pose through boot so gyro calibration sees a still airframe
    if (t < 5.5f) {
      in.angularVelocity = {0.0f, 0.0f, 0.0f};
      in.linearVelocity = {0.0f, 0.0f, 0.0f};
    }
    simitl_update(in);
    const StateOutput& o = simitl_get_state();
    if (o.armed) wasArmed = true;

    const quat q{o.orientation.x, o.orientation.y,
                 o.orientation.z, o.orientation.w};
    const mat3 rot = quat_to_mat3(q);
    copy(in.rotation, rot);
    in.angularVelocity = o.angularVelocity;
    in.linearVelocity  = o.linearVelocity;
    if (t >= 5.5f) {
      in.position.x += o.linearVelocity.x * dt;
      in.position.y += o.linearVelocity.y * dt;
      in.position.z += o.linearVelocity.z * dt;
    } else {
      // release into cruise speed when boot ends
      in.linearVelocity = {v0, 0.0f, 0.0f};
    }

    if (((int)(t * 1000.0f)) % 25 == 0) {   // 40 Hz log rate
      fmt::print("{:.3f},{},{:.3f},{:.3f},{:.0f},{:.0f},{:.2f},{:.2f},"
                 "{:.3f},{:.3f},{:.3f},{:.2f},{:.2f},{:.2f},"
                 "{:.2f},{:.2f},{:.2f},"
                 "{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},"
                 "{:.0f},{:.0f},{:.0f},{:.0f},"
                 "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},"
                 "{:.0f},{:.0f}\n",
                 t, (int)o.armed, o.motorOutput[0], o.motorOutput[1],
                 o.servoOutput[0], o.servoOutput[1],
                 o.elevonDeg[0], o.elevonDeg[1],
                 o.angularVelocity.x, o.angularVelocity.y,
                 o.angularVelocity.z,
                 o.linearVelocity.x, o.linearVelocity.y, o.linearVelocity.z,
                 in.position.x, in.position.y, in.position.z,
                 o.fcGyro[0], o.fcGyro[1], o.fcGyro[2],
                 o.fcPidSum[0], o.fcPidSum[1], o.fcPidSum[2],
                 o.fcRcCommand[0], o.fcRcCommand[1],
                 o.fcRcCommand[2], o.fcRcCommand[3],
                 // body->world rows, then per-side rpm
                 in.rotation[0].x, in.rotation[0].y, in.rotation[0].z,
                 in.rotation[1].x, in.rotation[1].y, in.rotation[1].z,
                 in.rotation[2].x, in.rotation[2].y, in.rotation[2].z,
                 o.motorRpm[0], o.motorRpm[1]);
    }
    t += dt;
  }
  simitl_stop();
  return 0;
}

int main(int argc, char** argv) {
  std::signal(SIGINT, onSigint);

  std::string mode = "idle";
  std::string wingPath;
  std::string eeprom;
  std::string gridPath;
  std::string sticksPath;
  std::string outPath = "forces_out.csv";
  float duration = 10.0f;
  bool verbose = false;
  bool frozen = false;
  vec3 injOmega{0.0f, 0.0f, 0.0f};

  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args[0][0] == '-') mode = "idle";
  else { mode = args[0]; args.erase(args.begin()); }

  for (size_t i = 0; i < args.size(); i++) {
    const std::string& a = args[i];
    auto val = [&](const char* name) -> std::string {
      return (i + 1 < args.size()) ? args[++i] : "";
    };
    if (a == "--wing") wingPath = val("wing");
    else if (a == "--eeprom") eeprom = val("eeprom");
    else if (a == "--grid") gridPath = val("grid");
    else if (a == "--sticks") sticksPath = val("sticks");
    else if (a == "--out") outPath = val("out");
    else if (a == "--dur") duration = std::stof(val("dur"));
    else if (a == "-v") verbose = true;
    else if (a == "--frozen") frozen = true;
    else if (a == "--nopulses") gPulses = false;
    else if (a == "--noarm") gArm = false;
    else if (a == "--level") gLevel = true;
    else if (a == "--vel") gVel = std::stof(val("vel"));
    else if (a == "--thr2") gThr2 = std::stof(val("thr2"));
    else if (a == "--thr2-at") gThr2At = std::stof(val("thr2-at"));
    else if (a == "--aux2") gAux2 = std::stof(val("aux2"));
    else if (a == "--kick") {
      std::string s = val("kick");
      sscanf(s.c_str(), "%f,%f,%f", &gKick[0], &gKick[1], &gKick[2]);
    }
    else if (a == "--probe") gProbe = true;
    else if (a == "--thr") gThrottle = std::stof(val("thr"));
    else if (a == "--hold") gHoldUntil = std::stof(val("hold"));
    else if (a == "--inj") {
      std::string s = val("inj");
      sscanf(s.c_str(), "%f,%f,%f", &injOmega[0], &injOmega[1], &injOmega[2]);
    }
  }

  if (wingPath.empty()) {
    fmt::print(stderr,
      "usage: simitl-wing <idle|forces> --wing as_built.json "
      "[--eeprom wing.bin] [--grid g.csv --out o.csv] [--dur s]\n");
    return 1;
  }

  if (mode == "forces") {
    return runForces(wingPath, gridPath, outPath);
  }
  if (mode == "replay") {
    return runReplay(wingPath, eeprom, sticksPath);
  }
  return runIdle(wingPath, eeprom, duration, verbose, frozen, injOmega);
}
