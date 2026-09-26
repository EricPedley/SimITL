#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#include <io.h>
#define bridge_dup _dup
#define bridge_dup2 _dup2
#define bridge_fileno _fileno
#define bridge_fdopen _fdopen
#else
#include <unistd.h>
#define bridge_dup dup
#define bridge_dup2 dup2
#define bridge_fileno fileno
#define bridge_fdopen fdopen
#endif

#include "config.h"
#include "sim/sim.h"
#include "quad_mpc.h"

namespace {
constexpr double tickSeconds = 0.000125;
constexpr double radToDeg = 57.29577951308232;

std::string quote(const std::string& text) {
  std::ostringstream out;
  out << '"';
  for (unsigned char c : text) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 32 || c >= 127) {
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
    } else out << c;
  }
  out << '"';
  return out.str();
}

double number(const json::JsonObject& value, const char* name) {
  if (value.type != json::JsonObject::Number || !std::isfinite(value.numberValue)) {
    throw std::runtime_error(std::string(name) + " must be a finite number");
  }
  return value.numberValue;
}

template<size_t N>
std::array<float, N> channels(const json::JsonObject& value, float low, float high) {
  if (value.type != json::JsonObject::Array || value.elements.size() != N) {
    throw std::runtime_error("Wrong channel array length");
  }
  std::array<float, N> result{};
  for (size_t i = 0; i < N; i++) {
    const double v = number(value.elements[i], "channel");
    if (v < low || v > high) throw std::runtime_error("Channel out of range");
    result[i] = static_cast<float>(v);
  }
  return result;
}

uint32_t ticksFor(const json::JsonObject& value) {
  const double dt = number(value, "dt_s");
  if (dt < tickSeconds || dt > 0.1) {
    throw std::runtime_error("dt_s must be in [0.000125,0.1]");
  }
  const double ticks = dt / tickSeconds;
  if (std::abs(ticks - std::round(ticks)) > 1e-7) {
    throw std::runtime_error("dt_s must be a multiple of 0.000125");
  }
  return static_cast<uint32_t>(std::llround(ticks));
}

template<typename Container>
void arrayJson(std::ostream& out, const Container& values) {
  out << '[';
  bool first = true;
  for (auto v : values) {
    if (!std::isfinite(v)) throw std::runtime_error("Non-finite simulation observation");
    if (!first) out << ',';
    out << v;
    first = false;
  }
  out << ']';
}

std::array<float, 3> xyz(const Vec3F& v) { return {v.x, v.y, v.z}; }

std::array<double, 3> bfRates(const SimITL::vec3& v) {
  return {-v[2] * radToDeg, v[0] * radToDeg, -v[1] * radToDeg};
}

class Bridge {
 public:
  SimITL::Sim& sim = SimITL::Sim::getInstance();
  bool initialized = false;
  bool external = true;
  bool pinVelocity = false;
  bool useMpc = false;
  double lastSolveUs = 0.;
  quad_mpc::Mpc mpc;
  uint64_t elapsedTicks = 0;
  uint64_t bootTimeUs = 0;
  unsigned seed = 1;
  std::filesystem::path workdir;

  void tick(bool held, double delta = tickSeconds) {
    StateInput input = sim.mSimState.stateInput;
    input.delta = static_cast<float>(delta);
    input.openLoop = 0;
    input.pinVelocity = pinVelocity ? 1 : 0;
    input.contact = held ? 1 : 0;
    if (held) {
      input.rotation[0] = {1, 0, 0};
      input.rotation[1] = {0, 1, 0};
      input.rotation[2] = {0, 0, 1};
      input.angularVelocity = {};
      input.linearVelocity = {};
      input.position = {0, 2, 0};
      sim.mSimState.stateInput.angularVelocity = {};
      sim.mSimState.stateInput.linearVelocity = {};
    }
    sim.update(input);
    auto& state = sim.mSimState.stateInput;
    if (!held) {
      state.position.x += state.linearVelocity.x * delta;
      state.position.y += state.linearVelocity.y * delta;
      state.position.z += state.linearVelocity.z * delta;
    }
  }

  void initialize(const json::JsonObject& request) {
    if (initialized) throw std::runtime_error("One init per process; restart for a new seed or config");
    const std::string mode = request.get("mode").asString("external");
    if (mode != "external" && mode != "bf") throw std::runtime_error("mode must be external or bf");
    external = mode == "external";
    const std::string controller = request.get("controller").asString("");
    if (!controller.empty() && controller != "mpc")
      throw std::runtime_error("controller must be mpc");
    useMpc = controller == "mpc";
    if (useMpc && !external)
      throw std::runtime_error("controller=mpc requires external mode");
    const double requestedSeed = request.has("seed") ? number(request.get("seed"), "seed") : 1;
    if (requestedSeed < 0 || requestedSeed > std::numeric_limits<unsigned>::max() ||
        std::floor(requestedSeed) != requestedSeed) throw std::runtime_error("Invalid seed");
    seed = static_cast<unsigned>(requestedSeed);
    pinVelocity = request.has("pin_velocity") && request.get("pin_velocity").asBool();
    const auto configPath = std::filesystem::absolute(request.get("quad_config").asString());
    const auto cfg = loadQuadConfig(configPath.string());
    std::filesystem::path eepromSource;
    if (request.has("eeprom_source")) {
      eepromSource = std::filesystem::absolute(request.get("eeprom_source").asString());
      if (!std::filesystem::is_regular_file(eepromSource)) throw std::runtime_error("eeprom_source is not a file");
    }
    auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    for (unsigned attempt = 0; attempt < 100; attempt++) {
      auto candidate = std::filesystem::temp_directory_path() /
          ("simitl-controller-" + std::to_string(stamp) + "-" + std::to_string(attempt));
      if (std::filesystem::create_directory(candidate)) {
        workdir = candidate;
        break;
      }
    }
    if (workdir.empty()) throw std::runtime_error("Cannot create isolated work directory");
    if (!eepromSource.empty()) std::filesystem::copy_file(eepromSource, workdir / "bridge-eeprom.bin");
    std::filesystem::current_path(workdir);
    std::srand(seed);
    if (useMpc && configPath.stem().string() != quad_mpc::kQuadKey)
      throw std::runtime_error("generated MPC constants are for quad '" +
                               std::string(quad_mpc::kQuadKey) + "', not '" +
                               configPath.stem().string() + "'; regenerate with "
                               "multirotor.optimal_control_symforce");
    if (useMpc) mpc.init();
    StateInit init{};
    cfg.applyTo(init);
    const char eeprom[] = "bridge-eeprom.bin";
    std::copy(std::begin(eeprom), std::end(eeprom), init.eepromName);
    sim.initHeadless(init);
    SimITL::BF::configureBridgeArming();
    StateInput input{};
    cfg.applyInputDefaultsTo(input);
    const std::array<float, 8> rc = {0, 0, -1, 0, -1, -1, -1, -1};
    std::copy(rc.begin(), rc.end(), input.rcData);
    input.rotation[0] = {1, 0, 0};
    input.rotation[1] = {0, 1, 0};
    input.rotation[2] = {0, 0, 1};
    sim.mSimState.stateInput = input;
    sim.setMotorOverride({0, 0, 0, 0});
    for (int i = 0; i < 64000; i++) tick(true);
    if (!SimITL::BF::getControllerTelemetry().calibrated) {
      throw std::runtime_error("Gyro calibration did not complete during 8-second startup");
    }
    sim.mSimState.stateInput.rcData[4] = 1;
    for (int i = 0; i < 800; i++) tick(true);
    if (!SimITL::BF::getControllerTelemetry().armed) {
      throw std::runtime_error("BF did not arm; arming_disabled_flags=" +
          std::to_string(sim.mSimState.armingDisabledFlags));
    }
    if (!external) sim.clearMotorOverride();
    auto& state = sim.mSimState.stateInput;
    state.rotation[0] = {1, 0, 0};
    state.rotation[1] = {0, 1, 0};
    state.rotation[2] = {0, 0, 1};
    state.angularVelocity = {};
    state.linearVelocity = {};
    state.position = {0, 2, 0};
    state.contact = 0;
    bootTimeUs = sim.mSimState.microsPassed;
    initialized = true;
  }

  void step(const json::JsonObject& request) {
    if (!initialized) throw std::runtime_error("Send init first");
    const uint32_t ticks = ticksFor(request.get("dt_s"));
    std::array<float, 8> rc{};
    std::copy(std::begin(sim.mSimState.stateInput.rcData), std::end(sim.mSimState.stateInput.rcData), rc.begin());
    if (request.has("rc")) rc = channels<8>(request.get("rc"), -1, 1);
    std::array<float, 4> pwm{};
    if (external && !useMpc) pwm = channels<4>(request.get("motor_pwm"), 0, 1);
    else if (!external && request.has("motor_pwm")) throw std::runtime_error("motor_pwm is only valid in external mode");
    if (request.has("prop_damage")) {
      // per-motor thrust-loss disturbance, e.g. [0,0.3,0,0]
      const auto dmg = request.get("prop_damage");
      for (int i = 0; i < 4 && i < (int)dmg.elements.size(); i++)
        sim.mSimState.stateInput.propDamage[i] =
            static_cast<float>(number(dmg.elements[i], "damage"));
    }
    std::copy(rc.begin(), rc.end(), sim.mSimState.stateInput.rcData);
    if (useMpc) {
      // Generated quad MPC drives the motors; the request's motor_pwm is
      // ignored.  Feedback = BF's filtered gyro + true motor state.
      const auto& s = sim.mSimState;
      const auto bf = SimITL::BF::getControllerTelemetry();
      double gyro[3], omega[4], filtered[4], ref[3];
      for (int i = 0; i < 3; ++i) {
        gyro[i] = bf.gyroDegS[i] / radToDeg;
        ref[i] = bf.setpointDegS[i] / radToDeg;
      }
      for (int i = 0; i < 4; ++i) {
        omega[i] = s.motorsState[i].rpm * quad_mpc::kRpmToRadS;
        filtered[i] = s.motorsState[i].pwmLowPassFilter.get();
      }
      const auto t0 = std::chrono::steady_clock::now();
      const std::array<double, 4> cmd = mpc.update(gyro, omega, filtered, ref);
      lastSolveUs = std::chrono::duration<double, std::micro>(
                        std::chrono::steady_clock::now() - t0).count();
      std::copy(cmd.begin(), cmd.end(), pwm.begin());
    }
    if (external) sim.setMotorOverride(pwm);
    const uint32_t substeps = (ticks + 7) / 8;
    for (uint32_t i = 0; i < substeps; i++) {
      const uint32_t chunk = std::min<uint32_t>(ticks - i * 8, 8);
      tick(false, chunk * tickSeconds);
    }
    elapsedTicks += ticks;
  }

  std::string observation() const {
    const auto& s = sim.mSimState;
    const auto& state = s.stateInput;
    SimITL::mat3 basis;
    SimITL::copy(basis, state.rotation);
    SimITL::vec3 angular;
    SimITL::copy(angular, state.angularVelocity);
    const auto body = SimITL::xform_inv(basis, angular);
    const auto orientation = SimITL::mat3_to_quat(basis);
    const auto bf = SimITL::BF::getControllerTelemetry();
    std::array<float, 4> pwm{}, rpm{}, filtered{};
    for (int i = 0; i < 4; i++) {
      pwm[i] = s.motorsState[i].pwm;
      rpm[i] = s.motorsState[i].rpm;
      filtered[i] = s.motorsState[i].pwmLowPassFilter.get();
    }
    std::ostringstream out;
    out << std::setprecision(17);
    out << "{\"ok\":true,\"mode\":" << quote(external ? "external" : "bf")
        << ",\"controller\":" << quote(useMpc ? "mpc" : "none")
        << ",\"mpc_solve_us\":" << lastSolveUs
        << ",\"seed\":" << seed << ",\"workdir\":" << quote(workdir.string())
        << ",\"open_loop\":0,\"pin_velocity\":" << (pinVelocity ? "true" : "false")
        << ",\"time_s\":" << elapsedTicks * tickSeconds
        << ",\"sim_time_us\":" << s.microsPassed << ",\"boot_time_us\":" << bootTimeUs
        << ",\"gyro_time_s\":" << (static_cast<double>(elapsedTicks) - 1) * tickSeconds
        << ",\"armed\":" << (s.armed ? "true" : "false")
        << ",\"bf_armed\":" << (bf.armed ? "true" : "false")
        << ",\"gyro_calibrated\":" << (bf.calibrated ? "true" : "false")
        << ",\"arming_disabled_flags\":" << s.armingDisabledFlags
        << ",\"body_rates_deg_s\":";
    arrayJson(out, bfRates(body));
    out << ",\"sensor_gyro_deg_s\":"; arrayJson(out, bfRates(s.gyro));
    out << ",\"gyro_deg_s\":"; arrayJson(out, bf.gyroDegS);
    out << ",\"bf_setpoint_deg_s\":"; arrayJson(out, bf.setpointDegS);
    out << ",\"bf_motor_pwm\":"; arrayJson(out, bf.motorPwm);
    out << ",\"bf_motor_rpm\":"; arrayJson(out, bf.motorRpm);
    out << ",\"bf_motor_erpm\":"; arrayJson(out, bf.motorErpm);
    out << ",\"bf_dshot_telemetry\":" << (bf.dshotTelemetryActive ? "true" : "false");
    out << ",\"bf_rc_data_us\":"; arrayJson(out, bf.rcDataUs);
    out << ",\"bf_rc_command\":"; arrayJson(out, bf.rcCommand);
    out << ",\"bf_raw_setpoint_deg_s\":"; arrayJson(out, bf.rawSetpointDegS);
    out << ",\"bf_attitude_deg\":"; arrayJson(out, bf.attitudeDeg);
    out << ",\"bf_crash_recovery\":" << (bf.crashRecovery ? "true" : "false");
    out << ",\"bf_launch_control\":" << (bf.launchControl ? "true" : "false");
    out << ",\"bf_flight_modes\":" << bf.flightModeFlags;
    out << ",\"bf_pid_setpoint_deg_s\":"; arrayJson(out, bf.pidSetpointDegS);
    out << ",\"mpc_fw_active\":" << (bf.mpcFwActive ? "true" : "false");
    out << ",\"mpc_fw_solve_us\":" << bf.mpcFwSolveUs;
    out << ",\"mpc_fw_bias\":"; arrayJson(out, bf.mpcFwBias);
    out << ",\"mpc_fw_coll\":" << bf.mpcFwColl;
    out << ",\"mpc_fw_uref\":"; arrayJson(out, bf.mpcFwUref);
    out << ",\"bf_maxvel_r\":" << bf.maxVelocityR
        << ",\"bf_iterm_rot\":" << (bf.itermRotation ? 1 : 0)
        << ",\"bf_ac_gain\":" << bf.acGain
        << ",\"bf_acro_trainer\":" << (bf.acroTrainer ? 1 : 0)
        << ",\"bf_pid_enabled\":" << (bf.pidEnabled ? 1 : 0);
    out << ",\"motor_pwm\":"; arrayJson(out, pwm);
    out << ",\"motor_rpm\":"; arrayJson(out, rpm);
    out << ",\"motor_filtered_pwm\":"; arrayJson(out, filtered);
    out << ",\"battery_voltage_v\":" << s.batteryState.batVoltageSag
        << ",\"battery_open_circuit_voltage_v\":" << s.batteryState.batVoltage
        << ",\"battery_capacity_mah\":" << s.batteryState.batCapacity
        << ",\"battery_current_a\":" << s.batteryState.amperage
        << ",\"linear_velocity_world_m_s\":";
    arrayJson(out, xyz(state.linearVelocity));
    out << ",\"position_world_m\":"; arrayJson(out, xyz(state.position));
    out << ",\"orientation_xyzw\":";
    arrayJson(out, std::array<float, 4>{orientation[0], orientation[1], orientation[2], orientation[3]});
    out << '}';
    return out.str();
  }
};
}

int main() {
  std::fflush(stdout);
  FILE* protocol = bridge_fdopen(bridge_dup(bridge_fileno(stdout)), "w");
  if (!protocol || bridge_dup2(bridge_fileno(stderr), bridge_fileno(stdout)) < 0) return 2;
  Bridge bridge;
  std::string line;
  while (std::getline(std::cin, line)) {
    try {
      const auto request = json::JsonParser(line).parseComplete();
      const std::string op = request.get("op").asString("step");
      if (op == "init") bridge.initialize(request);
      else if (op == "step") bridge.step(request);
      else if (op == "observe") {
        if (!bridge.initialized) throw std::runtime_error("Send init first");
      } else throw std::runtime_error("Unknown op");
      const auto response = bridge.observation();
      std::fprintf(protocol, "%s\n", response.c_str());
      std::fflush(protocol);
    } catch (const std::exception& error) {
      const auto response = "{\"ok\":false,\"error\":" + quote(error.what()) + "}";
      std::fprintf(protocol, "%s\n", response.c_str());
      std::fflush(protocol);
      return 1;
    }
  }
  return 0;
}
