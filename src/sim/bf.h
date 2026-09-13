/**
 * Abstraction of betaflight functions.
 */

#include <cstdint>
#include "state.h"

#ifndef BF_H
#define BF_H

#define E_DEBUG_SIM 90

namespace SimITL{
  namespace BF{
    extern "C" {
      // betaflight's init
      extern void init(void);
      extern void scheduler(void);
      extern bool getBeeper(void);
    }

    /**
     * \brief Resets rc channels to default values.
     * Prevents initial random channel states.
     */
    void resetRcData();

    /**
     * \brief Resets rc channels to default values.
     * Prevents initial random channel states.
     */
    void setRcData(const float (&data)[8]);

    /**
     * \brief Sets an eeprom name, where the virtual fc writes to and reads from.
     * (Path is set by the execution dir)
     * \param[in] filename The name of the eeprom file, the fc writes to and reads from.
     */
    void setEepromFileName(const char* filename = "eeprom.bin");

    /**
     * \brief Updates the virtual fc's data with the provided state,
     * performs a betaflight schedule or sleeps and updates the state.
     * \param[in] delta Time delta in micro seconds.
     * \param[in] simState The state that is writen to the fc.
     * \return True if scheduler was executed, false if slept.
     */
    bool update(uint64_t dt, SimState& simState);

    /**
     * \brief BF debug call. Writes to blackbox.
     * \param[in] mode  The mode.
     * \param[in] index The index.
     * \param[in] value The value.
     */
    void setDebugValue(uint8_t mode, uint8_t index, int16_t value);

    struct ControllerTelemetry {
      std::array<float, 3> gyroDegS{};
      std::array<float, 3> setpointDegS{};
      std::array<float, 4> motorPwm{};
      // mechanical motor RPM as decoded from the emulated bidirectional
      // DSHOT telemetry frames (getDshotRpm()), i.e. what betaflight sees
      std::array<float, 4> motorRpm{};
      // raw eRPM telemetry values as decoded by betaflight (getDshotErpm()),
      // the same units that show up in blackbox logs (eRPM / 100)
      std::array<float, 4> motorErpm{};
      std::array<float, 8> rcDataUs{};
      std::array<float, 4> rcCommand{};
      std::array<float, 3> rawSetpointDegS{};
      std::array<float, 3> attitudeDeg{};
      std::array<float, 3> pidSetpointDegS{};
      bool crashRecovery = false;
      bool launchControl = false;
      uint32_t flightModeFlags = 0;
      float maxVelocityR = 0;
      bool itermRotation = false;
      float acGain = 0;
      bool acroTrainer = false;
      bool pidEnabled = false;
      bool calibrated = false;
      bool armed = false;
      bool dshotTelemetryActive = false;
    };

    ControllerTelemetry getControllerTelemetry();
    void setTelemetryMotorPwm(const std::array<float, 4>& pwm);
    void configureBridgeArming();

    void updateSerial();
    void stopSerial();
  }
}

#endif // BF_H