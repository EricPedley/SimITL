#ifndef SIM_PACKETS_H
#define SIM_PACKETS_H

#include <cstdint>

enum CommandType : int32_t {
  None      = 0U,
  Stop      = 1U,
  Repair    = 2U,
  Reset     = 4U,
};

enum MotorStatus : int32_t {
  MotorNone = 0U,
  MotorBurnedOut = 1U,
  MotorDamaged = 2U,
};

enum VehicleType : int32_t {
  VehicleQuad = 0,
  VehicleTailsitter = 1,
};

struct Vec3F{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Vec4F{
  float x = 1.0f;
  float y = 0.0f;
  float z = 0.0f;
  float w = 1.0f;
};

struct GpsData{
  // 48.420876993590745, 10.711263681338096
  int32_t lat = 484208769;
  int32_t lon = 107112636;
  int32_t alt = 0;
};

/**
 * Parameters for the tailsitter flying-wing physics model.
 *
 * Geometry uses the airfoil_model.py convention: chordwise positions are
 * metres aft of the root leading edge (aft positive), spanwise positions are
 * metres from the centreline.  The flight dynamics internal body frame is the
 * aerospace frame (x forward/nose, y right/starboard, z down); the frame
 * handedness conversion to SimITL's world happens inside TailsitterPhysics.
 */
struct TailsitterInit {
  // mass and full inertia tensor about the CG, aerospace body frame
  // (x fwd, y right, z down). inertia = {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}
  float mass = 0.0f;
  float inertia[6] = {};
  // CG location, m aft of the root leading edge, on the centreline
  float cgX = 0.0f;

  // wing planform (see airfoil_model.py)
  float span = 0.0f;
  float rootChord = 0.0f;
  float tipChord = 0.0f;
  float rootHalfWidth = 0.0f;    // semi-width of the constant-chord centre strip
  float leSweepDeg = 0.0f;       // leading edge sweep of the tapered panel
  float thicknessRatioRoot = 0.0f;
  float thicknessRatioTip = 0.0f;
  // Birnbaum-Glauert camber coefficients, root and tip
  float camberA1Root = 0.0f;
  float camberA2Root = 0.0f;
  float camberA1Tip = 0.0f;
  float camberA2Tip = 0.0f;

  // elevons (per side, symmetric)
  float xHinge = 0.0f;           // hinge station as fraction of local chord
  float elevonInboardY = 0.0f;   // m from centreline
  float elevonOutboardY = 0.0f;  // m from centreline
  float elevonMaxDeg = 0.0f;     // geometric travel limit (linkage delta_max)
  // hinge axes/points in aerospace body frame, for the renderer
  // (physics only needs the scalar deflection per side)
  Vec3F hingePointL {};
  Vec3F hingeAxisL {};
  Vec3F hingePointR {};
  Vec3F hingeAxisR {};

  // propulsion: two tractor props ahead of the leading edge
  float motorY = 0.0f;           // spanwise station, m
  float motorXAft = 0.0f;        // prop disk chordwise station (aft of root LE,
                                 // negative = ahead of the wing)
  float motorZ = 0.0f;           // vertical offset of prop axis vs chord plane
  int32_t motorSpinDir[2] = {1, -1}; // spin direction about +x_a, {right, left}
  float thrustScale = 1.0f;      // multiplies prop thrust (unmeasured static
                                 // thrust correction, see FLIGHT_SIM_PLAN)

  // section / aero constants (airfoil_model.py values)
  float clMaxBase = 0.80f;
  float clMaxCamberGain = 8.0f;
  float clMaxTcGain = 1.5f;
  float clMaxTcRef = 0.085f;
  float clMaxReGain = 0.12f;
  float clMaxReRef = 40000.0f;
  float clMaxCeiling = 1.35f;
  float flapEffectiveness = 0.65f;
  float flapStallDeg = 15.0f;
  float flapSaturationDeg = 30.0f;
  float slipstreamDevelopment = 0.5f;
  float slipstreamWidthFactor = 1.0f;
  float propDiameter = 0.051f;
  float cd0 = 0.02f;             // section profile drag in the attached range
  float cdInduced = 0.0f;        // extra Cl^2 term; 0 -> 1/(pi*AR*e), e=0.85
  float cdmaxViterna = 0.0f;     // flat-plate drag ceiling; 0 -> 1.11+0.018*AR
  int32_t numStrips = 33;        // spanwise strips, odd recommended

  // servo model: pwm microseconds -> elevon deflection
  // deflection_deg = (pwm - centerUs) * servoSign * degPerUs, lagged and
  // slew-rate limited. servoIndex = BF smix target index per side.
  int32_t servoIndex[2] = {2, 3};   // {left, right} BF servo table indices
  float servoCenterUs = 1500.0f;
  float servoDegPerUs = 0.1f;       // deg of elevon travel per microsecond
  float servoSign[2] = {1.0f, 1.0f};// elevon deflection sign vs pwm, {left,right}
  float servoTauSec = 0.02f;        // first-order lag time constant
  float servoRateDps = 600.0f;      // slew rate limit, deg/s
  float servoMinUs = 500.0f;        // output clamps
  float servoMaxUs = 2500.0f;

  // flight controller board alignment (degrees about aircraft z axis, matches
  // align_board_yaw in the Betaflight config)
  float fcYawDeg = 180.0f;
};

// initial quad settings.
struct StateInit{
  int32_t vehicleType = 0;  // VehicleType

  // tailsitter / flying-wing parameters, used when
  // vehicleType == VehicleTailsitter
  TailsitterInit tailsitter {};
  float motorKV[4]    = {}; // KV
  float motorR[4]     = {}; // resistance
  float motorI0[4]    = {}; // idle current
  float motorRth   = 0.0f; // thermal resistance (deg C per Watt)
  float motorCth   = 0.0f; // thermal heat capacity (joules per deg C)
  float motorMaxT  = 0.0f; // max temp in C when motor windings short out

  uint8_t propBladeCount = 0U;
  float propMaxRpm = 0.0f;
  float propAFactor = 0.0f;
  float propTorqueFactor = 0.0f;
  float propInertia = 0.0f;
  Vec3F propThrustFactor {};
  float propHarmonic1Amp = 0.0f;
  float propHarmonic2Amp = 0.0f;

  Vec3F frameDragArea {};
  float frameDragConstant = 0.0f;

  float quadMass = 0.0f;
  Vec3F quadInvInertia {};
  float maxVoltageSag = 0.0f;
  uint8_t quadBatCellCount = 1;
  //charged up capacity, can be lower or higher than quadBatCapacity
  float quadBatCapacityCharged  = 0.0f;
  //battery capacacity rating
  float quadBatCapacity = 0.0f;
  //motor position in meters
  Vec3F quadMotorPos[4] {};

  // motor direction 1(CW) or -1(CCW) 
  float quadMotorDir[4] = {}; 

  // prop wash starts at this speed in m/s
  float minPropWashSpeed = 1.0f; 
  // prop wash reacheas peak at this speed in m/s
  float maxPropWashSpeed = 18.0f;
  // max prop wash angle of attack (0.0 = 180 degrees in reverse trust dir)
  // > 0.0 means propwash in trust dir is concentrated more towards reverse trust dir ( less than 180 deg)
  // < 0.0 prop was angle of attack is greater than 180 degrees so even angles in thrust dir have an effect
  float propWashAngleOfAttack = 0.5f;
  // multiplier for the final prop wash effect
  float propWashFactor = 1.0f;

  float ambientTemp = 0.0f;

  GpsData gps {};

  //eeprom file name
  uint8_t eepromName[512] = {};
};

//runtime quad parameters - simulation input
struct StateInput{
  float delta = 0.0f;

  // Normalised RC channel values in the range -1.0 to 1.0.
  float rcData[8] {};

  // Open-loop motor PWM override [-1.0, 1.0].
  // When openLoop is non-zero, these values are fed directly to the physics
  // engine, bypassing the Betaflight PID controller.
  float motorPwm[4] = {-1.0f, -1.0f, -1.0f, -1.0f};

  Vec3F position {};
  Vec3F rotation[3] {};

  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  // length is amplitude per axis
  Vec3F motorImbalance[4] {};

  float gyroBaseNoiseAmp = 0.0f;
  float gyrobaseNoiseFreq = 0.0f;

  float frameHarmonic1Amp = 0.0f;
  float frameHarmonic1Freq = 0.0f;

  float frameHarmonic2Amp = 0.0f;
  float frameHarmonic2Freq = 0.0f;

  float propDamage[4] {};
  float groundEffect[4] {};

  // charged battery voltage
  float vbat = 0.0f;

  // 1 true 0 false
  uint8_t contact = 0;

  // 1: bypass Betaflight PID and use motorPwm[] directly
  uint8_t openLoop = 0;
};

//simulation output for game
struct StateOutput{
  Vec4F orientation{};
  Vec3F angularVelocity {};
  Vec3F linearVelocity {};

  // motor output signal -1.0 to 1.0
  float motorOutput[4] {};

  // rpm
  float motorRpm[4] {};
  // motor torque
  float motorTorque[4] {};
  // propeller Torque
  float propellerTorque[4] {};
  // net torque
  float netTorque[4] {};
  // thrust in Nm
  float thrust[4] {};
  // amp draw in A
  float motorCurrent[4] {};
  // temperature in deg C
  float motorT[4] {};
  // status
  int32_t motorStatus[4] {};

  // battery voltage in V
  float batteryVoltage;

  // servo outputs in microseconds (BF servo table indices 0..7)
  float servoOutput[8] {};

  // actual elevon deflection in degrees after servo dynamics, {left, right}
  float elevonDeg[2] {};

  // betaflight armed state and disable-flag bitmask (armingDisableFlags_e)
  uint8_t armed = 0U;
  int32_t armingDisableFlags = 0;

  // gyro as seen by the flight controller, deg/s, in FC axes
  // (after board alignment) - diagnostic for axis/sign checks
  float fcGyro[3] {};

  // pidData[FD_*].Sum - roll, pitch, yaw controller outputs
  float fcPidSum[3] {};

  // BF rcCommand roll/pitch/throttle/yaw in native units (deg/s for rates)
  float fcRcCommand[4] {};

  // BF estimated attitude roll/pitch/yaw in degrees (attitude.values/10)
  float fcAttitude[3] {};

  // BF rcData channels 0..7 in us after channel mapping
  float fcRcData[8] {};

  // beeper on (1) / off (0)
  uint8_t beep = 0U;

  uint8_t osd[16*30] {};
};

#endif