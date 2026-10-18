#pragma once

/**
 * \file wingcfg.h
 * \brief Header-only loader for the tailsitter wing JSON config
 *        (the as_built.json produced by sim/as_built.py).
 *
 * The JSON mirrors the TailsitterInit packet fields plus the shared
 * motor/prop/battery StateInit fields.
 */

#include <string>
#include <filesystem>
#include <stdexcept>
#include <fmt/format.h>

#include "network/packets.h"
#include "json.h"

inline void loadWingConfigInto(const std::string& path, StateInit& s) {
  auto j = json::parseFile(path);

  s.vehicleType = VehicleTailsitter;

  auto& ts = s.tailsitter;
  ts.mass             = (float)j.get("mass").asNumber(0.08);
  ts.cgX              = (float)j.get("cgX").asNumber(0.0);

  if (j.has("inertia")) {
    auto I = j.get("inertia");
    ts.inertia[0] = (float)I.get("xx").asNumber(0.0);
    ts.inertia[1] = (float)I.get("yy").asNumber(0.0);
    ts.inertia[2] = (float)I.get("zz").asNumber(0.0);
    ts.inertia[3] = (float)I.get("xy").asNumber(0.0);
    ts.inertia[4] = (float)I.get("xz").asNumber(0.0);
    ts.inertia[5] = (float)I.get("yz").asNumber(0.0);
  }

  ts.span             = (float)j.get("span").asNumber(0.33);
  ts.rootChord        = (float)j.get("rootChord").asNumber(0.15);
  ts.tipChord         = (float)j.get("tipChord").asNumber(0.08);
  ts.rootHalfWidth    = (float)j.get("rootHalfWidth").asNumber(0.005);
  ts.leSweepDeg       = (float)j.get("leSweepDeg").asNumber(0.0);
  ts.thicknessRatioRoot = (float)j.get("thicknessRatioRoot").asNumber(0.09);
  ts.thicknessRatioTip  = (float)j.get("thicknessRatioTip").asNumber(0.09);

  if (j.has("camber")) {
    auto c = j.get("camber");
    ts.camberA1Root = (float)c.get("a1Root").asNumber(0.0);
    ts.camberA2Root = (float)c.get("a2Root").asNumber(0.0);
    ts.camberA1Tip  = (float)c.get("a1Tip").asNumber(ts.camberA1Root);
    ts.camberA2Tip  = (float)c.get("a2Tip").asNumber(ts.camberA2Root);
  }

  ts.xHinge           = (float)j.get("xHinge").asNumber(0.78);
  ts.elevonInboardY   = (float)j.get("elevonInboardY").asNumber(0.0);
  ts.elevonOutboardY  = (float)j.get("elevonOutboardY").asNumber(0.15);
  ts.elevonMaxDeg     = (float)j.get("elevonMaxDeg").asNumber(30.0);

  if (j.has("hinge")) {
    auto h = j.get("hinge");
    ts.hingePointL = h.get("pointL").asVec3();
    ts.hingeAxisL  = h.get("axisL").asVec3();
    ts.hingePointR = h.get("pointR").asVec3();
    ts.hingeAxisR  = h.get("axisR").asVec3();
  }

  ts.motorY           = (float)j.get("motorY").asNumber(0.036);
  ts.motorXAft        = (float)j.get("motorXAft").asNumber(-0.04);
  ts.motorZ           = (float)j.get("motorZ").asNumber(0.0);
  if (j.has("motorSpinDir")) {
    auto d = j.get("motorSpinDir");
    // prop spin direction about +x_a, per side
    ts.motorSpinDir[0] = (int32_t)d.get("right").asNumber(1.0);
    ts.motorSpinDir[1] = (int32_t)d.get("left").asNumber(-1.0);
  }
  ts.thrustScale      = (float)j.get("thrustScale").asNumber(1.0);

  if (j.has("aero")) {
    auto a = j.get("aero");
    ts.clMaxBase      = (float)a.get("clMaxBase").asNumber(ts.clMaxBase);
    ts.clMaxCamberGain= (float)a.get("clMaxCamberGain").asNumber(ts.clMaxCamberGain);
    ts.clMaxTcGain    = (float)a.get("clMaxTcGain").asNumber(ts.clMaxTcGain);
    ts.clMaxTcRef     = (float)a.get("clMaxTcRef").asNumber(ts.clMaxTcRef);
    ts.clMaxReGain    = (float)a.get("clMaxReGain").asNumber(ts.clMaxReGain);
    ts.clMaxReRef     = (float)a.get("clMaxReRef").asNumber(ts.clMaxReRef);
    ts.clMaxCeiling   = (float)a.get("clMaxCeiling").asNumber(ts.clMaxCeiling);
    ts.flapEffectiveness = (float)a.get("flapEffectiveness").asNumber(ts.flapEffectiveness);
    ts.flapStallDeg   = (float)a.get("flapStallDeg").asNumber(ts.flapStallDeg);
    ts.flapSaturationDeg = (float)a.get("flapSaturationDeg").asNumber(ts.flapSaturationDeg);
    ts.slipstreamDevelopment = (float)a.get("slipstreamDevelopment").asNumber(ts.slipstreamDevelopment);
    ts.slipstreamWidthFactor = (float)a.get("slipstreamWidthFactor").asNumber(ts.slipstreamWidthFactor);
    ts.cd0            = (float)a.get("cd0").asNumber(ts.cd0);
    ts.cdInduced      = (float)a.get("cdInduced").asNumber(ts.cdInduced);
    ts.cdmaxViterna   = (float)a.get("cdmaxViterna").asNumber(ts.cdmaxViterna);
    ts.numStrips      = a.get("numStrips").asInt(ts.numStrips);
  }
  ts.propDiameter     = (float)j.get("propDiameter").asNumber(ts.propDiameter);

  if (j.has("servo")) {
    auto sv = j.get("servo");
    ts.servoIndex[0]  = sv.get("indexL").asInt(ts.servoIndex[0]);
    ts.servoIndex[1]  = sv.get("indexR").asInt(ts.servoIndex[1]);
    ts.servoCenterUs  = (float)sv.get("centerUs").asNumber(ts.servoCenterUs);
    ts.servoDegPerUs  = (float)sv.get("degPerUs").asNumber(ts.servoDegPerUs);
    ts.servoSign[0]   = (float)sv.get("signL").asNumber(ts.servoSign[0]);
    ts.servoSign[1]   = (float)sv.get("signR").asNumber(ts.servoSign[1]);
    ts.servoTauSec    = (float)sv.get("tauSec").asNumber(ts.servoTauSec);
    ts.servoRateDps   = (float)sv.get("rateDps").asNumber(ts.servoRateDps);
    ts.servoMinUs     = (float)sv.get("minUs").asNumber(ts.servoMinUs);
    ts.servoMaxUs     = (float)sv.get("maxUs").asNumber(ts.servoMaxUs);
  }

  if (j.has("motorIndex")) {
    auto mi = j.get("motorIndex");
    ts.motorIndex[0] = mi.get("right").asInt(ts.motorIndex[0]);
    ts.motorIndex[1] = mi.get("left").asInt(ts.motorIndex[1]);
  }

  ts.fcYawDeg         = (float)j.get("fcYawDeg").asNumber(ts.fcYawDeg);

  // ---- shared propulsion / battery fields --------------------------------
  if (j.has("motor")) {
    auto m = j.get("motor");
    const float kv = (float)m.get("kv").asNumber(21000.0);
    s.motorKV[0] = kv;  s.motorKV[1] = kv;
    s.motorKV[2] = kv;  s.motorKV[3] = kv;
    const float r = (float)m.get("r").asNumber(0.07);
    s.motorR[0] = r; s.motorR[1] = r; s.motorR[2] = r; s.motorR[3] = r;
    const float i0 = (float)m.get("i0").asNumber(0.8);
    s.motorI0[0] = i0; s.motorI0[1] = i0; s.motorI0[2] = i0; s.motorI0[3] = i0;
    s.motorRth  = (float)m.get("rth").asNumber(35.0);
    s.motorCth  = (float)m.get("cth").asNumber(2.5);
    s.motorMaxT = (float)m.get("maxT").asNumber(180.0);
  }

  if (j.has("propeller")) {
    auto p = j.get("propeller");
    s.propBladeCount   = (uint8_t)p.get("bladeCount").asInt(3);
    s.propMaxRpm       = (float)p.get("maxRpm").asNumber(30000.0);
    s.propAFactor      = (float)p.get("aFactor").asNumber(5.3e-10);
    s.propTorqueFactor = (float)p.get("torqueFactor").asNumber(6.6e-3);
    s.propInertia      = (float)p.get("inertia").asNumber(1.15e-7);
    s.propThrustFactor = p.get("thrustFactor").asVec3();
  }

  if (j.has("battery")) {
    auto b = j.get("battery");
    s.maxVoltageSag        = (float)b.get("maxVoltageSag").asNumber(0.3);
    s.quadBatCellCount     = (uint8_t)b.get("cellCount").asInt(1);
    s.quadBatCapacityCharged = (float)b.get("capacityCharged").asNumber(480.0);
    s.quadBatCapacity        = (float)b.get("capacity").asNumber(480.0);
  }

  s.ambientTemp = (float)j.get("ambientTemp").asNumber(25.0);

  // eeprom name
  std::string eeprom = j.get("eepromName").asString();
  if (!eeprom.empty()) {
    std::fill(s.eepromName, s.eepromName + 512, 0);
    memcpy(s.eepromName, eeprom.c_str(),
           std::min(eeprom.size(), (size_t)511));
    s.eepromName[511] = '\0';
  }
}
