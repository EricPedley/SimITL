#include "tailsitter.h"
#include <fmt/format.h>
#include <cassert>

namespace SimITL{
  #ifndef M_PI
  #define M_PI 3.14159265358979
  #endif

  const auto TS_RHO = 1.225f;   // kg/m^3
  const auto TS_MU  = 1.81e-5f; // Pa.s
  const auto TS_G   = 9.81f;    // m/s^2

  // sim body (x=port, y=dorsal, z=nose) <-> aerospace (x fwd, y right, z down):
  //   x_a = z_s,  y_a = -x_s,  z_a = -y_s
  static const mat3 SIM_TO_AERO = {
    vec3{ 0.0f, 0.0f, 1.0f},
    vec3{-1.0f, 0.0f, 0.0f},
    vec3{ 0.0f,-1.0f, 0.0f},
  };
  // aero -> sim is the transpose

  static void matInv(const float m[9], float inv[9]){
    const float det =
        m[0]*(m[4]*m[8] - m[5]*m[7])
      - m[1]*(m[3]*m[8] - m[5]*m[6])
      + m[2]*(m[3]*m[7] - m[4]*m[6]);
    const float id = 1.0f / det;
    inv[0] =  (m[4]*m[8] - m[5]*m[7]) * id;
    inv[1] = -(m[1]*m[8] - m[2]*m[7]) * id;
    inv[2] =  (m[1]*m[5] - m[2]*m[4]) * id;
    inv[3] = -(m[3]*m[8] - m[5]*m[6]) * id;
    inv[4] =  (m[0]*m[8] - m[2]*m[6]) * id;
    inv[5] = -(m[0]*m[5] - m[2]*m[3]) * id;
    inv[6] =  (m[3]*m[7] - m[4]*m[6]) * id;
    inv[7] = -(m[0]*m[7] - m[1]*m[6]) * id;
    inv[8] =  (m[0]*m[4] - m[1]*m[3]) * id;
  }

  static vec3 matVec(const float m[9], const vec3& v){
    return {m[0]*v[0] + m[1]*v[1] + m[2]*v[2],
            m[3]*v[0] + m[4]*v[1] + m[5]*v[2],
            m[6]*v[0] + m[7]*v[1] + m[8]*v[2]};
  }

  bool TailsitterPhysics::checkSimState(){
    return mSimState != nullptr;
  }

  void TailsitterPhysics::setSimState(SimState* state){
    mSimState = state;
  }

  void TailsitterPhysics::initState(const StateInit& state){
    memcpy((void*)&(mSimState->stateInit), (const void*)(&state), sizeof(StateInit));

    mSimState->batteryState.batVoltage = 0.1f;
    mSimState->batteryState.batVoltageSag = 0.1f;
    mSimState->batteryState.batCapacity = mSimState->stateInit.quadBatCapacityCharged;

    const auto& ts = mSimState->stateInit.tailsitter;

    // inertia tensor (row-major): {Ixx,Iyy,Izz,Ixy,Ixz,Iyz}
    mInertia[0] = ts.inertia[0]; mInertia[1] = -ts.inertia[3]; mInertia[2] = -ts.inertia[4];
    mInertia[3] = -ts.inertia[3]; mInertia[4] = ts.inertia[1]; mInertia[5] = -ts.inertia[5];
    mInertia[6] = -ts.inertia[4]; mInertia[7] = -ts.inertia[5]; mInertia[8] = ts.inertia[2];
    matInv(mInertia, mInvInertia);

    // wing area / aspect ratio, same construction as airfoil_model.planform
    const float semi = 0.5f * ts.span;
    const float f0 = clamp(ts.rootHalfWidth / semi, 0.0f, 1.0f);
    const float spanRoot = f0 * semi;
    const float spanTaper = semi - spanRoot;
    const float areaRoot = ts.rootChord * spanRoot;
    const float areaTaper = 0.5f * (ts.rootChord + ts.tipChord) * spanTaper;
    mWingArea = 2.0f * (areaRoot + areaTaper);
    mAspectRatio = (mWingArea > 1e-9f) ? ts.span * ts.span / mWingArea : 4.0f;
    mCdmax = (ts.cdmaxViterna > 0.0f) ? ts.cdmaxViterna
                                    : std::min(1.11f + 0.018f * mAspectRatio, 2.0f);

    mServoDeg[0] = 0.0f;
    mServoDeg[1] = 0.0f;

    buildStrips();
  }

  void TailsitterPhysics::buildStrips(){
    const auto& ts = mSimState->stateInit.tailsitter;
    mStrips.clear();

    const int n = std::max(3, ts.numStrips | 1); // odd: a strip on centreline
    const float semi = 0.5f * ts.span;
    const float f0 = clamp(ts.rootHalfWidth / semi, 0.0f, 1.0f);
    const float sweep = ts.leSweepDeg * float(M_PI) / 180.0f;
    const float dy = ts.span / n;

    for (int i = 0; i < n; i++){
      Strip s{};
      s.y = -semi + (i + 0.5f) * dy;
      s.dy = dy;
      const float f = fabsf(s.y) / semi;
      // taper fraction: 0 across the centre strip, linear to the tip
      const float t = clamp((f - f0) / std::max(1.0f - f0, 1e-9f), 0.0f, 1.0f);
      s.chord = ts.rootChord + (ts.tipChord - ts.rootChord) * t;
      s.tc = ts.thicknessRatioRoot + (ts.thicknessRatioTip - ts.thicknessRatioRoot) * t;
      s.leX = tanf(sweep) * std::max(fabsf(s.y) - ts.rootHalfWidth, 0.0f);
      s.a1 = ts.camberA1Root + (ts.camberA1Tip - ts.camberA1Root) * t;
      s.a2 = ts.camberA2Root + (ts.camberA2Tip - ts.camberA2Root) * t;
      const float ay = fabsf(s.y);
      s.flapSide = (ay >= ts.elevonInboardY && ay <= ts.elevonOutboardY)
                 ? (s.y < 0.0f ? 0 : 1) : -1;
      mStrips.push_back(s);
    }
    buildDownwash();
  }

  // z-velocity at (px, py) from a unit vortex segment a->b in the z = 0
  // plane (Biot-Savart).
  static float segmentW(float px, float py, float ax, float ay, float bx, float by){
    const float r0x = bx - ax, r0y = by - ay;
    const float r1x = px - ax, r1y = py - ay;
    const float r2x = px - bx, r2y = py - by;
    const float cr = r1x * r2y - r1y * r2x;
    const float n1 = sqrtf(r1x * r1x + r1y * r1y), n2 = sqrtf(r2x * r2x + r2y * r2y);
    if (cr * cr < 1e-12f || n1 < 1e-9f || n2 < 1e-9f) return 0.0f;
    const float d = r0x * (r1x / n1 - r2x / n2) + r0y * (r1y / n1 - r2y / n2);
    return cr / (cr * cr) * d / (4.0f * float(M_PI));
  }

  // Discrete Prandtl lifting line: downwash (aero +z) on the lifting line at
  // strip i per unit circulation on strip j's horseshoe (bound along the
  // span, trailing legs aft).  Laid out on the *unswept* planform: the
  // sections already apply sweep theory, so this adds only the finite-span
  // downwash.  On the bound line only the trailing legs contribute; the 2D
  // section curve already carries the bound vortex's own effect.
  void TailsitterPhysics::buildDownwash(){
    const size_t n = mStrips.size();
    const float far = 1000.0f;
    mDownwash.assign(n * n, 0.0f);
    float trace = 0.0f;
    for (size_t i = 0; i < n; i++){
      const float py = mStrips[i].y;
      for (size_t j = 0; j < n; j++){
        const float ay = mStrips[j].y - 0.5f * mStrips[j].dy;
        const float by = mStrips[j].y + 0.5f * mStrips[j].dy;
        mDownwash[i * n + j] = segmentW(0.0f, py, -far, ay, 0.0f, ay)
                             + segmentW(0.0f, py, 0.0f, ay, 0.0f, by)
                             + segmentW(0.0f, py, 0.0f, by, -far, by);
      }
      trace += mDownwash[i * n + i];
    }
    // positive circulation (lift) must wash its own strip down
    if (trace < 0.0f) for (float& k : mDownwash) k = -k;
  }



  void TailsitterPhysics::updateCommands(CommandType commands){
    switch (commands){
    case CommandType::Repair: repair(); break;
    case CommandType::Reset:  reset();  break;
    default: break;
    }
  }

  void TailsitterPhysics::updateState(const StateInput& state){
    memcpy((void*)&(mSimState->stateInput), (const void*)(&state), sizeof(StateInput));
  }

  float TailsitterPhysics::motorPwm(int side) const {
    const int idx = clamp(mSimState->stateInit.tailsitter.motorIndex[side], 0, 3);
    return mSimState->motorsState[idx].pwm;
  }

  // Body-frame vector (sim axes: x=port, y=dorsal, z=nose) -> what the
  // physical sensor on the flight controller reads, expressed so that bf.cpp's
  // sensor remap (written for the quad's left-handed Unity frame) passes it
  // through unchanged.
  //
  // Betaflight's body frame is x forward, y left, z up (roll right, pitch
  // nose-down, yaw nose-left positive), i.e. (nose, port, dorsal).  The board
  // is mounted rotated fcYawDeg about z; Betaflight's align_board_yaw undoes
  // that, so the raw reading is the body vector rotated by -fcYawDeg.
  // bf.cpp then reads gyro as (-g[2], g[0], -g[1]) and acc as
  // (-a[2], a[0], a[1]), which is inverted here.
  static vec3 bodyToBoard(const vec3& sim, float fcYawDeg){
    const float c = cosf(fcYawDeg * float(M_PI) / 180.0f);
    const float s = sinf(fcYawDeg * float(M_PI) / 180.0f);
    const float x = sim[2], y = sim[0], z = sim[1];   // Betaflight body axes
    return vec3{c * x + s * y, -s * x + c * y, z};
  }

  void TailsitterPhysics::updateGyro(double dt){
    mat3 basis;
    copy(basis, mSimState->stateInput.rotation);
    const float yawDeg = mSimState->stateInit.tailsitter.fcYawDeg;

    // Attitude for Betaflight: body (x fwd, y left, z up) -> earth (z up).
    // Body axes are (nose, port, dorsal) = sim (z, x, y); take earth axes as
    // world (z, x, y).  Both are the same cyclic permutation of the sim
    // frames, so Betaflight's quaternion is the sim one with its vector part
    // permuted (x, y, z) -> (z, x, y).  bf.cpp hands Betaflight
    // (w, -r[2], -r[0], r[1]) (written for the quad's frame), so store the
    // pre-inverted form.  Board alignment does not apply: this is body
    // attitude, not a sensor reading.  Betaflight's simulator build also
    // negates two rMat entries for the quad's frame; the patched imu.c skips
    // that for fixed-wing mixers so this quaternion arrives as a proper
    // rotation (sim/test_control_signs.py checks the attitude it reports).
    const quat qs = mat3_to_quat(basis);            // (x, y, z, w), body->world
    mSimState->rotation = quat{-qs[0], qs[1], -qs[2], qs[3]};

    vec3 angularVelocity;
    copy(angularVelocity, mSimState->stateInput.angularVelocity);
    const vec3 gb = bodyToBoard(xform_inv(basis, angularVelocity), yawDeg);
    mSimState->gyro = vec3{gb[1], -gb[2], -gb[0]};

    // accelerometers read specific force, a - g: +1 g up when at rest
    const vec3 gravity{0.0f, -TS_G, 0.0f};
    const vec3 ab = bodyToBoard(
        xform_inv(basis, mSimState->acceleration - gravity), yawDeg);
    mSimState->acc = vec3{ab[1], ab[2], -ab[0]};
  }

  // --- propulsion ------------------------------------------------------------

  float TailsitterPhysics::inducedVelocity(float thrustN, float vInf,
                                           float diskArea) const {
    // T = 2 rho A (v_inf + w) w  ->  w = -v/2 + sqrt((v/2)^2 + T/(2 rho A))
    const float halfV = 0.5f * vInf;
    return -halfV + sqrtf(halfV * halfV +
                          std::max(thrustN, 0.0f) / (2.0f * TS_RHO * diskArea));
  }

  float TailsitterPhysics::clMaxFor(const Strip& s, float re) const {
    const auto& ts = mSimState->stateInit.tailsitter;
    // camber_max sampled like airfoil_model.py (41-point scan of camber_line)
    float camber = 0.0f;
    for (int k = 0; k < 41; k++){
      const float x = k / 40.0f;
      const float c = clamp(1.0f - 2.0f * x, -1.0f + 1e-9f, 1.0f - 1e-9f);
      const float th = acosf(c);
      const float ct = cosf(th);
      const float z = s.a1 * sinf(th) * sinf(th) / 4.0f
                    + s.a2 * (-ct * ct * ct / 3.0f + ct / 2.0f - 1.0f / 6.0f)
                    + s.a2 * x / 3.0f;
      camber = std::max(camber, fabsf(z));
    }
    float cl = ts.clMaxBase
             + ts.clMaxCamberGain * camber
             + ts.clMaxTcGain * (s.tc - ts.clMaxTcRef)
             + ts.clMaxReGain * logf(std::max(re, 1e3f) / ts.clMaxReRef);
    return clamp(cl, 0.2f, ts.clMaxCeiling);
  }

  void TailsitterPhysics::sectionClCdCm(const Strip& s, float alphaEffRad,
                                        float& cl, float& cd, float& cm) const {
    const auto& ts = mSimState->stateInit.tailsitter;
    // local Reynolds for the Cl_max correlation
    // (caller folds local q into forces; Re needs |v|, passed via alpha path)
    // alpha_s: effective incidence at which the attached slope reaches Cl_max
    const float clMax = s.clMax;
    const float slope = 2.0f * float(M_PI);
    const float aS = clMax / slope;
    const float ae = alphaEffRad;
    const float sgn = (ae >= 0.0f) ? 1.0f : -1.0f;
    const float aAbs = fabsf(ae);

    // no Cl^2/(pi A e) term: induced drag comes from the lifting line (each
    // strip's lift is tilted back by its induced angle)
    if (aAbs <= aS){
      cl = slope * ae;
      cd = ts.cd0;
    } else {
      // flat-plate post-stall: attached Cl_s continued as a scaled sin(2a),
      // Cd blended from its stall value to CDmax at 90 deg and back down.
      const float clS = clMax;                    // CL at stall (signed below)
      const float cdS = ts.cd0;
      // Viterna-Corrigan: flat-plate 1/2 Cdmax sin 2a plus a term that
      // matches cl_s at the stall angle and decays (cl used to be
      // cl_s sin 2a / sin 2a_s, peaking ~3x cl_s at 45 deg)
      const float a2 = (clS - mCdmax * sinf(aS) * cosf(aS)) * sinf(aS)
                     / (cosf(aS) * cosf(aS));
      cl = sgn * (0.5f * mCdmax * sinf(2.0f * aAbs)
                  + a2 * cosf(aAbs) * cosf(aAbs) / std::max(sinf(aAbs), 1e-6f));
      const float s2 = sinf(aAbs) * sinf(aAbs);
      const float s2s = sinf(aS) * sinf(aS);
      cd = cdS + (mCdmax - cdS) * (s2 - s2s) / std::max(1.0f - s2s, 1e-6f);
      cd = std::max(cd, 0.0f);
    }
    // thin-airfoil Cm about c/4, held constant through stall
    cm = 0.25f * float(M_PI) * (s.a2 - s.a1);
  }

  void TailsitterPhysics::computeForces(const vec3& velWorld,
                                        const vec3& omegaWorld,
                                        const mat3& basis,
                                        float elevonDegL, float elevonDegR,
                                        const float thrustN[2],
                                        vec3& outForceAero,
                                        vec3& outMomentAero) const {
    const auto& ts = mSimState->stateInit.tailsitter;

    // aero axes in world = sim basis * M^T
    const mat3 basisA = basis * transpose(SIM_TO_AERO);

    const vec3 vA = xform_inv(basisA, velWorld);   // aero body-frame velocity
    const vec3 wA = xform_inv(basisA, omegaWorld); // aero body-frame rates p,q,r

    const float diskArea = 0.25f * float(M_PI) * ts.propDiameter * ts.propDiameter;
    const float halfWash = 0.5f * ts.slipstreamWidthFactor * ts.propDiameter;
    const float elevonDefl[2] = {elevonDegL, elevonDegR};

    // --- pass 1: per-strip flow and section parameters ----------------------
    struct StripFlow {
      vec3 cHat, sHat;
      float vs, v2, vn, alpha, shift, flapM, clMax;
      bool reversed;
    };
    const size_t n = mStrips.size();
    std::vector<StripFlow> st(n);
    for (size_t k = 0; k < n; k++){
      const Strip& s = mStrips[k];
      StripFlow& q = st[k];
      // strip quarter-chord point relative to CG, aerospace frame
      const vec3 r{ts.cgX - (s.leX + 0.25f * s.chord), s.y, 0.0f};
      vec3 v = vA + cross(wA, r);

      // slipstream: strips inside the wash of either prop get the induced
      // axial increment on their forward component
      const float ay = fabsf(s.y);
      if (fabsf(ay - ts.motorY) <= halfWash){
        const int mi = (s.y > 0.0f) ? 0 : 1; // {right, left}
        const float w = inducedVelocity(thrustN[mi], v[0], diskArea);
        v[0] += ts.slipstreamDevelopment * 2.0f * w;
      }

      // Simple sweep theory: the section sees the flow perpendicular to the
      // strip's swept quarter-chord line (c-hat, z); flow along that line
      // (s-hat) only adds skin friction.  The sweep is mirrored left/right,
      // so sideslip loads the two halves differently -- the swept wing's
      // roll-from-sideslip and weak directional stiffness.  The centre
      // section (|y| < rootHalfWidth) is unswept.
      const float lam = (ay > ts.rootHalfWidth) ? ts.sweepC4Deg * float(M_PI) / 180.0f : 0.0f;
      const float side = (s.y > 0.0f) ? 1.0f : -1.0f;
      q.cHat = vec3{cosf(lam), side * sinf(lam), 0.0f};   // chordwise, forward
      q.sHat = vec3{-sinf(lam), side * cosf(lam), 0.0f};  // along the c/4 line, outboard
      const float un = dot(v, q.cHat);
      const float wn = v[2];
      q.vs = dot(v, q.sHat);
      q.v2 = un * un + wn * wn;
      q.vn = sqrtf(q.v2);

      // Flow from the trailing edge (un < 0): mirror the section.  The
      // force then acts at 3/4 chord (the quarter chord of the reversed
      // section), camber is ignored (flat plate), and the elevon is a
      // leading-edge flap: thin-airfoil dCl/d(delta) = -2 (th - sin th) for
      // a flap of (1 - xHinge) chord at the front -- weak and of opposite
      // sign to the trailing-edge flap.
      q.reversed = un < 0.0f;
      q.alpha = q.reversed ? atan2f(wn, -un) : atan2f(wn, un);

      float deltaRad = 0.0f;
      float flapLiftSlope = 0.0f;
      float flapMomentSlope = 0.0f;
      if (s.flapSide >= 0){
        const float deg = elevonDefl[s.flapSide];
        const float excess = std::max(fabsf(deg) - ts.flapStallDeg, 0.0f);
        const float eff = ts.flapEffectiveness
                        * expf(-excess / (ts.flapSaturationDeg - ts.flapStallDeg));
        deltaRad = deg * float(M_PI) / 180.0f;
        if (!q.reversed){
          const float th = acosf(clamp(1.0f - 2.0f * ts.xHinge,
                                       -1.0f + 1e-9f, 1.0f - 1e-9f));
          flapLiftSlope = 2.0f * (float(M_PI) - th + sinf(th)) * eff;
          flapMomentSlope = 0.25f * (sinf(2.0f * th) - 2.0f * sinf(th)) * eff;
        } else {
          const float th = acosf(clamp(1.0f - 2.0f * (1.0f - ts.xHinge),
                                       -1.0f + 1e-9f, 1.0f - 1e-9f));
          flapLiftSlope = -2.0f * (th - sinf(th)) * eff;
        }
      }
      // The section flow is normal to the c/4 line (dynamic pressure x
      // cos^2 of the c/4 sweep), but the elevon rotates about its own hinge,
      // whose effectiveness goes as cos^2 of the hinge sweep: rescale.
      if (s.flapSide >= 0 && lam > 0.0f){
        const float ch = cosf(ts.hingeSweepDeg * float(M_PI) / 180.0f) / cosf(lam);
        flapLiftSlope *= ch * ch;
        flapMomentSlope *= ch * ch;
      }
      const float alphaL0 = q.reversed ? 0.0f : -0.5f * s.a1;
      // elevon as an equivalent incidence shift, so stall carries over
      q.shift = flapLiftSlope * deltaRad / (2.0f * float(M_PI)) - alphaL0;
      q.flapM = flapMomentSlope * deltaRad;
      const float re = TS_RHO * q.vn * s.chord / TS_MU;
      q.clMax = clMaxFor(s, re);
    }

    // --- pass 2: lifting line --------------------------------------------------
    // One linear solve for the circulation: gamma_i = 1/2 c_i 2pi (vfl_i
    // alpha_c,i - (K gamma)_i), with each strip's angle clamped to its stall
    // angle.  Exact lifting line in attached flow; in stall the downwash is
    // that of stall-level lift (a nonlinear post-stall lifting line has
    // multiple solutions and Newton wanders between them).  Reversed-flow
    // strips carry no circulation.  vfl = max(Vn, 0.5).
    std::vector<float> gamma(n, 0.0f), aInd(n, 0.0f), vfl(n), A(n * n, 0.0f);
    for (size_t i = 0; i < n; i++){
      vfl[i] = std::max(st[i].vn, 0.5f);
      A[i * n + i] = 1.0f;
      if (st[i].reversed) continue;
      const float aS = st[i].clMax / (2.0f * float(M_PI));
      const float aC = clamp(st[i].alpha + st[i].shift, -aS, aS);
      const float g = 0.5f * mStrips[i].chord * 2.0f * float(M_PI);
      for (size_t j = 0; j < n; j++) A[i * n + j] += g * mDownwash[i * n + j];
      gamma[i] = g * vfl[i] * aC;                      // right-hand side
    }
    // Gaussian elimination with partial pivoting: A gamma = rhs, in place
    for (size_t c = 0; c < n; c++){
      size_t piv = c;
      for (size_t r2 = c + 1; r2 < n; r2++)
        if (fabsf(A[r2 * n + c]) > fabsf(A[piv * n + c])) piv = r2;
      if (piv != c){
        for (size_t k2 = 0; k2 < n; k2++) std::swap(A[c * n + k2], A[piv * n + k2]);
        std::swap(gamma[c], gamma[piv]);
      }
      const float d = A[c * n + c];
      for (size_t r2 = c + 1; r2 < n; r2++){
        const float f = A[r2 * n + c] / d;
        if (f == 0.0f) continue;
        for (size_t k2 = c; k2 < n; k2++) A[r2 * n + k2] -= f * A[c * n + k2];
        gamma[r2] -= f * gamma[c];
      }
    }
    for (size_t c = n; c-- > 0;){
      float acc = gamma[c];
      for (size_t k2 = c + 1; k2 < n; k2++) acc -= A[c * n + k2] * gamma[k2];
      gamma[c] = acc / A[c * n + c];
    }
    for (size_t i = 0; i < n; i++){
      if (st[i].reversed) continue;
      float w = 0.0f;
      for (size_t j = 0; j < n; j++) w += mDownwash[i * n + j] * gamma[j];
      aInd[i] = w / vfl[i];
    }

    // --- pass 3: forces, lift perpendicular to the downwashed inflow --------
    vec3 F{0.0f, 0.0f, 0.0f};
    vec3 M{0.0f, 0.0f, 0.0f};
    const vec3 zHat{0.0f, 0.0f, 1.0f};
    for (size_t k = 0; k < n; k++){
      const Strip& s = mStrips[k];
      const StripFlow& q = st[k];
      const float aFlow = q.alpha - aInd[k];
      Strip sc = s;
      sc.clMax = q.clMax;
      float cl, cd, cm;
      sectionClCdCm(sc, aFlow + q.shift, cl, cd, cm);
      cm = q.reversed ? 0.0f : cm + q.flapM;
      const float qd = 0.5f * TS_RHO * q.v2;
      const float dA = s.chord * s.dy;
      // lift perpendicular to the relative wind (-cos a, -sin a in the
      // section plane): (+sin a, -cos a), mirrored for reversed flow; drag
      // along the wind.  (The model used to have -sin a in the lift term,
      // tilting lift back by 2 alpha.)
      const float sa = sinf(aFlow), ca = cosf(aFlow);
      const float xs = q.reversed ? -1.0f : 1.0f;
      const vec3 fDir = q.cHat * (sa * xs) + zHat * (-ca);
      const vec3 dDir = q.cHat * (-ca * xs) + zHat * (-sa);
      // spanwise skin friction: half the profile drag coefficient
      const vec3 fSpan = q.sHat * (-0.5f * TS_RHO * fabsf(q.vs) * q.vs * 0.5f * ts.cd0 * dA);
      const vec3 fStrip = fDir * (qd * dA * cl) + dDir * (qd * dA * cd) + fSpan;
      const float xa = q.reversed ? 0.75f : 0.25f;
      const vec3 rf{ts.cgX - (s.leX + xa * s.chord), s.y, 0.0f};
      F = F + fStrip;
      M = M + cross(rf, fStrip);
      M[1] += qd * s.chord * s.chord * s.dy * cm; // nose-up positive, aero y
    }

    outForceAero = F;
    outMomentAero = M;
  }

  // --- servo dynamics ----------------------------------------------------------

  void TailsitterPhysics::updateServos(double dt){
    const auto& ts = mSimState->stateInit.tailsitter;
    for (int side = 0; side < 2; side++){
      const int idx = clamp(ts.servoIndex[side], 0, 7);
      const float us = clamp(mSimState->servosPwm[idx],
                             ts.servoMinUs, ts.servoMaxUs);
      float cmd = (us - ts.servoCenterUs) * ts.servoSign[side]
                * ts.servoDegPerUs;
      cmd = clamp(cmd, -ts.elevonMaxDeg, ts.elevonMaxDeg);

      // first-order lag combined with a slew-rate limit
      const float tau = std::max(ts.servoTauSec, 1e-4f);
      float rate = (cmd - mServoDeg[side]) / tau;
      const float rMax = ts.servoRateDps;
      rate = clamp(rate, -rMax, rMax);
      mServoDeg[side] += rate * static_cast<float>(dt);
      mServoDeg[side] = clamp(mServoDeg[side], -ts.elevonMaxDeg, ts.elevonMaxDeg);

      mSimState->elevonDeg[side] = mServoDeg[side];
      mSimState->stateOutput.elevonDeg[side] = mServoDeg[side];
    }
  }

  // --- motor model, same equations as Physics (quad) ---------------------------

  static float tsMotorTorque(float volts, float rpm, float kV, float R, float I0){
    const float minKV = std::max(kV, 0.0001f);
    const float backEmfV = rpm / minKV;
    float current = (volts - backEmfV) / std::max(R, 0.0001f);
    const float nmPerA = 8.3f / minKV;
    float torque = current * nmPerA;
    const float friction = I0 * nmPerA;
    if (rpm > 1.0f)      torque -= friction;
    else if (rpm < -1.0f) torque += friction;
    else {
      if (torque > friction) torque -= friction;
      else if (torque < -friction) torque += friction;
      else torque = 0.0f;
    }
    return torque;
  }

  static float tsMotorCurrent(float torque, float kV){
    return torque * kV / 8.3f;
  }

  float TailsitterPhysics::calculateMotors(double dt){
    const auto& ts = mSimState->stateInit.tailsitter;
    const auto& init = mSimState->stateInit;

    mat3 rotation;
    copy(rotation, mSimState->stateInput.rotation);
    const mat3 basisA = rotation * transpose(SIM_TO_AERO);

    vec3 linVel;
    copy(linVel, mSimState->stateInput.linearVelocity);
    vec3 angVel;
    copy(angVel, mSimState->stateInput.angularVelocity);
    const vec3 vA = xform_inv(basisA, linVel);
    const vec3 wA = xform_inv(basisA, angVel);

    // prop disk stations, aerospace frame {right, left}
    const float motorXAero = ts.cgX - ts.motorXAft; // x_a forward from CG
    const vec3 motorArm[2] = {
      {motorXAero,  ts.motorY, ts.motorZ},
      {motorXAero, -ts.motorY, ts.motorZ},
    };

    float resTorqueX = 0.0f;

    for (int i = 0; i < 2; i++){
      // per-side state lives in motorsState[side]; the command comes from
      // whichever Betaflight motor output drives this side
      MotorState& m = mSimState->motorsState[i];
      const float pwm = motorPwm(i);

      // axial inflow at the prop disk
      const vec3 vLoc = vA + cross(wA, motorArm[i]);
      const float vel = vLoc[0];

      float vbat = std::max(1.0f, mSimState->batteryState.batVoltageSag);

      // ESC transport delay + PWM lowpass, as the closed-loop quad path
      m.pwmHistory.push_back(pwm);
      constexpr float escDelay = 0.002f;
      size_t delaySamples = static_cast<size_t>(escDelay / std::max(dt, 0.000001));
      if (delaySamples < 1) delaySamples = 1;
      float delayedPwm = pwm;
      while (m.pwmHistory.size() > delaySamples){
        delayedPwm = m.pwmHistory.front();
        m.pwmHistory.pop_front();
      }
      float volts = 0.0f;
      if (mSimState->stateInput.openLoop == 1){
        volts = m.pwmLowPassFilter.update(pwm, dt, 500.0f) * vbat;
      } else {
        volts = m.pwmLowPassFilter.update(delayedPwm, dt, 100.0f) * vbat;
      }
      if (!mSimState->armed && mSimState->stateInput.openLoop == 0){
        volts = 0.0f;
      }

      const float kV = init.motorKV[i];
      const float R  = init.motorR[i];
      const float I0 = init.motorI0[i];

      const float mTorque = tsMotorTorque(volts, m.rpm, kV, R, I0);
      const float current = tsMotorCurrent(mTorque, kV);

      // prop thrust: same quadratic form as Physics::propThrust
      const float propF = std::max(0.0f,
          init.propThrustFactor.x * vel * vel +
          init.propThrustFactor.y * vel +
          init.propThrustFactor.z);
      const float maxRpm = std::max(init.propMaxRpm, 0.01f);
      const float b = (propF - init.propAFactor * maxRpm * maxRpm) / maxRpm;
      float thrust = std::max(b * m.rpm + init.propAFactor * m.rpm * m.rpm, 0.0f);
      thrust *= ts.thrustScale;
      const float pTorque = thrust * init.propTorqueFactor;

      const float netTorque = mTorque - pTorque;
      const float domega = netTorque / std::max(init.propInertia, 1e-8f);
      const float drpm = (domega * static_cast<float>(dt)) * 60.0f
                       / (2.0f * float(M_PI));
      const float maxdrpm = fabsf(volts * kV - m.rpm);
      m.rpm += clamp(drpm, -maxdrpm, maxdrpm);

      m.current = current;
      m.mTorque = mTorque;
      m.pTorque = pTorque;
      m.netTorque = netTorque;
      m.thrust = thrust;

      // airframe reaction torque about x_a: -(spin dir) * motor torque
      resTorqueX += -static_cast<float>(ts.motorSpinDir[i]) * mTorque;
    }

    return resTorqueX;
  }

  // --- battery, same model as Physics -------------------------------------------

  void TailsitterPhysics::updateBat(double dt){
    const auto& init = mSimState->stateInit;
    auto& bat = mSimState->batteryState;
    const double capFull = std::max(init.quadBatCapacity, 1.0f);
    bat.batVoltage = mBatVoltageCurve.sample(
        1.0f - static_cast<float>(bat.batCapacity / capFull))
        * init.quadBatCellCount;
    bat.batVoltage = std::max(bat.batVoltage, 0.1f);

    const float pwmSum = mSimState->motorsState[0].pwm
                       + mSimState->motorsState[1].pwm;
    const float powerFactor = std::max(0.0f, pwmSum / 2.0f);
    const float powerFactor2 = powerFactor * powerFactor;
    const float chargeFactorInv = 1.0f - static_cast<float>(bat.batCapacity) /
        std::max(init.quadBatCapacityCharged, 1.0f);
    const float vSag = init.maxVoltageSag * powerFactor2
        + init.maxVoltageSag * chargeFactorInv * chargeFactorInv * powerFactor2;
    bat.batVoltageSag = clamp(bat.batVoltage - vSag, 0.0f, 100.0f);

    float batteryCurrent = 0.0f;
    for (int i = 0; i < 2; i++){
      batteryCurrent += fabsf(mSimState->motorsState[i].current)
                      * clamp(motorPwm(i), 0.0f, 1.0f);
    }
    double currentmAs = batteryCurrent / 3.6;
    const double mAMin = std::min(0.2, 0.5 /
        std::max(bat.batVoltageSag, 0.01f));
    currentmAs = std::max(currentmAs, mAMin);
    bat.amperage = currentmAs * 3.6;
    bat.batCapacity -= currentmAs * dt;
    bat.mAhDrawn = init.quadBatCapacityCharged - bat.batCapacity;
  }

  // --- main step ----------------------------------------------------------------

  void TailsitterPhysics::updateRotation(double dt, StateInput& state){
    // world-frame quaternion integration, identical to Physics::updateRotation
    vec3 angularVelocity;
    copy(angularVelocity, state.angularVelocity);

    mat3 rotation;
    copy(rotation, state.rotation);
    quat q = mat3_to_quat(rotation);

    quat qw = {angularVelocity[0], angularVelocity[1], angularVelocity[2], 0.0f};
    quat qdot = quat_multiply(qw, q);
    const float hdt = 0.5f * static_cast<float>(dt);
    q[0] += qdot[0] * hdt;
    q[1] += qdot[1] * hdt;
    q[2] += qdot[2] * hdt;
    q[3] += qdot[3] * hdt;
    q = quat_normalize(q);

    mat3 nr = quat_to_mat3(q);
    copy(state.rotation, nr);
  }

  void TailsitterPhysics::updatePhysics(double dt){
    const auto& ts = mSimState->stateInit.tailsitter;
    StateInput& state = mSimState->stateInput;

    updateServos(dt);
    const float torqueX = calculateMotors(dt);

    mat3 basis;
    copy(basis, state.rotation);
    const mat3 basisA = basis * transpose(SIM_TO_AERO);

    vec3 vW; copy(vW, state.linearVelocity);
    vec3 wW; copy(wW, state.angularVelocity);

    const float thrust[2] = {mSimState->motorsState[0].thrust,
                             mSimState->motorsState[1].thrust};

    vec3 fAero, mAero;
    computeForces(vW, wW, basis, mServoDeg[0], mServoDeg[1], thrust,
                  fAero, mAero);

    // thrust force along +x_a plus its moment arms
    const float motorXAero = ts.cgX - ts.motorXAft;
    const vec3 motorArm[2] = {
      {motorXAero,  ts.motorY, ts.motorZ},
      {motorXAero, -ts.motorY, ts.motorZ},
    };
    for (int i = 0; i < 2; i++){
      const vec3 fT{thrust[i], 0.0f, 0.0f};
      fAero = fAero + fT;
      mAero = mAero + cross(motorArm[i], fT);
    }
    // motor reaction torque (props counter-rotating cancels in hover)
    mAero[0] += torqueX;
    // rotor spin gyroscopic: -(w x L), L = J * dir * rpm along +x_a
    const float jProp = std::max(mSimState->stateInit.propInertia, 0.0f);
    const vec3 wA = xform_inv(basisA, wW);
    for (int i = 0; i < 2; i++){
      const float l = jProp * static_cast<float>(ts.motorSpinDir[i])
                    * (mSimState->motorsState[i].rpm * 2.0f * float(M_PI) / 60.0f);
      const vec3 lvec{l, 0.0f, 0.0f};
      mAero = mAero - cross(wA, lvec);
    }

    // forces to world, integrate velocity
    vec3 fW = xform(basisA, fAero);
    const vec3 grav{0.0f, -TS_G * std::max(ts.mass, 1e-3f), 0.0f};
    fW = fW + grav;

    vec3 acc = fW / std::max(ts.mass, 1e-3f);
    mSimState->acceleration = acc;
    vec3 linVel = vW + acc * static_cast<float>(dt);
    copy(state.linearVelocity, linVel);

    // moments in aero body frame, integrate omega
    const vec3 wAx = xform_inv(basisA, wW);
    const vec3 iw = matVec(mInertia, wAx);
    const vec3 gyroT = cross(wAx, iw);
    const vec3 wDot = matVec(mInvInertia, mAero - gyroT);
    vec3 wNewA = wAx + wDot * static_cast<float>(dt);
    wNewA = clamp(wNewA, -100.0f, 100.0f);
    vec3 wNewW = xform(basisA, wNewA);
    copy(state.angularVelocity, wNewW);

    updateRotation(dt, state);

    updateBat(dt);

    // outputs
    mat3 basisOut;
    copy(basisOut, state.rotation);
    quat orientation = mat3_to_quat(basisOut);
    mSimState->stateOutput.orientation.w = orientation[3];
    mSimState->stateOutput.orientation.x = orientation[0];
    mSimState->stateOutput.orientation.y = orientation[1];
    mSimState->stateOutput.orientation.z = orientation[2];
    mSimState->stateOutput.angularVelocity = state.angularVelocity;
    mSimState->stateOutput.linearVelocity = state.linearVelocity;

    for (int i = 0; i < 2; i++){
      const MotorState& m = mSimState->motorsState[i];
      mSimState->stateOutput.motorOutput[i] = motorPwm(i);   // per side, like rpm
      mSimState->stateOutput.motorRpm[i] = m.rpm;
      mSimState->stateOutput.motorTorque[i] = m.mTorque;
      mSimState->stateOutput.propellerTorque[i] = m.pTorque;
      mSimState->stateOutput.netTorque[i] = m.netTorque;
      mSimState->stateOutput.thrust[i] = m.thrust;
      mSimState->stateOutput.motorCurrent[i] = m.current;
      mSimState->stateOutput.motorT[i] = m.temp;
      mSimState->stateOutput.motorStatus[i] = m.status;
    }
    mSimState->stateOutput.batteryVoltage = mSimState->batteryState.batVoltage;
  }

  void TailsitterPhysics::repair(){
    mSimState->batteryState.batCapacity =
        mSimState->stateInit.quadBatCapacityCharged;
    for (int i = 0; i < 2; i++){
      mSimState->motorsState[i].temp = mSimState->stateInit.ambientTemp;
      mSimState->motorsState[i].status = MotorStatus::MotorNone;
    }
  }

  void TailsitterPhysics::reset(){
    mSimState->acceleration = {0.0f, 0.0f, 0.0f};
    mSimState->stateInput.linearVelocity = {0.0f, 0.0f, 0.0f};
    mSimState->stateInput.angularVelocity = {0.0f, 0.0f, 0.0f};
  }

}
