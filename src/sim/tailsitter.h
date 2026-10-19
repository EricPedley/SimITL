#ifndef TAILSITTER_PHYSICS_H
#define TAILSITTER_PHYSICS_H

#include <cstring>
#include <vector>

#include "state.h"
#include "util/SampleCurve.h"
#include "util/LowPassFilter.h"

namespace SimITL{

  /**
   * 6-DOF physics for the dual-motor tailsitter flying wing.
   *
   * Internal aerodynamics work in an aerospace body frame
   *   x_a = forward (nose), y_a = right wing tip, z_a = down,
   * which relates to SimITL's sim body frame by a fixed permutation
   *   x_s = -y_a  (port),  y_s = -z_a  (dorsal),  z_s = +x_a  (nose).
   * That permutation plus the bf.cpp sensor remapping
   *   (sensor = (-gyro[2], +gyro[0], -gyro[1])) and the configured
   * align_board_yaw = 180 makes Betaflight's internal body axes coincide
   * with the aerospace axes, so BF roll/pitch/yaw mean the right things.
   *
   * The strip model, section coefficients and all constants are shared with
   * sim/wing_model.py in the parent repo; the two implementations are checked
   * against each other over a state grid (see sim/test_wing_physics.py).
   */
  class TailsitterPhysics{
    public:
      TailsitterPhysics() = default;
      ~TailsitterPhysics() = default;

      bool checkSimState();
      void setSimState(SimState* state);
      void initState(const StateInit& state);
      void updateCommands(CommandType commands);
      void updateState(const StateInput& state);
      void updateGyro(double dt);
      void updatePhysics(double dt);

      // Force/moment evaluation in the aerospace body frame, exposed for the
      // equivalence test against the Python reference. F in N, M in N.m.
      // Does not touch integration state.
      void computeForces(const vec3& velWorld, const vec3& omegaWorld,
                         const mat3& basis,
                         float elevonDegL, float elevonDegR,
                         const float thrustN[2],
                         vec3& outForceAero, vec3& outMomentAero) const;

    private:
      struct Strip {
        float y;        // spanwise centre, m (aerospace y, +right)
        float dy;       // strip width, m
        float chord;    // local chord, m
        float leX;      // local LE, m aft of root LE
        float a1, a2;   // camber coefficients
        float tc;       // thickness ratio
        int   flapSide; // -1 none, 0 left elevon, 1 right elevon
        float clMax;    // from the correlation (evaluated lazily per Re)
      };

      void buildStrips();
      void buildDownwash();
      float inducedVelocity(float thrustN, float vInf, float diskArea) const;
      float clMaxFor(const Strip& s, float re) const;
      void sectionClCdCm(const Strip& s, float alphaEffRad,
                         float& cl, float& cd, float& cm) const;
      void updateServos(double dt);
      float calculateMotors(double dt);
      float motorPwm(int side) const;  // BF command for {right, left}
      void updateBat(double dt);
      void updateRotation(double dt, StateInput& state);
      void repair();
      void reset();

      SimState* mSimState = nullptr;
      std::vector<Strip> mStrips;
      std::vector<float> mDownwash;   // n x n lifting-line influence (row-major)
      float mAspectRatio = 4.0f;
      float mWingArea = 0.0f;
      float mCdmax = 1.5f;

      // inertia tensor, aerospace body frame (flat array, row-major)
      float mInertia[9] = {};
      float mInvInertia[9] = {};

      // servo dynamic state, deg of elevon deflection {left, right}
      float mServoDeg[2] = {0.0f, 0.0f};

      SampleCurve mBatVoltageCurve{{
        {-0.06, 4.4  }, //allows overcharge
        {0.0,   4.2  },
        {0.01,  4.05 },
        {0.04,  3.97 },
        {0.50,  3.85 },
        {0.60,  3.7  },
        {0.85,  3.63 },
        {1.0,   3.49 },
        {1.01,  3.4  },
        {1.03,  3.3  },
        {1.06,  3.0  },
        {1.08,  0.0  }
      }};
  };
}

#endif // TAILSITTER_PHYSICS_H
