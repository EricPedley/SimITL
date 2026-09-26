#ifndef SIM_MPC_H
#define SIM_MPC_H

#ifdef __cplusplus
extern "C" {
#endif

// Called once per motor-write cycle from the simITL target's
// pwmCompleteMotorUpdate().  When the BOXMPC ("MPC CONTROL") mode is active
// and the quad is armed, overwrites motorsPwm[] with the generated MPC's
// commands.
void simMpcUpdate(void);

// Diagnostics for bridge telemetry.
int simMpcActive(void);
double simMpcLastSolveUs(void);
double simMpcBias(int axis);
double simMpcColl(void);
double simMpcUref(int i);

#ifdef __cplusplus
}
#endif

#endif
