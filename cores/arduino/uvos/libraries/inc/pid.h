#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>
#include "mathmisc.h"

// !
struct pid {
    float p;
    float i;
    float d;
    float iLim;
    float iAccumulator;
    float lastErr;
    float lastDer;
};

// pid2 structure for a PID+setpoint weighting, anti-windup and filtered derivative control
struct pid2 {
    float   u0;
    float   va;
    float   vb;
    float   kp;
    float   bi;
    float   ad;
    float   bd;
    float   br;
    float   beta;
    float   yold;
    float   P;
    float   I;
    float   D;
    uint8_t reconfigure;
};

typedef struct pid_scaler_s {
    float p;
    float i;
    float d;
} pid_scaler;

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// ! Methods to use the pid structures
float pid_apply(struct pid *pid, const float err, float dT);
float pid_apply_setpoint(struct pid *pid, const pid_scaler *scaler, const float setpoint, const float measured, float dT, bool meas_based_d_term);
void pid_zero(struct pid *pid);
void pid_configure(struct pid *pid, float p, float i, float d, float iLim);
void pid_configure_derivative(float cutoff, float gamma);

// Methods for use with pid2 structure
void pid2_configure(struct pid2 *pid, float kp, float ki, float kd, float Tf, float kt, float dT, float beta, float u0, float va, float vb);
void pid2_transfer(struct pid2 *pid, float u0);
float pid2_apply(struct pid2 *pid, const float r, const float y, float ulow, float uhigh);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif /* PID_H */
