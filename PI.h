/*
 * PI.h
 *
 *  Created on: Apr 13, 2019
 *      Author: aosun
 */

#ifndef PI_H_
#define PI_H_

#include <stdbool.h>

typedef struct //must use typedef struct but not struct directly, to call it in a function
{
    float  Ref;             // Input: reference set-point
    float  Fbk;             // Input: feedback
    float  Out;             // Output: controller output. Duty cycle
    float  Kp;              // Parameter: proportional loop gain
    float  Ki;              // Parameter: integral gain.
    float  Ts;              // Sample time.
    float  Umax;            // Parameter: upper saturation limit
    float  Umin;            // Parameter: lower saturation limit
    float  up;              // Data: proportional term
    float  ui;              // Data: integral term
    float  v1;              // Data: pre-saturated controller output
    float  i1;              // Data: integrator storage: ui(k-1)
    //float  w1;              // Data: saturation record: [u(k-1) - v(k-1)]
    float Feedforward;
    volatile   float Ramp;
} PI_Controller;

extern float satur(float,float,float);
extern void PI_Cal(PI_Controller *v);
extern void Ramp_Ref(float Final_Ref, PI_Controller *v);
//extern unsigned int inRange(unsigned low, unsigned high, unsigned x);
extern bool inRange(unsigned low, unsigned high, unsigned x);

#define PI_CONTROLLER_DEFAULTS {        \
                           0,           \
                           0,           \
                           0,           \
                           1.0,    \
                           0.0,    \
                           1/15000, \
                           1.0,    \
                           0,   \
                           0,    \
                           0,    \
                           0,    \
                           0,    \
                           1,     \
                           0.5,     \
                           1    \
                          }


#endif /* PI_H_ */
