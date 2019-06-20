/*
 * PI_Controller.c
 *
 *  Created on: Apr 13, 2019
 *      Author: aosun
 */
#include "PI.h"

#pragma CODE_SECTION(satur, ".TI.ramfunc");

bool inRange(unsigned low, unsigned high, unsigned x)
{
    return  ((x<=high) && (x>=low));
}

float satur(float v1, float vmax, float vmin)
{
    if(v1>vmax)
        return vmax;
    else if(v1<vmin)
        return vmin;
    else return v1;
}

//PI(z) = KP*(1+Ki*Ts*z/(z-1))
#pragma CODE_SECTION(PI_Cal, ".TI.ramfunc");
void PI_Cal(PI_Controller *v)
{
    /* proportional term */
    v->up = v->Ref - v->Fbk;
    float kiZ = v->Ki*v->Ts;
    /* integral term */
    v->ui = (v->Out == v->v1)?(kiZ*v->up+ v->i1) : v->i1;//backward euler. If already saturated, discard the integrator part.
    v->i1 = v->ui;
    /* control output */
    v->v1 = v->Kp*(v->up + v->ui) + v->Feedforward;
    v->Out= satur(v->v1, v->Umax, v->Umin);
    //v.w1 = (v.Out == v.v1) ? _IQ(1.0) : _IQ(0.0);
}

#pragma CODE_SECTION(Ramp_Ref, ".TI.ramfunc");
void Ramp_Ref(float Final_Ref, PI_Controller *v)//ramp the reference. Prevent it from sudden change.
{
    float diff = Final_Ref - v->Ref;
    if(diff>0)
    {
        if(diff<=v->Ramp)
            v->Ref = Final_Ref;
        else v->Ref += v->Ramp;
    }
    if(diff<0)
    {
        if(diff>=-v->Ramp)
            v->Ref = Final_Ref;
        else v->Ref-=v->Ramp;
    }
}
