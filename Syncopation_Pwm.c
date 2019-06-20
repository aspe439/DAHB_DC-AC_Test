/*
 * MSST_Pwm.c
 *
 *  Created on: April, 2019
 *      Author: Ao Sun
 */

#include "F28x_Project.h"

#define PWM_PRD      4999//40kHZ
#define PWM_CMP      2500
#define PWM_DB       200//160

void Epwm1Init()
{
    EPwm1Regs.TBPRD = 4629;//to fit the look-up table

    EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;
    EPwm1Regs.TBCTL.bit.PHSEN = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;

    EPwm1Regs.AQCTLA.bit.CAU = 2;
    EPwm1Regs.AQCTLA.bit.CAD = 1;

    EPwm1Regs.DBRED.bit.DBRED = PWM_DB-1;
    EPwm1Regs.DBFED.bit.DBFED = PWM_DB-1;
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;
    EPwm1Regs.DBCTL.bit.POLSEL = 2;
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;

    EPwm1Regs.CMPA.bit.CMPA = 2315;

    EPwm1Regs.TBCTL.bit.CTRMODE = 2; // UP-DOWN mode, start the counter

    EPwm1Regs.ETSEL.bit.SOCASEL = 1; // SOCA at counter equals to 0
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable SOCA
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;  // Pulse at every event
}

void Epwm2Init()
{
    volatile struct EPWM_REGS* PWMMODLE = &EPwm2Regs;
    PWMMODLE->TBPRD = 3077;                       // Set timer period. fsw=200MHZ/(TBPRD+1).
    PWMMODLE->TBPHS.bit.TBPHS = 0x0000;          // Phase is 0
    PWMMODLE->TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    PWMMODLE->TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    PWMMODLE->TBCTL.bit.PHSEN = TB_DISABLE;       // Disable phase loading
    PWMMODLE->TBCTL.bit.HSPCLKDIV = TB_DIV1;      // Clock ratio to SYSCLKOUT. TBCLOCK = EPwmClock//1= SYSCLKOUT/1 = 200MHZ
    PWMMODLE->TBCTL.bit.CLKDIV = TB_DIV1;
    PWMMODLE->TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;   //Master
    PWMMODLE->TBCTL.bit.PRDLD = TB_SHADOW;

    PWMMODLE->CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    //PWMMODLE->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    PWMMODLE->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    //PWMMODLE->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // Set actions
    PWMMODLE->AQCTLA.bit.CAU = 2; // Set output A at TBCTR = 0
    PWMMODLE->AQCTLA.bit.CBU = 1; // Clear output A at TBCTR = CMPA
    PWMMODLE->CMPA.bit.CMPA = 0;
    PWMMODLE->CMPB.bit.CMPB = 1538;

//    PWMMODLE->AQCTLB.bit.CAU = AQ_CLEAR;         // Set PWM1B on Zero
//    PWMMODLE->AQCTLB.bit.ZRO = AQ_SET;

    // Active Low PWMs - Setup Deadband
    PWMMODLE->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    PWMMODLE->DBCTL.bit.POLSEL = DB_ACTV_HIC;
    PWMMODLE->DBCTL.bit.IN_MODE = DBA_ALL;

    PWMMODLE->DBRED.bit.DBRED = PWM_DB-1;
    PWMMODLE->DBFED.bit.DBFED = PWM_DB-1;
    EALLOW;
    PWMMODLE->TZCTL.bit.TZA = TZ_FORCE_LO;
    PWMMODLE->TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;

}

void Epwm3Init()
{

    volatile struct EPWM_REGS* PWMMODLE = &EPwm3Regs;
    PWMMODLE->TBPRD = 3077;                       // Set timer period.fsw=200MHZ/TBPRD
    PWMMODLE->TBPHS.bit.TBPHS = 0;          // Phase is 0. TBPHS/TBPRD = g  PHS=445<->g=80'
    PWMMODLE->TBCTR = 0x0000;                     // Clear counter

    // Setup TBCLK
    PWMMODLE->TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
    PWMMODLE->TBCTL.bit.PHSEN = TB_ENABLE;       // Enable phase loading
    PWMMODLE->TBCTL.bit.HSPCLKDIV = TB_DIV1;      // Clock ratio to SYSCLKOUT
    PWMMODLE->TBCTL.bit.CLKDIV = TB_DIV1;         // Slow just to observe on the scope

    PWMMODLE->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  //Slave
    PWMMODLE->TBCTL.bit.PRDLD = TB_SHADOW;
    PWMMODLE->TBCTL2.bit.PRDLDSYNC = 2;         //Load occurs only when a SYNC is received.
    //EPwm3Regs.EPWMXLINK.bit.TBPRDLINK   =   1;      // EPwm3 TBPRD Link to EPwm2

    PWMMODLE->CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    PWMMODLE->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    // Setup compare

    PWMMODLE->AQCTLA.bit.CAU = AQ_SET;            // Set PWM2A on CTR=CMPA
    PWMMODLE->AQCTLA.bit.CBU = AQ_CLEAR;          // Clear PWM2A on CTR = CMPB
    PWMMODLE->CMPA.bit.CMPA = 0;
    PWMMODLE->CMPB.bit.CMPB = 1538;

//    PWMMODLE->AQCTLB.bit.CAU = AQ_CLEAR;          // Set PWM2A on Zero
//    PWMMODLE->AQCTLB.bit.ZRO = AQ_SET;

    // Active Low complementary PWMs - setup the deadband
    PWMMODLE->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    PWMMODLE->DBCTL.bit.POLSEL = DB_ACTV_HIC;
    PWMMODLE->DBCTL.bit.IN_MODE = DBA_ALL;

    PWMMODLE->DBRED.bit.DBRED = PWM_DB-1;
    PWMMODLE->DBFED.bit.DBFED = PWM_DB-1;
    EALLOW;
    PWMMODLE->TZCTL.bit.TZA = TZ_FORCE_LO;
    PWMMODLE->TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;

}

void Epwm4Init()//Battery Chopper
{
    EPwm4Regs.TBPRD = 6667;

    EPwm4Regs.TBCTL.bit.SYNCOSEL = 0;
    EPwm4Regs.TBCTL.bit.PHSEN = 0;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;

    EPwm4Regs.AQCTLA.bit.CAU = 2;//Duty cycle of top switch is 1-D. So A is top, B is bottom
    EPwm4Regs.AQCTLA.bit.CAD = 1;

    EPwm4Regs.DBRED.bit.DBRED = PWM_DB-1;
    EPwm4Regs.DBFED.bit.DBFED = PWM_DB-1;
    EPwm4Regs.DBCTL.bit.IN_MODE = 0;
    EPwm4Regs.DBCTL.bit.POLSEL = 2;
    EPwm4Regs.DBCTL.bit.OUT_MODE = 3;
    //EPwm4Regs.DBCTL.bit.OUTSWAP = 3;

    EPwm4Regs.CMPA.bit.CMPA = 2700;//0.405*6667. Duty of bottom switch is 0.405
    EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;   // Load registers every ZERO
    //EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    //EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm4Regs.TBCTL.bit.CTRMODE = 2;

    EALLOW;
    EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;
    EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;
    EDIS;
}
void EpwmInit()
{
    Epwm1Init();
    Epwm2Init();
    Epwm3Init();
    Epwm4Init();
}

void Dab_EN()
{
    EALLOW;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;
}

void Dab_DIS()
{
    EALLOW;
    EPwm2Regs.TZFRC.bit.OST = 1;
    EPwm3Regs.TZFRC.bit.OST = 1;
    EDIS;
}

Uint16 dab_prd = PWM_PRD;
int16 dab_phs = 0;

void DabFreq_INC()
{
    if(dab_prd > 1000)
        dab_prd-=10;
}

void DabFreq_DEC()
{
    if(dab_prd < 10000)
        dab_prd+=10;
}

void DabPhs_INC()
{
    if(dab_phs > -1500)
        dab_phs--;
}

void DabPhs_DEC()
{
    if(dab_phs < 1500)
        dab_phs++;
}

void DabPhs_SET(int16 arg)
{
    if((dab_phs > -1500) && (dab_phs < 1500))
        dab_phs = arg;
}

void Dab_Update()
{
    Uint16 CMP2B, CMP3A, CMP3B;
    CMP2B = (int)(dab_prd>>1);
    CMP3A = dab_phs;
    //CMP3B = (int)(CMP3A + CMP2B);
    if (CMP3A > CMP2B)
     {
         CMP3B = (int)(CMP3A-CMP2B);
     }
     else
     {
         CMP3B = (int)(CMP3A+CMP2B);
     }
    EPwm2Regs.TBPRD     =   dab_prd;
    EPwm2Regs.CMPB.bit.CMPB = CMP2B;
    EPwm3Regs.TBPRD     =   dab_prd;
    EPwm3Regs.CMPA.bit.CMPA = CMP3A;
    EPwm3Regs.CMPB.bit.CMPB = CMP3B;
//    EPwm2Regs.TBPRD = dab_prd;
//    EPwm3Regs.TBPRD = dab_prd;
//
//    Uint16 cmp = dab_prd >> 1;
//
//    EPwm2Regs.CMPA.bit.CMPA = cmp;
//    EPwm3Regs.CMPA.bit.CMPA = cmp;
//
//    EPwm3Regs.TBPHS.bit.TBPHS = dab_phs;
/*
    if(dab_phs >= 0)
    {
        EPwm3Regs.TBPHS.bit.TBPHS = dab_phs;
        EPwm3Regs.TBCTL.bit.PHSDIR = 0;
    }
    else
    {
        EPwm3Regs.TBPHS.bit.TBPHS = -dab_phs;
        EPwm3Regs.TBCTL.bit.PHSDIR = 1;
    }
*/
}

void Boost_EN()
{
    EALLOW;
    EPwm4Regs.TZCLR.bit.OST = 1;
    EDIS;
}


void Boost_DIS()
{
    EALLOW;
    EPwm4Regs.TZFRC.bit.OST = 1;
    EDIS;
}

#pragma CODE_SECTION(Boost_SET, ".TI.ramfunc");
void Boost_SET(float duty)
{
    Uint16 cmp = duty*EPwm4Regs.TBPRD;
    EPwm4Regs.CMPA.bit.CMPA = cmp;
}

