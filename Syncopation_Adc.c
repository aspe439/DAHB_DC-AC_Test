/*
 * MSST_ADC.c
 *
 *  Created on: April, 2019
 *      Author: Ao Sun
 */

#include "F28x_Project.h"
#include "Syncopation_Data.h"
#include "Syncopation_Pwm.h"
#include "PI.h"
#include "Lookup.h"

#define DSP_CONTROLLOOP_TGL GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1
#define SAMPLE_WINDOW 49
#define ADC_TRIG_SELECT  5  // EPWM1 SOCA
#define R_batt 1

void Adc_A_Init();
void Adc_B_Init();
void Adc_C_Init();
void Adc_D_Init();


interrupt void ControlLoop(void);

void AdcInit()
{
    Adc_A_Init();
    Adc_B_Init();
    Adc_C_Init();
    Adc_D_Init();
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // ADC-A interrupt 1
    EALLOW;
    PieVectTable.ADCA1_INT = &ControlLoop;
    AnalogSubsysRegs.TSNSCTL.bit.ENABLE = 1;
    EDIS;
}

void Adc_A_Init()
{
    volatile struct ADC_REGS *adc_module = &AdcaRegs;
    EALLOW;
    adc_module->ADCCTL1.bit.ADCPWDNZ = 1;  // ADC-A power up
    adc_module->ADCCTL1.bit.INTPULSEPOS = 1; // Interrupt position at the end of conversion
    adc_module->ADCCTL2.bit.PRESCALE = 2; // ADC Clock = SYSCLK / 2;
    adc_module->ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
    adc_module->ADCCTL2.bit.SIGNALMODE = 0; // Single-ended signal mode
    adc_module->ADCBURSTCTL.bit.BURSTEN = 0; // Disable burst mode
    adc_module->ADCINTFLGCLR.all = 15; // Clear all the interrupt flags
    adc_module->ADCINTOVFCLR.all = 15; // Clear all the interrupt overflow flags
    adc_module->ADCINTSEL1N2.all = 0x0023; //  Enable ADCINT1 and it is triggered by EOC3, disable ADCINT2
    adc_module->ADCINTSEL3N4.all = 0; //  Disable ADCINT3 and ADCINT4
    adc_module->ADCSOCPRICTL.all = 0; // Round robin control. Conversion starts from SOC0.
    adc_module->ADCINTSOCSEL1.all = 0; // ADC interrupt doesn't trigger any SOC
    adc_module->ADCINTSOCSEL2.all = 0; // ADC interrupt doesn't trigger any SOC

    adc_module->ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC0CTL.bit.CHSEL = 0; // SOC-0 convert channel 1, which is ADC_A1 pin
    adc_module->ADCSOC0CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC1CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC1CTL.bit.CHSEL = 1; // SOC-1 convert channel 2, which is ADC_A2 pin
    adc_module->ADCSOC1CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC2CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-2 software start
    adc_module->ADCSOC2CTL.bit.CHSEL = 2; // SOC-2 convert channel 3, which is ADC_A3 pin
    adc_module->ADCSOC2CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC3CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-3 software start
    adc_module->ADCSOC3CTL.bit.CHSEL = 3; // SOC-3 convert channel 4, which is ADC_A4 pin
    adc_module->ADCSOC3CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC4CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC4CTL.bit.CHSEL = 4; // SOC-0 convert channel 5, which is ADC_A5 pin
    adc_module->ADCSOC4CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC5CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC5CTL.bit.CHSEL = 5; // SOC-1 convert channel 6, which is ADC_A6 pin
    adc_module->ADCSOC5CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC6CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-2 software start
    adc_module->ADCSOC6CTL.bit.CHSEL = 13; // SOC-2 convert channel 4, which is ADC_A4 pin
    adc_module->ADCSOC6CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCCTL1.bit.INTPULSEPOS = 1;    // ADCINT1 trips after AdcResults latch
    adc_module->ADCINTSEL1N2.bit.INT1E     = 1;    // Enabled ADCINT1
    //adc_module->ADCINTSEL1N2.bit.INT1CONT  = 0;    // Disable ADCINT1 Continuous mode
    adc_module->ADCINTSEL1N2.bit.INT1CONT  = 1;    // Disable ADCINT1 Continuous mode
    adc_module->ADCINTSEL1N2.bit.INT1SEL   = 6;    // setup EOC3 to trigger ADCINT1 to fire
    EDIS;
}

void Adc_B_Init()
{
    volatile struct ADC_REGS *adc_module = &AdcbRegs;
    EALLOW;
    adc_module->ADCCTL1.bit.ADCPWDNZ = 1;  // ADC-A power up
    adc_module->ADCCTL1.bit.INTPULSEPOS = 1; // Interrupt position at the end of conversion
    adc_module->ADCCTL2.bit.PRESCALE = 2; // ADC Clock = SYSCLK / 2;
    adc_module->ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
    adc_module->ADCCTL2.bit.SIGNALMODE = 0; // Single-ended signal mode
    adc_module->ADCBURSTCTL.bit.BURSTEN = 0; // Disable burst mode
    adc_module->ADCINTFLGCLR.all = 15; // Clear all the interrupt flags
    adc_module->ADCINTOVFCLR.all = 15; // Clear all the interrupt overflow flags
    adc_module->ADCINTSEL1N2.all = 0x0003; //  Enable ADCINT1 and it is triggered by EOC3, disable ADCINT2
    adc_module->ADCINTSEL3N4.all = 0; //  Disable ADCINT3 and ADCINT4
    adc_module->ADCSOCPRICTL.all = 0; // Round robin control. Conversion starts from SOC0.
    adc_module->ADCINTSOCSEL1.all = 0; // ADC interrupt doesn't trigger any SOC
    adc_module->ADCINTSOCSEL2.all = 0; // ADC interrupt doesn't trigger any SOC

    adc_module->ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC0CTL.bit.CHSEL = 0; // SOC-0 convert channel 2, which is ADC_A2 pin
    adc_module->ADCSOC0CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC1CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC1CTL.bit.CHSEL = 1; // SOC-1 convert channel 3, which is ADC_A3 pin
    adc_module->ADCSOC1CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC2CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-2 software start
    adc_module->ADCSOC2CTL.bit.CHSEL = 2; // SOC-2 convert channel 4, which is ADC_A4 pin
    adc_module->ADCSOC2CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC3CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-3 software start
    adc_module->ADCSOC3CTL.bit.CHSEL = 3; // SOC-3 convert channel 5, which is ADC_A5 pin
    adc_module->ADCSOC3CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC4CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC4CTL.bit.CHSEL = 4; // SOC-0 convert channel 2, which is ADC_A2 pin
    adc_module->ADCSOC4CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC5CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC5CTL.bit.CHSEL = 5; // SOC-1 convert channel 3, which is ADC_A3 pin
    adc_module->ADCSOC5CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    EDIS;
}
void Adc_C_Init()
{
    volatile struct ADC_REGS *adc_module = &AdccRegs;
    EALLOW;
    adc_module->ADCCTL1.bit.ADCPWDNZ = 1;  // ADC-A power up
    adc_module->ADCCTL1.bit.INTPULSEPOS = 1; // Interrupt position at the end of conversion
    adc_module->ADCCTL2.bit.PRESCALE = 2; // ADC Clock = SYSCLK / 2;
    adc_module->ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
    adc_module->ADCCTL2.bit.SIGNALMODE = 0; // Single-ended signal mode
    adc_module->ADCBURSTCTL.bit.BURSTEN = 0; // Disable burst mode
    adc_module->ADCINTFLGCLR.all = 15; // Clear all the interrupt flags
    adc_module->ADCINTOVFCLR.all = 15; // Clear all the interrupt overflow flags
    adc_module->ADCINTSEL1N2.all = 0x0003; //  Enable ADCINT1 and it is triggered by EOC3, disable ADCINT2
    adc_module->ADCINTSEL3N4.all = 0; //  Disable ADCINT3 and ADCINT4
    adc_module->ADCSOCPRICTL.all = 0; // Round robin control. Conversion starts from SOC0.
    adc_module->ADCINTSOCSEL1.all = 0; // ADC interrupt doesn't trigger any SOC
    adc_module->ADCINTSOCSEL2.all = 0; // ADC interrupt doesn't trigger any SOC

    adc_module->ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC0CTL.bit.CHSEL = 14; // SOC-0 convert channel 2, which is ADC_A2 pin
    adc_module->ADCSOC0CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC1CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC1CTL.bit.CHSEL = 15; // SOC-1 convert channel 3, which is ADC_A3 pin
    adc_module->ADCSOC1CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC2CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-2 software start
    adc_module->ADCSOC2CTL.bit.CHSEL = 2; // SOC-2 convert channel 4, which is ADC_A4 pin
    adc_module->ADCSOC2CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC3CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-3 software start
    adc_module->ADCSOC3CTL.bit.CHSEL = 3; // SOC-3 convert channel 5, which is ADC_A5 pin
    adc_module->ADCSOC3CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC4CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC4CTL.bit.CHSEL = 4; // SOC-0 convert channel 2, which is ADC_A2 pin
    adc_module->ADCSOC4CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC5CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC5CTL.bit.CHSEL = 5; // SOC-1 convert channel 3, which is ADC_A3 pin
    adc_module->ADCSOC5CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    EDIS;
}

void Adc_D_Init()
{
    volatile struct ADC_REGS *adc_module = &AdcdRegs;
    EALLOW;
    adc_module->ADCCTL1.bit.ADCPWDNZ = 1;  // ADC-A power up
    adc_module->ADCCTL1.bit.INTPULSEPOS = 1; // Interrupt position at the end of conversion
    adc_module->ADCCTL2.bit.PRESCALE = 2; // ADC Clock = SYSCLK / 2;
    adc_module->ADCCTL2.bit.RESOLUTION = 0; // 12-bit resolution
    adc_module->ADCCTL2.bit.SIGNALMODE = 0; // Single-ended signal mode
    adc_module->ADCBURSTCTL.bit.BURSTEN = 0; // Disable burst mode
    adc_module->ADCINTFLGCLR.all = 15; // Clear all the interrupt flags
    adc_module->ADCINTOVFCLR.all = 15; // Clear all the interrupt overflow flags
    adc_module->ADCINTSEL1N2.all = 0x0003; //  Enable ADCINT1 and it is triggered by EOC3, disable ADCINT2
    adc_module->ADCINTSEL3N4.all = 0; //  Disable ADCINT3 and ADCINT4
    adc_module->ADCSOCPRICTL.all = 0; // Round robin control. Conversion starts from SOC0.
    adc_module->ADCINTSOCSEL1.all = 0; // ADC interrupt doesn't trigger any SOC
    adc_module->ADCINTSOCSEL2.all = 0; // ADC interrupt doesn't trigger any SOC

    adc_module->ADCSOC0CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC0CTL.bit.CHSEL = 0; // SOC-0 convert channel 1, which is ADC_A1 pin
    adc_module->ADCSOC0CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC1CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC1CTL.bit.CHSEL = 1; // SOC-1 convert channel 2, which is ADC_A2 pin
    adc_module->ADCSOC1CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC2CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-2 software start
    adc_module->ADCSOC2CTL.bit.CHSEL = 2; // SOC-2 convert channel 3, which is ADC_A3 pin
    adc_module->ADCSOC2CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC3CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-3 software start
    adc_module->ADCSOC3CTL.bit.CHSEL = 3; // SOC-3 convert channel 4, which is ADC_A4 pin
    adc_module->ADCSOC3CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC4CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-0 software start
    adc_module->ADCSOC4CTL.bit.CHSEL = 4; // SOC-4 convert channel 5, which is ADC_A5 pin
    adc_module->ADCSOC4CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    adc_module->ADCSOC5CTL.bit.TRIGSEL = ADC_TRIG_SELECT; // SOC-1 software start
    adc_module->ADCSOC5CTL.bit.CHSEL = 5; // SOC-5 convert channel 6, which is ADC_A6 pin
    adc_module->ADCSOC5CTL.bit.ACQPS = SAMPLE_WINDOW; // Sample window is SAMPLE_WINDOW+1 clock cycles

    EDIS;
}


Uint16 AdcResult[24];
Uint16 TempSensor;
Uint16 vac_reading = 0;
Uint16 iac_reading = 0;

//////////Battery Voltage chopper definition
float Iboost_reading = 0;
float Ibb_reading = 0;
float Vbatt_reading = 0;
float Iboost_Reference = 25;
float Vbatt_Ref = 880;
PI_Controller PIBoost={         //output is duty cycle
                       -100,// Iref. Iref>0 means discharging
                       0,
                       0,
                       0.0005,//Kp
                       362,//Ki
                       6.67e-5,
                       0.75,
                       0.2,
                       0,0,0,0,
                       0.2,//feedforward
                       0.03
};
PI_Controller PIVBoost={        //output is reference current
                        950,
                        0,
                        0,
                        0.1,
                        1,
                        6.67e-5,
                        300,
                        -300,
                        0,0,0,0,
                        -100,//feedforward
                        1
};
Uint16 BoostScenerio=0;



//extern float SinglePhasePLL(float Vac, float *Freq, float *Vac_amp);

extern lookup_table DAHB_20;
extern Uint16 dab_prd;
extern int16 dab_phs;
Uint16 DABScenerio = 0;
Uint16 DAB_Supevisor=0;

float Vac,Iac;
float Freq, Vac_amp;
float Theta;


float index = 0;
Uint16 state = 0;

Uint16 DAB_Start_Flag = 0;
Uint16 DAB_Trip_Flag = 0;
Uint16 fault_status = 0;

#pragma CODE_SECTION(ControlLoop, ".TI.ramfunc");
__interrupt void ControlLoop(void)
{
    GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1;
    AdcResult[0]  = AdcaResultRegs.ADCRESULT0;
    AdcResult[1]  = AdcaResultRegs.ADCRESULT1;
    AdcResult[2]  = AdcaResultRegs.ADCRESULT2;
    AdcResult[3]  = AdcaResultRegs.ADCRESULT3;//boost voltage sensor
    AdcResult[4]  = AdcaResultRegs.ADCRESULT4;//boost current sensor
    AdcResult[5]  = AdcaResultRegs.ADCRESULT5;
    TempSensor    = AdcaResultRegs.ADCRESULT6;
    AdcResult[6]  = AdcbResultRegs.ADCRESULT0;
    AdcResult[7]  = AdcbResultRegs.ADCRESULT1;
    AdcResult[8]  = AdcbResultRegs.ADCRESULT2;
    AdcResult[9]  = AdcbResultRegs.ADCRESULT3;
    AdcResult[10] = AdcbResultRegs.ADCRESULT4;
    AdcResult[11] = AdcbResultRegs.ADCRESULT5;
    AdcResult[12] = AdccResultRegs.ADCRESULT0;
    AdcResult[13] = AdccResultRegs.ADCRESULT1;
    AdcResult[14] = AdccResultRegs.ADCRESULT2;
    AdcResult[15] = AdccResultRegs.ADCRESULT3;
    AdcResult[16] = AdccResultRegs.ADCRESULT4;
    AdcResult[17] = AdccResultRegs.ADCRESULT5;
    AdcResult[18] = AdcdResultRegs.ADCRESULT0;
    AdcResult[19] = AdcdResultRegs.ADCRESULT1;
    AdcResult[20] = AdcdResultRegs.ADCRESULT2;
    AdcResult[21] = AdcdResultRegs.ADCRESULT3;
    AdcResult[22] = AdcdResultRegs.ADCRESULT4;
    AdcResult[23] = AdcdResultRegs.ADCRESULT5;

    //Battery Side
    //Iboost_reading = 0.072351*AdcResult[4]+3.8448;
    Iboost_reading = 0.072351*AdcResult[4]+0.8448;//HIL Gain:2.5/225 offset:0
    Ibb_reading = 1.5625*Iboost_reading-225;//Negative sign means charging HIL Gain:1.6/225 offset:1.6
    Vbatt_reading = 0.25457*AdcResult[3]+13.528;

////Boost Control part
    switch(BoostScenerio)
    {
    case 0:Boost_DIS();break;
    case 1:{            //discharging
        Boost_EN();
        PIBoost.Fbk = Ibb_reading;
        Ramp_Ref(Iboost_Reference,&PIBoost);
        PI_Cal(&PIBoost);
        Boost_SET(PIBoost.Out);
        break;
    }
    case 2:{            //charging
        Boost_EN();
        PIBoost.Fbk = Ibb_reading;
        Ramp_Ref(Iboost_Reference,&PIBoost);
        PI_Cal(&PIBoost);
        Boost_SET(PIBoost.Out);
        float temp_Vbatt_Ref = Vbatt_Ref-Ibb_reading*R_batt;//Vbatt-R*I
        if(Vbatt_reading>temp_Vbatt_Ref)
            {
            BoostScenerio=3;
            PIVBoost.Feedforward=(-1)*PIBoost.Ref;//the feedforward will be inverted latter
            }
        break;
    }
    case 3:{            //charging
        if(Vbatt_reading<(Vbatt_Ref-70))
            BoostScenerio = 2;
        else if(Ibb_reading>-5)
            Boost_DIS();
        else{
            PIVBoost.Fbk=Vbatt_reading; //Voltage loop
            PI_Cal(&PIVBoost);
            Ramp_Ref(-PIVBoost.Out,&PIBoost);
            PIBoost.Fbk = Ibb_reading;
            PI_Cal(&PIBoost);
            Boost_SET(PIBoost.Out);
        }
        break;
    }
    default : Boost_DIS(); break;
    }

///// DAB Control part
    switch(DABScenerio)
    {
    case 0: index = 0;break;

    case 1:{
        Dab_Update();
        if(DAB_Supevisor==2){
            dab_prd=3077;
            dab_phs=0;
            DABScenerio = 2;
            index = 0;
        }
        break;
    }

    case 2:
    {
        //Dab_EN();
        index += 1;
        int index_table = (int) index;
        if(index>=180)
        {
            index_table = 0;
            index -= 180;
        }
        dab_prd = DAHB_20.Prd[index_table];
        float PhS_Angle = DAHB_20.PhShift[index_table];
        dab_phs = dab_prd*PhS_Angle*0.00277778;
        Dab_Update();

        //trip_zone part
        if((index == 177)||(index == 178))
            DAB_Trip_Flag = 1;
        else if (!inRange(3, 178, index)) //if index<3 or index > 177, forced trip. In case of adc interrupt fails
        {
            EALLOW;
            EPwm1Regs.TZFRC.bit.OST = 1;
            EPwm2Regs.TZFRC.bit.OST = 1;
            EPwm3Regs.TZFRC.bit.OST = 1;
            EPwm4Regs.TZFRC.bit.OST = 1;
            EPwm5Regs.TZFRC.bit.OST = 1;
            EPwm6Regs.TZFRC.bit.OST = 1;
            EDIS;
            //DAB_Start_Flag = 0;

            if((DAB_Supevisor==1))
            {
                DABScenerio=1;
                dab_prd=4999;
                dab_phs=0;
                index = 0;
                DAB_Start_Flag = 0;
                DAB_Trip_Flag = 0;
                break;
            }
        }
        else if((index == 3)||(index == 4))
            DAB_Start_Flag = 1;
        break;}


    default: Dab_DIS(); break;
    }


    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//Clear ADCINT1 flag reinitialize for next SOC
	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
	GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1;
}

void AC_Power_Switch(Uint16 status)
{
    DAB_Supevisor = status;
}

__interrupt void Xint1Isr()
{
//    fault_status = 1;
    Dab_DIS();
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}


