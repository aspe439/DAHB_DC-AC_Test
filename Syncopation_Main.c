

/**
 * main.c
 */

#include "F28x_Project.h"
#include "Syncopation_SCI.h"
#include "Syncopation_Data.h"
#include "Syncopation_Pwm.h"
#include "PI.h"

#define DSP_LED_1_OFF       GpioDataRegs.GPDSET.bit.GPIO109 = 1
#define DSP_LED_1_ON        GpioDataRegs.GPDCLEAR.bit.GPIO109 = 1
#define DSP_LED_1_TGL       GpioDataRegs.GPDTOGGLE.bit.GPIO109 = 1

#define DSP_LED_2_OFF       GpioDataRegs.GPDSET.bit.GPIO110 = 1
#define DSP_LED_2_ON        GpioDataRegs.GPDCLEAR.bit.GPIO110 = 1
#define DSP_LED_2_TGL       GpioDataRegs.GPDTOGGLE.bit.GPIO110 = 1

#define DSP_MainLoop_TGL    GpioDataRegs.GPATOGGLE.bit.GPIO12 = 1
#define DSP_Timer_TGL       GpioDataRegs.GPATOGGLE.bit.GPIO13 = 1
//#define DSP_SCIINT_TGL   GpioDataRegs.GPATOGGLE.bit.GPIO14 = 1
//#define DSP_CONTROLLOOP_TGL GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1

#define FPGA_RESET          GpioDataRegs.GPASET.bit.GPIO24 = 1
#define FPGA_RELEASE        GpioDataRegs.GPACLEAR.bit.GPIO24 = 1

extern void Syncopation_Init(void);
void MainLoop();


Uint16 *fpga_led    =    (Uint16 *)0x00100010;
Uint16 *fpga_relay  =    (Uint16 *)0x00100040;

extern Uint16 iac_reading;
extern Uint16 dab_prd;
extern int16 dab_phs;
extern Uint16 DAB_Start_Flag ;
extern Uint16 DAB_Trip_Flag;
extern Uint16 DAB_Supevisor;
//int *fpga_rtd = (int *)0x00100054;
#define fpga_rtd1  (int *)0x00100054
#define fpga_rtd1_addr 0x00100054
#define fpga_rtd2_addr 0x00100055
int read_rtd1;
int read_rtd2;

//void sdram_test();
Uint32 sdram_data;

extern volatile struct SCI_REGS *sci_ch;

void main(void) {
    InitSysCtrl();
    Syncopation_Init();
    Dab_DIS();
    Boost_DIS();
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;  // Enable the PIE block
    IER = M_INT1 | M_INT3 | M_INT8;
    IER |= M_INT13;
    FPGA_RESET;//if(RESET)  OUT_REG_1 <= 16'h0000;

    FPGA_RELEASE;
    DSP_LED_1_OFF;
    DSP_LED_2_OFF;

    SCI_Config();
    EINT;

//    Dab_EN();

    CpuTimer1Regs.TCR.bit.TIF = 1;
    CpuTimer1Regs.TCR.all = 0x4001;

    DSP_LED_1_OFF;
    DSP_LED_2_OFF;

    //sdram_test();
    //sdram_data = __addr32_read_uint16(0x100003e8);
    //Dab_EN();//enable Dab directly to test GPIO
//while(1)
//{
    MainLoop();
//}
}
//        if ((DAB_Start_Flag == 1)&&(DABScenerio==2))   //THIS FUN NEEDS TO GO TO RAM.
//        {
//            //gpioXX=1
//            if(inRange(EPwm2Regs.TBCTR,800,1000))
//            {
//                EALLOW;
//                EPwm2Regs.TZCLR.bit.OST = 1;
//                EPwm3Regs.TZCLR.bit.OST = 1;
//                EDIS;
//
//            //GPIOXX=0
//            }
//    }





//#define TEST_SIZE 5000000
//Uint16 *sd_ram_start = (Uint16 *)0x80000000;//how to define this address? Column+Row = 22? 0x80000000-0x9000000 are safe
//Uint32 wr_index = 0;
//Uint32 rd_index = 0;
//Uint32 error_count = 0;

//void sdram_test()
//{
//    Uint16 data_wr = 0;
//    for(wr_index=0;wr_index<TEST_SIZE;wr_index++)
//    {
//        *(sd_ram_start + wr_index) = data_wr;
//        data_wr += 3;
//    }
//
//    data_wr = 0;
//
//    Uint16 data_rd = 0;
//
//    for(rd_index=0;rd_index<TEST_SIZE;rd_index++)
//    {
//        data_rd = *(sd_ram_start + rd_index);
//        if(data_rd != data_wr)
//            error_count++;
//        data_wr += 3;
//    }
//
//    sdram_data = *(sd_ram_start+0x3e8);
//
//}


#pragma CODE_SECTION(CpuTimerIsr, ".TI.ramfunc");
__interrupt void CpuTimerIsr()
{
    DSP_Timer_TGL;

    IER = M_INT1;
    DSP_LED_1_TGL;
    read_rtd1 = __addr32_read_uint16(fpga_rtd1_addr);
    read_rtd2 = __addr32_read_uint16(fpga_rtd2_addr);
    Uint16 temperature_1 = read_rtd1*5.42/2.5-244; //2.5=%*12.5M/50K; 12.5M = 200M/16
    Uint16 temperature_2 = read_rtd2*5.42/2.5-244;
    //read_function=__addr32_read_uint16(fpga_rtd2_addr);

    SCI_UpdatePacketFloat(0, (float)iac_reading);
    SCI_UpdatePacketInt16(0, dab_prd);
    SCI_UpdatePacketInt16(1, dab_phs);
    SCI_UpdatePacketInt16(2, temperature_1);
    SCI_UpdatePacketInt16(3, temperature_2);

    SCI_SendPacket();
    CpuTimer1Regs.TCR.bit.TIF = 1;

    DSP_Timer_TGL;
    //PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#pragma CODE_SECTION(MainLoop, ".TI.ramfunc");
void MainLoop()
{
    for(;;)
    {
        DSP_MainLoop_TGL;
        if(DAB_Supevisor==2)
        {
            if ((DAB_Start_Flag == 1))
            {
                Uint16 start_point = (int) (dab_prd*0.25);
                if(inRange(start_point-50,start_point+150,EPwm2Regs.TBCTR))
                {
                    EALLOW;
                    EPwm2Regs.TZCLR.bit.OST = 1;
                    EPwm3Regs.TZCLR.bit.OST = 1;
                    EDIS;
                    DAB_Start_Flag = 0;
                }
            }
            if ((DAB_Trip_Flag == 1))
            {
                Uint16 trip_point = (int) (dab_prd*0.25);
                if(inRange(trip_point-50,trip_point+150,EPwm2Regs.TBCTR))
                {
                    EALLOW;
                    EPwm2Regs.TZFRC.bit.OST = 1;
                    EPwm3Regs.TZFRC.bit.OST = 1;
                    EDIS;
                    DAB_Trip_Flag = 0;
                }
            }
        }
        DSP_MainLoop_TGL;
    }
}

void Relay_mainClose()
{
    *fpga_relay |= 1;
}

void Relay_mainOpen()
{
    *fpga_relay &= (~1);
}

