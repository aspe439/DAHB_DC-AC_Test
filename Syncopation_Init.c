/*
 * Syncopation_Init.c
 *
 *  Created on: April, 2019
 *      Author: Ao Sun
 */


#include "F28x_Project.h"

//#define GPAMUX1_MASK        0xffffffff
//#define GPAMUX2_MASK        0xf0ffffff
//#define GPAMUX1_VALUE       0x00000000
//#define GPAMUX2_VALUE       0xa0550000
//#define GPAGMUX1_VALUE      0x00000000
//#define GPAGMUX2_VALUE      0x00000000

//#define GPAMUX1_MASK        0xffffffff
//#define GPAMUX2_MASK        0xf00fffff
//#define GPAMUX1_VALUE       0x00000000
//#define GPAMUX2_VALUE       0xa0000000
//#define GPAGMUX1_VALUE      0x00000000
//#define GPAGMUX2_VALUE      0x00000000

#define GPAMUX1_MASK        0xffffffff
#define GPAMUX2_MASK        0xf0cfffff
#define GPAMUX1_VALUE       0x00000000//0x00000000 or 0x00555555
#define GPAMUX2_VALUE       0xa0000000
#define GPAGMUX1_VALUE      0x00000000
#define GPAGMUX2_VALUE      0x00000000

#define GPBMUX1_MASK        0xffffff3f
#define GPBMUX2_MASK        0xcffff3ff
#define GPBMUX1_VALUE       0xaaaaaa2a
#define GPBMUX2_VALUE       0xcffaa2aa
#define GPBGMUX1_VALUE      0x00500000
#define GPBGMUX2_VALUE      0xcff55000
#define GPCMUX1_MASK        0xfffffc3f
#define GPCMUX2_MASK        0x0f0ffcff
#define GPCMUX1_VALUE       0xaaaaa83f
#define GPCMUX2_VALUE       0x0f0ff8aa
#define GPCGMUX1_VALUE      0x0000003f
#define GPCGMUX2_VALUE      0x00000000
#define GPDMUX1_MASK        0x3cffffff
#define GPDMUX2_MASK        0xfff0cc30
#define GPDMUX1_VALUE       0x00555555
#define GPDMUX2_VALUE       0xfff00c00
#define GPDGMUX1_VALUE      0x00555555
#define GPDGMUX2_VALUE      0x55500000
#define GPEMUX1_MASK        0x003fffff
#define GPEMUX2_MASK        0xfffffffc
#define GPEMUX1_VALUE       0x000fffff
#define GPEMUX2_VALUE       0x55555554//0x55555554
#define GPEGMUX1_VALUE      0x00055555
#define GPEGMUX2_VALUE      0x00000000
#define GPFMUX1_MASK        0x00000003
#define GPFMUX1_VALUE       0x00000001
#define GPFGMUX1_VALUE      0x00000000
#define GPBAMSEL_MASK       0x00000c00
#define GPBAMSEL_VALUE      0x00000000

void GPIO_setPinMuxConfig(void)
{
    Uint32 lockValA;
    Uint32 lockValB;
    Uint32 lockValC;
    Uint32 lockValD;
    Uint32 lockValE;
    Uint32 lockValF;

    EALLOW;

    //
    // Save the current value of the GPIO lock registers
    //
    lockValA = GpioCtrlRegs.GPALOCK.all;
    lockValB = GpioCtrlRegs.GPBLOCK.all;
    lockValC = GpioCtrlRegs.GPCLOCK.all;
    lockValD = GpioCtrlRegs.GPDLOCK.all;
    lockValE = GpioCtrlRegs.GPELOCK.all;
    lockValF = GpioCtrlRegs.GPFLOCK.all;

    //
    // Unlock the GPIO control registers
    //
    GpioCtrlRegs.GPALOCK.all = 0x00000000;
    GpioCtrlRegs.GPBLOCK.all = 0x00000000;
    GpioCtrlRegs.GPCLOCK.all = 0x00000000;
    GpioCtrlRegs.GPDLOCK.all = 0x00000000;
    GpioCtrlRegs.GPELOCK.all = 0x00000000;
    GpioCtrlRegs.GPFLOCK.all = 0x00000000;

    //
    // Clear the mux register fields that are about to be changed
    //
    GpioCtrlRegs.GPAMUX1.all    &= ~GPAMUX1_MASK;
    GpioCtrlRegs.GPAMUX2.all    &= ~GPAMUX2_MASK;
    GpioCtrlRegs.GPAGMUX1.all   &= ~GPAMUX1_MASK;
    GpioCtrlRegs.GPAGMUX2.all   &= ~GPAMUX2_MASK;
    GpioCtrlRegs.GPBMUX1.all    &= ~GPBMUX1_MASK;
    GpioCtrlRegs.GPBMUX2.all    &= ~GPBMUX2_MASK;
    GpioCtrlRegs.GPBGMUX1.all   &= ~GPBMUX1_MASK;
    GpioCtrlRegs.GPBGMUX2.all   &= ~GPBMUX2_MASK;
    GpioCtrlRegs.GPCMUX1.all    &= ~GPCMUX1_MASK;
    GpioCtrlRegs.GPCMUX2.all    &= ~GPCMUX2_MASK;
    GpioCtrlRegs.GPCGMUX1.all   &= ~GPCMUX1_MASK;
    GpioCtrlRegs.GPCGMUX2.all   &= ~GPCMUX2_MASK;
    GpioCtrlRegs.GPDMUX1.all    &= ~GPDMUX1_MASK;
    GpioCtrlRegs.GPDMUX2.all    &= ~GPDMUX2_MASK;
    GpioCtrlRegs.GPDGMUX1.all   &= ~GPDMUX1_MASK;
    GpioCtrlRegs.GPDGMUX2.all   &= ~GPDMUX2_MASK;
    GpioCtrlRegs.GPEMUX1.all    &= ~GPEMUX1_MASK;
    GpioCtrlRegs.GPEMUX2.all    &= ~GPEMUX2_MASK;
    GpioCtrlRegs.GPEGMUX1.all   &= ~GPEMUX1_MASK;
    GpioCtrlRegs.GPEGMUX2.all   &= ~GPEMUX2_MASK;
    GpioCtrlRegs.GPFMUX1.all    &= ~GPFMUX1_MASK;
    GpioCtrlRegs.GPFGMUX1.all   &= ~GPFMUX1_MASK;

    //
    // Write pin muxing to mux registers
    //
    GpioCtrlRegs.GPAGMUX1.all   |=  GPAGMUX1_VALUE;
    GpioCtrlRegs.GPAGMUX2.all   |=  GPAGMUX2_VALUE;
    GpioCtrlRegs.GPAMUX1.all    |=  GPAMUX1_VALUE;
    GpioCtrlRegs.GPAMUX2.all    |=  GPAMUX2_VALUE;
    GpioCtrlRegs.GPBGMUX1.all   |=  GPBGMUX1_VALUE;
    GpioCtrlRegs.GPBGMUX2.all   |=  GPBGMUX2_VALUE;
    GpioCtrlRegs.GPBMUX1.all    |=  GPBMUX1_VALUE;
    GpioCtrlRegs.GPBMUX2.all    |=  GPBMUX2_VALUE;
    GpioCtrlRegs.GPCGMUX1.all   |=  GPCGMUX1_VALUE;
    GpioCtrlRegs.GPCGMUX2.all   |=  GPCGMUX2_VALUE;
    GpioCtrlRegs.GPCMUX1.all    |=  GPCMUX1_VALUE;
    GpioCtrlRegs.GPCMUX2.all    |=  GPCMUX2_VALUE;
    GpioCtrlRegs.GPDGMUX1.all   |=  GPDGMUX1_VALUE;
    GpioCtrlRegs.GPDGMUX2.all   |=  GPDGMUX2_VALUE;
    GpioCtrlRegs.GPDMUX1.all    |=  GPDMUX1_VALUE;
    GpioCtrlRegs.GPDMUX2.all    |=  GPDMUX2_VALUE;
    GpioCtrlRegs.GPEGMUX1.all   |=  GPEGMUX1_VALUE;
    GpioCtrlRegs.GPEGMUX2.all   |=  GPEGMUX2_VALUE;
    GpioCtrlRegs.GPEMUX1.all    |=  GPEMUX1_VALUE;
    GpioCtrlRegs.GPEMUX2.all    |=  GPEMUX2_VALUE;
    GpioCtrlRegs.GPFGMUX1.all   |=  GPFGMUX1_VALUE;
    GpioCtrlRegs.GPFMUX1.all    |=  GPFMUX1_VALUE;

    //
    // Write pin analog mode select to registers
    //
    GpioCtrlRegs.GPBAMSEL.all   &= ~GPBAMSEL_MASK;
    GpioCtrlRegs.GPBAMSEL.all   |= GPBAMSEL_VALUE;

    //
    // Restore GPIO lock register values
    //
    GpioCtrlRegs.GPALOCK.all = lockValA;
    GpioCtrlRegs.GPBLOCK.all = lockValB;
    GpioCtrlRegs.GPCLOCK.all = lockValC;
    GpioCtrlRegs.GPDLOCK.all = lockValD;
    GpioCtrlRegs.GPELOCK.all = lockValE;
    GpioCtrlRegs.GPFLOCK.all = lockValF;

    // User's configuration
    EALLOW;
    // Enable FPGA_DONE pull-up
    GpioCtrlRegs.GPDPUD.bit.GPIO114 = 0;
    // Set user LED as output
    GpioCtrlRegs.GPDDIR.bit.GPIO109 = 1;
    GpioCtrlRegs.GPDDIR.bit.GPIO110 = 1;
    // Set FPGA RESET pin as output
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;

    /*GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;
*/
    EDIS;
}

void InterruptInit(void)
{
    DINT;
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;

    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER2.all = 0;
    PieCtrlRegs.PIEIER3.all = 0;
    PieCtrlRegs.PIEIER4.all = 0;
    PieCtrlRegs.PIEIER5.all = 0;
    PieCtrlRegs.PIEIER6.all = 0;
    PieCtrlRegs.PIEIER7.all = 0;
    PieCtrlRegs.PIEIER8.all = 0;
    PieCtrlRegs.PIEIER9.all = 0;
    PieCtrlRegs.PIEIER10.all = 0;
    PieCtrlRegs.PIEIER11.all = 0;
    PieCtrlRegs.PIEIER12.all = 0;

    PieCtrlRegs.PIEIFR1.all = 0;
    PieCtrlRegs.PIEIFR2.all = 0;
    PieCtrlRegs.PIEIFR3.all = 0;
    PieCtrlRegs.PIEIFR4.all = 0;
    PieCtrlRegs.PIEIFR5.all = 0;
    PieCtrlRegs.PIEIFR6.all = 0;
    PieCtrlRegs.PIEIFR7.all = 0;
    PieCtrlRegs.PIEIFR8.all = 0;
    PieCtrlRegs.PIEIFR9.all = 0;
    PieCtrlRegs.PIEIFR10.all = 0;
    PieCtrlRegs.PIEIFR11.all = 0;
    PieCtrlRegs.PIEIFR12.all = 0;

    PieCtrlRegs.PIEACK.all = 0xFFFF;
}

extern void setup_emif1_pinmux_sdram_16bit(Uint16 cpu_sel);
void emifInit()
{
    setup_emif1_pinmux_sdram_16bit(0);
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 0x1;
    Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;
    Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;
    Emif1ConfigRegs.EMIF1LOCK.all = 0x1;
    EDIS;
    Emif1Regs.SDRAM_TR.all = 0x31114610;
    Emif1Regs.SDR_EXT_TMNG.all = 0x7;
    Emif1Regs.SDRAM_RCR.all = 0x30E;
    Emif1Regs.SDRAM_CR.all = 0x00015621;
    DELAY_US(10);
    Emif1Regs.ASYNC_CS2_CR.all = 0x04122195;
    DELAY_US(10);
}

extern void EpwmInit();
extern void AdcInit();
extern void CpuTimerIsr();

#define CPU_INT_USEC 20

void CpuTimerInit()
{
    //CpuTimer1Regs.TCR.all = 0x4010;//0100 0000 0001 0000
    CpuTimer1Regs.TCR.all = 0x4010;
    CpuTimer1Regs.PRD.all = 1000000;//5000us
    EALLOW;
    PieVectTable.TIMER1_INT = &CpuTimerIsr;
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    EDIS;
}

extern void Xint1Isr();
void XintInit()
{
    EALLOW;
    InputXbarRegs.INPUT4SELECT = 27;//route GPIO27 as input4
    PieVectTable.XINT1_INT = &Xint1Isr;
    PieCtrlRegs.PIEIER1.bit.INTx4 = 1; //XINT1
    XintRegs.XINT1CR.bit.ENABLE = 1;
    EDIS;
}

void Syncopation_Init(void)
{
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;
    EDIS;

    GPIO_setPinMuxConfig();

    DINT;
    InterruptInit();
    emifInit();

    EpwmInit();
    AdcInit();
    XintInit();
    CpuTimerInit();
}
