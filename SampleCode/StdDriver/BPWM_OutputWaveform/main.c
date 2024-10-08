/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:29p $
 * @brief    Demonstrate how to use BPWM counter output waveform.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /* Enable BPWM0 ,BPWM1, BPWM2 and BPWM3 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    CLK_EnableModuleClock(BPWM1_MODULE);
    CLK_EnableModuleClock(BPWM2_MODULE);
    CLK_EnableModuleClock(BPWM3_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set multi-function pins for BPWM0 */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk |
                       SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk |
                       SYS_GPA_MFPL_PA5MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_BPWM0_CH0 | SYS_GPA_MFPL_PA1MFP_BPWM0_CH1 |
                      SYS_GPA_MFPL_PA2MFP_BPWM0_CH2 | SYS_GPA_MFPL_PA3MFP_BPWM0_CH3 |
                      SYS_GPA_MFPL_PA5MFP_BPWM0_CH5);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF5MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF5MFP_BPWM0_CH4);

	/* Set multi-function pins for BPWM1 */
    SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk);
    SYS->GPF_MFPH |= (SYS_GPF_MFPH_PF15MFP_BPWM1_CH0);
    SYS->GPF_MFPL &= ~(SYS_GPF_MFPL_PF2MFP_Msk);
    SYS->GPF_MFPL |= (SYS_GPF_MFPL_PF2MFP_BPWM1_CH1);
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA7MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA7MFP_BPWM1_CH2 | SYS_GPA_MFPL_PA6MFP_BPWM1_CH3);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_BPWM1_CH4 | SYS_GPB_MFPL_PB6MFP_BPWM1_CH5);

	/* Set multi-function pins for BPWM2 */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB5MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk |
                       SYS_GPB_MFPL_PB3MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk |
                       SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB5MFP_BPWM2_CH0 | SYS_GPB_MFPL_PB4MFP_BPWM2_CH1 |
                      SYS_GPB_MFPL_PB3MFP_BPWM2_CH2 | SYS_GPB_MFPL_PB2MFP_BPWM2_CH3 |
                      SYS_GPB_MFPL_PB1MFP_BPWM2_CH4 | SYS_GPB_MFPL_PB0MFP_BPWM2_CH5);

	/* Set multi-function pins for BPWM3 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB15MFP_Msk | SYS_GPB_MFPH_PB14MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB15MFP_BPWM3_CH0 | SYS_GPB_MFPH_PB14MFP_BPWM3_CH1);
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk |
                       SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC0MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC3MFP_BPWM3_CH2 | SYS_GPC_MFPL_PC2MFP_BPWM3_CH3 |
                      SYS_GPC_MFPL_PC1MFP_BPWM3_CH4 | SYS_GPC_MFPL_PC0MFP_BPWM3_CH5);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0, BPWM1, BPWM2 and BPWM3 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  BPWM0 channel 0: 180000 Hz, duty 90%%.\n");
    printf("  BPWM0 channel 1: 180000 Hz, duty 80%%.\n");
    printf("  BPWM0 channel 2: 180000 Hz, duty 75%%.\n");
    printf("  BPWM0 channel 3: 180000 Hz, duty 70%%.\n");
    printf("  BPWM0 channel 4: 180000 Hz, duty 60%%.\n");
    printf("  BPWM0 channel 5: 180000 Hz, duty 50%%.\n");
    printf("  BPWM1 channel 0:  60000 Hz, duty 50%%.\n");
    printf("  BPWM1 channel 1:  60000 Hz, duty 40%%.\n");
    printf("  BPWM1 channel 2:  60000 Hz, duty 30%%.\n");
    printf("  BPWM1 channel 3:  60000 Hz, duty 25%%.\n");
    printf("  BPWM1 channel 4:  60000 Hz, duty 20%%.\n");
    printf("  BPWM1 channel 5:  60000 Hz, duty 10%%.\n");
    printf("  BPWM2 channel 0: 360000 Hz, duty 90%%.\n");
    printf("  BPWM2 channel 1: 360000 Hz, duty 80%%.\n");
    printf("  BPWM2 channel 2: 360000 Hz, duty 75%%.\n");
    printf("  BPWM2 channel 3: 360000 Hz, duty 70%%.\n");
    printf("  BPWM2 channel 4: 360000 Hz, duty 60%%.\n");
    printf("  BPWM2 channel 5: 360000 Hz, duty 50%%.\n");
    printf("  BPWM3 channel 0: 120000 Hz, duty 50%%.\n");
    printf("  BPWM3 channel 1: 120000 Hz, duty 40%%.\n");
    printf("  BPWM3 channel 2: 120000 Hz, duty 30%%.\n");
    printf("  BPWM3 channel 3: 120000 Hz, duty 25%%.\n");
    printf("  BPWM3 channel 4: 120000 Hz, duty 20%%.\n");
    printf("  BPWM3 channel 5: 120000 Hz, duty 10%%.\n");
    printf("\n  waveform output pin:\n");
    printf("  BPWM0_CH0(PA.0), BPWM0_CH1(PA.1), BPWM0_CH2(PA.2), BPWM0_CH3(PA.3), BPWM0_CH4(PF.5),BPWM0_CH5(PA.5)\n");
    printf("  BPWM1_CH0(PF.15),BPWM1_CH1(PF.2), BPWM1_CH2(PA.7), BPWM1_CH3(PA.6), BPWM1_CH4(PB.7),BPWM1_CH5(PB.6)\n");
    printf("  BPWM2_CH0(PB.5), BPWM2_CH1(PB.4), BPWM2_CH2(PB.3), BPWM2_CH3(PB.2), BPWM2_CH4(PB.1),BPWM2_CH5(PB.0)\n");
    printf("  BPWM3_CH0(PB.15),BPWM3_CH1(PB.14),BPWM3_CH2(PC.3), BPWM3_CH3(PC.2), BPWM3_CH4(PC.1),BPWM3_CH5(PC.0)\n");


    /* BPWM0, BPWM1, BPWM2 and BPWM3 channel 0~5 frequency and duty configuration are as follows */
    /* Because of BPWM0 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM0, 0, 180000, 90);
    BPWM_ConfigOutputChannel(BPWM0, 1, 180000, 80);
    BPWM_ConfigOutputChannel(BPWM0, 2, 180000, 75);
    BPWM_ConfigOutputChannel(BPWM0, 3, 180000, 70);
    BPWM_ConfigOutputChannel(BPWM0, 4, 180000, 60);
    BPWM_ConfigOutputChannel(BPWM0, 5, 180000, 50);
    /* Because of BPWM1 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM1, 0, 60000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 1, 60000, 40);
    BPWM_ConfigOutputChannel(BPWM1, 2, 60000, 30);
    BPWM_ConfigOutputChannel(BPWM1, 3, 60000, 25);
    BPWM_ConfigOutputChannel(BPWM1, 4, 60000, 20);
    BPWM_ConfigOutputChannel(BPWM1, 5, 60000, 10);
    /* Because of BPWM2 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM2, 0, 360000, 90);
    BPWM_ConfigOutputChannel(BPWM2, 1, 360000, 80);
    BPWM_ConfigOutputChannel(BPWM2, 2, 360000, 75);
    BPWM_ConfigOutputChannel(BPWM2, 3, 360000, 70);
    BPWM_ConfigOutputChannel(BPWM2, 4, 360000, 60);
    BPWM_ConfigOutputChannel(BPWM2, 5, 360000, 50);
    /* Because of BPWM3 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM3, 0, 120000, 50);
    BPWM_ConfigOutputChannel(BPWM3, 1, 120000, 40);
    BPWM_ConfigOutputChannel(BPWM3, 2, 120000, 30);
    BPWM_ConfigOutputChannel(BPWM3, 3, 120000, 25);
    BPWM_ConfigOutputChannel(BPWM3, 4, 120000, 20);
    BPWM_ConfigOutputChannel(BPWM3, 5, 120000, 10);

    /* Enable output of BPWM0, BPWM1, BPWM2 and BPWM3 channel 0~5 */
    BPWM_EnableOutput(BPWM0, 0x3F);
    BPWM_EnableOutput(BPWM1, 0x3F);
    BPWM_EnableOutput(BPWM2, 0x3F);
    BPWM_EnableOutput(BPWM3, 0x3F);

    /* Start BPWM0 counter */
    BPWM_Start(BPWM0, 0x3F);
    /* Start BPWM1 counter */
    BPWM_Start(BPWM1, 0x3F);
    /* Start BPWM2 counter */
    BPWM_Start(BPWM2, 0x3F);
    /* Start BPWM3 counter */
    BPWM_Start(BPWM3, 0x3F);

    printf("\nPress any key to stop.\n");
    getchar();

    /* Stop BPWM0 counter */
    BPWM_ForceStop(BPWM0, 0x3F);
    /* Stop BPWM1 counter */
    BPWM_ForceStop(BPWM1, 0x3F);
    /* Stop BPWM2 counter */
    BPWM_ForceStop(BPWM2, 0x3F);
    /* Stop BPWM3 counter */
    BPWM_ForceStop(BPWM3, 0x3F);

    printf("Done.");
    while(1);

}
