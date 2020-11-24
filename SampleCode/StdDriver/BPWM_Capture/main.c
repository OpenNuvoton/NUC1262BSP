/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:29p $
 * @brief    Use BPWM0 channel 0 to capture the BPWM1 channel 0 waveform.
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


/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
void CalPeriodTime(BPWM_T *BPWM, uint32_t u32Ch)
{
    uint16_t au16Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

    /* Wait for Capture Falling Indicator  */
    while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2);

    /* Clear Capture Falling Indicator (Time B)*/
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2);

        /* Clear Capture Falling and Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        au16Count[u32i++] = (uint16_t)BPWM_GET_CAPTURE_FALLING_DATA(BPWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 1);

        /* Clear Capture Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        au16Count[u32i++] = (uint16_t)BPWM_GET_CAPTURE_RISING_DATA(BPWM, u32Ch);
    }

    u16RisingTime = au16Count[1];

    u16FallingTime = au16Count[0];

    u16HighPeriod = au16Count[1] - au16Count[2];

    u16LowPeriod = 0x10000 - au16Count[1];

    u16TotalPeriod = 0x10000 - au16Count[2];

    printf("\nBPWM generate: \nHigh Period=17279 ~ 17281, Low Period=40319 ~ 40321, Total Period=57599 ~ 57601\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 17279) || (u16HighPeriod > 17281) || (u16LowPeriod < 40319) || (u16LowPeriod > 40321) || (u16TotalPeriod < 57599) || (u16TotalPeriod > 57601))
        printf("Capture Test Fail!!\n");
    else
        printf("Capture Test Pass!!\n");
}

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

    /* Enable BPWM0 and BPWM1 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    CLK_EnableModuleClock(BPWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set multi-function pins for BPWM */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;
    SYS->GPF_MFPH &= ~(SYS_GPF_MFPH_PF15MFP_Msk);
    SYS->GPF_MFPH |= SYS_GPF_MFPH_PF15MFP_BPWM1_CH0;

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
    printf("  This sample code will use BPWM0 channel 0 to capture\n  the signal from BPWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0 channel 0(PA.0) <--> BPWM1 channel 0(PF.15)\n\n");
    printf("Use BPWM0 Channel 0(PA.0) to capture the BPWM1 Channel 0(PF.15) Waveform\n");

    while(1)
    {
        printf("\n\nPress any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM1 Channel 0 as BPWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           PWM clock source frequency = PLL = 72000000
           (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                   = 72000000/5/250 = 57600
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 57599
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 17279
           Prescale value is 4 : prescaler= 5
        */

        /* set BPWM1 channel 0 output configuration */
        BPWM_ConfigOutputChannel(BPWM1, 0, 250, 30);

        /* Enable BPWM Output path for BPWM1 channel 0 */
        BPWM_EnableOutput(BPWM1, BPWM_CH_0_MASK);

        /* Enable Timer for BPWM1 channel 0 */
        BPWM_Start(BPWM1, BPWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL = 72000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 72000000/5/250 = 57600
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1/Capture clock source frequency/prescaler
           69.4ns = 1/72000000/5
        */

        /* set BPWM0 channel 0 capture configuration */
        BPWM_ConfigCaptureChannel(BPWM0, 0, 69, 0);

        /* Enable Timer for PWM0 channel 0 */
        BPWM_Start(BPWM0, BPWM_CH_0_MASK);

        /* Enable Capture Function for BPWM0 channel 0 */
        BPWM_EnableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        /* Wait until BPWM0 channel 0 Timer start to count */
        while((BPWM0->CNT) == 0);

        /* Capture the Input Waveform Data */
        CalPeriodTime(BPWM0, 0);
        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM1 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set BPWM1 channel 0 loaded value as 0 */
        BPWM_Stop(BPWM1, BPWM_CH_0_MASK);

        /* Wait until BPWM1 channel 0 Timer Stop */
        while((BPWM1->CNT & BPWM_CNT_CNT_Msk) != 0);

        /* Disable Timer for BPWM1 channel 0 */
        BPWM_ForceStop(BPWM1, BPWM_CH_0_MASK);

        /* Disable BPWM Output path for BPWM1 channel 0 */
        BPWM_DisableOutput(BPWM1, BPWM_CH_0_MASK);

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM_Stop(BPWM0, BPWM_CH_0_MASK);

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        while((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0);

        /* Disable Timer for BPWM1 channel 0 */
        BPWM_ForceStop(BPWM0, BPWM_CH_0_MASK);

        /* Disable Capture Function and Capture Input path for BPWM0 channel 0 */
        BPWM_DisableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }
}
