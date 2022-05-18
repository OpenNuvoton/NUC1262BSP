/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:27p $
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
int32_t CalPeriodTime()
{
    uint16_t au16Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk;

    /* Wait for Capture Falling Indicator  */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((BPWM0->CAPIF & BPWM_CAPIF_CAPFIF0_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM Capture Falling Indicator time-out!\n");
            return -1;
        }
    }

    /* Clear Capture Falling Indicator (Time B)*/
    BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk;

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((BPWM0->CAPIF & BPWM_CAPIF_CAPFIF0_Msk) == 0)
        while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Falling Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk | BPWM_CAPIF_CAPRIF0_Msk;

        /* Get Capture Falling Latch Counter Data */
        au16Count[u32i++] = BPWM_GET_CAPTURE_FALLING_DATA(BPWM0, 0);

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((BPWM0->CAPIF & BPWM_CAPIF_CAPRIF0_Msk) == 0)
        while(BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 1)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Rising Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Rising Indicator */
        BPWM0->CAPIF = BPWM_CAPIF_CAPRIF0_Msk;

        /* Get Capture Rising Latch Counter Data */
        au16Count[u32i++] = BPWM_GET_CAPTURE_RISING_DATA(BPWM0, 0);
    }

    u16RisingTime = au16Count[1];

    u16FallingTime = au16Count[0];

    u16HighPeriod = au16Count[1] - au16Count[2];

    u16LowPeriod = 0x10000 - au16Count[1];

    u16TotalPeriod = 0x10000 - au16Count[2];

    printf("\nBPWM generate: \nHigh Period=7199 ~ 7201, Low Period=16799 ~ 16801, Total Period=23999 ~ 24001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 7199) || (u16HighPeriod > 7201) || (u16LowPeriod < 16799) || (u16LowPeriod > 16801) || (u16TotalPeriod < 23999) || (u16TotalPeriod > 24001))
    {
        printf("Capture Test Fail!!\n");
        return -1;
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
    }
}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    /* Select HCLK clock source as PLL/2 and HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL_DIV2;

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC_DIV2;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);

    /* Enable BPWM0 and BPWM1 module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_BPWM0CKEN_Msk | CLK_APBCLK0_BPWM1CKEN_Msk);

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
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC>>1, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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

    uint32_t u32TimeOutCnt;

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
        /* Set the BPWM1 Channel 0 as BPWM output function.                                     */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 1500Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           up counter type:
           duty ratio = (CMR)/(CNR+1)
           cycle time = CNR+1
           High level = CMR
           PWM clock source frequency = PLL = 72000000
           (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                   = 72000000/2/1500 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 23999
           duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
           CMR = 7200
           Prescale value is 1 : prescaler= 2
        */

        /*Set counter as up count*/
        BPWM1->CTL1 = (BPWM1->CTL1 & ~BPWM_CTL1_CNTTYPE0_Msk);

        /*Set BPWM Timer clock prescaler*/
        BPWM_SET_PRESCALER(BPWM1, 0, 1); // Divided by 2

        /*Set BPWM Timer duty*/
        BPWM_SET_CMR(BPWM1, 0, 7200);

        /*Set BPWM Timer period*/
        BPWM_SET_CNR(BPWM1, 0, 23999);

        /* Set waveform generation */
        BPWM1->WGCTL0 = 0x00000002;
        BPWM1->WGCTL1 = 0x00000001;

        /* Enable BPWM Output path for BPWM1 channel 0 */
        BPWM1->POEN |= BPWM_CH_0_MASK;

        /* Enable Timer for BPWM1 channel 0 */
        BPWM1->CNTEN |= BPWM_CH_0_MASK;

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                         */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 1500Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PLL = 72000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 72000000/2/1500 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */

        /*Set counter as down count*/
        BPWM0->CTL1 = (BPWM0->CTL1 & ~BPWM_CTL1_CNTTYPE0_Msk) | (0x1 << BPWM_CTL1_CNTTYPE0_Pos);

        /*Set BPWM0 channel 0 Timer clock prescaler*/
        BPWM_SET_PRESCALER(BPWM0, 0, 1); // Divided by 2

        /*Set BPWM0 channel 0 Timer period*/
        BPWM_SET_CNR(BPWM0, 0, 0xFFFF);

        /* Enable capture function */
        BPWM0->CAPCTL |= BPWM_CAPCTL_CAPEN0_Msk;

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        /* Start */
        BPWM0->CNTEN |= BPWM_CNTEN_CNTEN0_Msk;

        /* Wait until BPWM0 channel 0 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((BPWM0->CNT) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 Timer start to count time-out!\n");
                return -1;
            }
        }

        /* Enable capture input path for BPWM0 channel 0 */
        BPWM0->CAPINEN |= BPWM_CAPINEN_CAPINEN0_Msk;

        /* Capture the Input Waveform Data */
        if( CalPeriodTime() < 0)
            return -1;

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM1 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set BPWM1 channel 0 loaded value as 0 */
        BPWM1->PERIOD = 0;

        /* Wait until BPWM1 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((BPWM1->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 Timer Stop time-out!\n");
                return -1;
            }
        }

        /* Disable Timer for BPWM1 channel 0 */
        BPWM1->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;

        /* Disable BPWM Output path for BPWM1 channel 0 */
        BPWM1->POEN &= ~BPWM_CH_0_MASK;

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM0->PERIOD = 0;

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock;  /* 1 second time-out */
        while((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM channel 0 current counter reach to 0 time-out!\n");
                return -1;
            }
        }

        /* Disable Timer for BPWM0 channel 0 */
        BPWM0->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;

        /* Disable Capture Function and Capture Input path for BPWM0 channel 0 */
        BPWM0->CAPCTL &= ~BPWM_CAPCTL_CAPEN0_Msk;
        BPWM0->CAPINEN &= ~BPWM_CAPINEN_CAPINEN0_Msk;

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM0->CAPIF = BPWM_CAPIF_CAPFIF0_Msk;

    }
}
