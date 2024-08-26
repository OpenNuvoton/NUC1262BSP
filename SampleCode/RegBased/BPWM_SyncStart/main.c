/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:27p $
 * @brief    Demonstrate how to use BPWM counter synchronous start function.
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
    uint32_t u32TimeOutCnt;



    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

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

    /* Enable BPWM0, BPWM1, BPWM2 and BPWM3 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_BPWM0CKEN_Msk | CLK_APBCLK0_BPWM1CKEN_Msk | CLK_APBCLK0_BPWM2CKEN_Msk | CLK_APBCLK0_BPWM3CKEN_Msk;

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
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC >> 1, 115200);
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
    printf("  This sample code will output waveform with BPWM0, BPWM1, BPWM2 and BPWM3 channel 0~5 at the same time.\n");
    printf("  I/O configuration:\n");
    printf("\n  waveform output pin:\n");
    printf("  BPWM0_CH0(PA.0), BPWM0_CH1(PA.1), BPWM0_CH2(PA.2), BPWM0_CH3(PA.3), BPWM0_CH4(PF.5),BPWM0_CH5(PA.5)\n");
    printf("  BPWM1_CH0(PF.15),BPWM1_CH1(PF.2), BPWM1_CH2(PA.7), BPWM1_CH3(PA.6), BPWM1_CH4(PB.7),BPWM1_CH5(PB.6)\n");
    printf("  BPWM2_CH0(PB.5), BPWM2_CH1(PB.4), BPWM2_CH2(PB.3), BPWM2_CH3(PB.2), BPWM2_CH4(PB.1),BPWM2_CH5(PB.0)\n");
    printf("  BPWM3_CH0(PB.15),BPWM3_CH1(PB.14),BPWM3_CH2(PC.3), BPWM3_CH3(PC.2), BPWM3_CH4(PC.1),BPWM3_CH5(PC.0)\n");

    /* BPWM0, BPWM1, BPWM2 and BPWM3 channel 0~5 frequency and duty configuration are as follows */

    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 72 MHz / (2 * (35999 + 1)) = 1000 Hz
      Duty ratio = (18000) / (35999 + 1) = 50%
    */

    /* Set BPWM0 to up counter type(edge aligned) */
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM0 channel 0~5 frequency and duty configuration */
    /* Set BPWM0 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 1); /* Divided by 2 */
    /* Because of BPWM0 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM0 Timer period */
    BPWM_SET_CNR(BPWM0, 0, 35999);
    /* Set BPWM0 Timer duty */
    BPWM_SET_CMR(BPWM0, 0, 18000);
    BPWM_SET_CMR(BPWM0, 1, 18000);
    BPWM_SET_CMR(BPWM0, 2, 18000);
    BPWM_SET_CMR(BPWM0, 3, 18000);
    BPWM_SET_CMR(BPWM0, 4, 18000);
    BPWM_SET_CMR(BPWM0, 5, 18000);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM0 channel 0 ~ 5 */
    BPWM0->POEN |= 0x3F;


    /* Set BPWM1 to up counter type(edge aligned) */
    BPWM1->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM1 channel 0~5 frequency and duty configuration */
    /* Set BPWM1 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM1, 0, 1); /* Divided by 2 */
    /* Because of BPWM1 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM1 Timer period */
    BPWM_SET_CNR(BPWM1, 0, 35999);
    /* Set BPWM1 Timer duty */
    BPWM_SET_CMR(BPWM1, 0, 18000);
    BPWM_SET_CMR(BPWM1, 1, 18000);
    BPWM_SET_CMR(BPWM1, 2, 18000);
    BPWM_SET_CMR(BPWM1, 3, 18000);
    BPWM_SET_CMR(BPWM1, 4, 18000);
    BPWM_SET_CMR(BPWM1, 5, 18000);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM1, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM1 channel 0 ~ 5 */
    BPWM1->POEN |= 0x3F;


    /* Set BPWM2 to up counter type(edge aligned) */
    BPWM2->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM2 channel 0~5 frequency and duty configuration */
    /* Set BPWM2 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM2, 0, 1); /* Divided by 2 */
    /* Because of BPWM2 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM2 Timer period */
    BPWM_SET_CNR(BPWM2, 0, 35999);
    /* Set BPWM2 Timer duty */
    BPWM_SET_CMR(BPWM2, 0, 18000);
    BPWM_SET_CMR(BPWM2, 1, 18000);
    BPWM_SET_CMR(BPWM2, 2, 18000);
    BPWM_SET_CMR(BPWM2, 3, 18000);
    BPWM_SET_CMR(BPWM2, 4, 18000);
    BPWM_SET_CMR(BPWM2, 5, 18000);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM2, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM2 channel 0 ~ 5 */
    BPWM2->POEN |= 0x3F;


    /* Set BPWM3 to up counter type(edge aligned) */
    BPWM3->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM3 channel 0~5 frequency and duty configuration */
    /* Set BPWM3 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM3, 0, 1); /* Divided by 2 */
    /* Because of BPWM3 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM3 Timer period */
    BPWM_SET_CNR(BPWM3, 0, 35999);
    /* Set BPWM3 Timer duty */
    BPWM_SET_CMR(BPWM3, 0, 18000);
    BPWM_SET_CMR(BPWM3, 1, 18000);
    BPWM_SET_CMR(BPWM3, 2, 18000);
    BPWM_SET_CMR(BPWM3, 3, 18000);
    BPWM_SET_CMR(BPWM3, 4, 18000);
    BPWM_SET_CMR(BPWM3, 5, 18000);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM3, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM3 channel 0 ~ 5 */
    BPWM3->POEN |= 0x3F;

    /* Enable counter synchronous start function for BPWM0, BPWM1, BPWM2 and BPWM3 channel 0~5 */
    BPWM0->SSCTL = BPWM_SSCTL_SSRC_BPWM0 | BPWM_SSCTL_SSEN0_Msk;
    BPWM1->SSCTL = BPWM_SSCTL_SSRC_BPWM0 | BPWM_SSCTL_SSEN0_Msk;
    BPWM2->SSCTL = BPWM_SSCTL_SSRC_BPWM0 | BPWM_SSCTL_SSEN0_Msk;
    BPWM3->SSCTL = BPWM_SSCTL_SSRC_BPWM0 | BPWM_SSCTL_SSEN0_Msk;

    printf("Press any key to start.\n");
    getchar();

    /* Trigger BPWM counter synchronous start by BPWM0 */
    BPWM0->SSTRG = BPWM_SSTRG_CNTSEN_Msk;

    printf("Done.");
    while(1);

}
