/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:27p $
 * @brief    Demonstrate how to use BPWM output waveform.
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

/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{

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

    /* Enable BPWM0, BPWM1, BPWM2 and BPWM3 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_BPWM0CKEN_Msk|CLK_APBCLK0_BPWM1CKEN_Msk|CLK_APBCLK0_BPWM2CKEN_Msk|CLK_APBCLK0_BPWM3CKEN_Msk;

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

    /*
      Configure BPWM0 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 72 MHz / (1 * (399 + 1)) = 180000 Hz
      Duty ratio = (360) / (400) = 90%
      Duty ratio = (320) / (400) = 80%
      Duty ratio = (300) / (400) = 75%
      Duty ratio = (280) / (400) = 70%
      Duty ratio = (240) / (400) = 60%
      Duty ratio = (200) / (400) = 50%
    */
    /* Set BPWM0 to up counter type(edge aligned) */
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM0 channel 0~5 frequency and duty configuration */
    /* Set BPWM0 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 0); /* Divided by 1 */
    /* Because of BPWM0 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM0 Timer period */
    BPWM_SET_CNR(BPWM0, 0, 399);
    /* Set BPWM0 Timer duty */
    BPWM_SET_CMR(BPWM0, 0, 360);
    BPWM_SET_CMR(BPWM0, 1, 320);
    BPWM_SET_CMR(BPWM0, 2, 300);
    BPWM_SET_CMR(BPWM0, 3, 280);
    BPWM_SET_CMR(BPWM0, 4, 240);
    BPWM_SET_CMR(BPWM0, 5, 200);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM0 channel 0 ~ 5 */
    BPWM0->POEN |= 0x3F;

    /*
      Configure BPWM1 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 72 MHz / (1 * (1199 + 1)) = 60000 Hz
      Duty ratio = (600) / (1200) = 50%
      Duty ratio = (480) / (1200) = 40%
      Duty ratio = (360) / (1200) = 30%
      Duty ratio = (300) / (1200) = 25%
      Duty ratio = (240) / (1200) = 20%
      Duty ratio = (120) / (1200) = 10%
    */
    /* Set BPWM1 to up counter type(edge aligned) */
    BPWM1->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM1 channel 0~5 frequency and duty configuration */
    /* Set BPWM1 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM1, 0, 0); /* Divided by 1 */
    /* Because of BPWM1 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM1 Timer period */
    BPWM_SET_CNR(BPWM1, 0, 1199);
    /* Set BPWM1 Timer duty */
    BPWM_SET_CMR(BPWM1, 0, 600);
    BPWM_SET_CMR(BPWM1, 1, 480);
    BPWM_SET_CMR(BPWM1, 2, 360);
    BPWM_SET_CMR(BPWM1, 3, 300);
    BPWM_SET_CMR(BPWM1, 4, 240);
    BPWM_SET_CMR(BPWM1, 5, 120);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM1, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM1 channel 0 ~ 5 */
    BPWM1->POEN |= 0x3F;


    /*
      Configure BPWM2 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 72 MHz / (1 * (199 + 1)) = 360000 Hz
      Duty ratio = (180) / (200) = 90%
      Duty ratio = (160) / (200) = 80%
      Duty ratio = (150) / (200) = 75%
      Duty ratio = (140) / (200) = 70%
      Duty ratio = (120) / (200) = 60%
      Duty ratio = (100) / (200) = 50%
    */
    /* Set BPWM2 to up counter type(edge aligned) */
    BPWM2->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM2 channel 0~5 frequency and duty configuration */
    /* Set BPWM2 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM2, 0, 0); /* Divided by 1 */
    /* Because of BPWM2 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM2 Timer period */
    BPWM_SET_CNR(BPWM2, 0, 199);
    /* Set BPWM2 Timer duty */
    BPWM_SET_CMR(BPWM2, 0, 180);
    BPWM_SET_CMR(BPWM2, 1, 160);
    BPWM_SET_CMR(BPWM2, 2, 150);
    BPWM_SET_CMR(BPWM2, 3, 140);
    BPWM_SET_CMR(BPWM2, 4, 120);
    BPWM_SET_CMR(BPWM2, 5, 100);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM2, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM2 channel 0 ~ 5 */
    BPWM2->POEN |= 0x3F;


    /*
      Configure BPWM3 channel 0 init period and duty(up counter type).
      Period is PLL / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 72 MHz / (1 * (599 + 1)) = 120000 Hz
      Duty ratio = (300) / (600) = 50%
      Duty ratio = (240) / (600) = 40%
      Duty ratio = (180) / (600) = 30%
      Duty ratio = (150) / (600) = 25%
      Duty ratio = (120) / (600) = 20%
      Duty ratio =  (60) / (600) = 10%
    */
    /* Set BPWM3 to up counter type(edge aligned) */
    BPWM3->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    /* BPWM3 channel 0~5 frequency and duty configuration */
    /* Set BPWM3 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM3, 0, 0); /* Divided by 1 */
    /* Because of BPWM3 channel 0~5 share one period, so the period value of all channels need set the same. */
    /* Set BPWM3 Timer period */
    BPWM_SET_CNR(BPWM3, 0, 599);
    /* Set BPWM1 Timer duty */
    BPWM_SET_CMR(BPWM3, 0, 300);
    BPWM_SET_CMR(BPWM3, 1, 240);
    BPWM_SET_CMR(BPWM3, 2, 180);
    BPWM_SET_CMR(BPWM3, 3, 150);
    BPWM_SET_CMR(BPWM3, 4, 120);
    BPWM_SET_CMR(BPWM3, 5, 60);

    /* Set waveform generation */
    BPWM_SET_OUTPUT_LEVEL(BPWM3, 0x3F, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_NOTHING);
    /* Enable output of BPWM3 channel 0 ~ 5 */
    BPWM3->POEN |= 0x3F;


    /* Start BPWM0 counter */
    BPWM0->CNTEN = BPWM_CNTEN_CNTEN0_Msk;
    /* Start BPWM1 counter */
    BPWM1->CNTEN = BPWM_CNTEN_CNTEN0_Msk;
    /* Start BPWM2 counter */
    BPWM2->CNTEN = BPWM_CNTEN_CNTEN0_Msk;
    /* Start BPWM3 counter */
    BPWM3->CNTEN = BPWM_CNTEN_CNTEN0_Msk;

    printf("Press any key to stop.\n");
    getchar();

    /* Stop BPWM0 counter */
    BPWM0->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;
    /* Stop BPWM1 counter */
    BPWM1->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;
    /* Stop BPWM2 counter */
    BPWM2->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;
    /* Stop BPWM3 counter */
    BPWM3->CNTEN &= ~BPWM_CNTEN_CNTEN0_Msk;

    printf("Done.\n");

    while(1);

}
