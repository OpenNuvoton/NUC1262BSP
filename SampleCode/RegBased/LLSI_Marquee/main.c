/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is a LLSI demo for marquee display in software mode.
 *           It needs to be used with WS2812 LED strip.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT 1

volatile uint32_t g_au32RED_Marquee[TEST_COUNT] = {0x00000000};
volatile uint32_t g_u32PatternToggle = 0;
volatile uint32_t g_u32DataCount = 0;

void LLSI0_IRQHandler()
{
    if(LLSI0->STATUS & LLSI_STATUS_TXTHIF_Msk)
    {
        while(g_u32DataCount < TEST_COUNT)
        {
            if(g_u32DataCount == (TEST_COUNT - 1))
                LLSI0->STATUS |= LLSI_STATUS_LDT_Msk;
            LLSI0->DATA = g_au32RED_Marquee[g_u32DataCount++];
        }
        if(g_u32DataCount >= TEST_COUNT)
        {
            LLSI0->CTL &= ~LLSI_CTL_TXTHIEN_Msk;
        }
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

    /* Enable LLSI0 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_LLSI0CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PB multi-function pins for LLSI0 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB15MFP_Msk)) | SYS_GPB_MFPH_PB15MFP_LLSI0_OUT;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void LLSI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set clock divider. LLSI clock rate = f_APBCLK / (5+1) */
    LLSI0->CLKDIV = (LLSI0->CLKDIV & (~LLSI_CLKDIV_DIVIDER_Msk)) | 0x5;
    /* Set transfer period. f_Period = f_APBCLK / (5+1) / 15 */
    LLSI0->PERIOD = (LLSI0->PERIOD & (~LLSI_PERIOD_PERIOD_Msk)) | 0xF;
    /* Set duty period. f_T0H = f_APBCLK / (5+1) / 5; f_T1H = f_APBCLK / (5+1) / 10 */
    LLSI0->DUTY = (0x5 << LLSI_DUTY_T0H_Pos) | (0xA << LLSI_DUTY_T1H_Pos);
    /* Set reset period. f_ResetPeriod = f_APBCLK / (5+1) / 600 */
    LLSI0->RSTPERIOD = (LLSI0->RSTPERIOD & (~LLSI_RSTPERIOD_RSTPERIOD_Msk)) | 0x258;
    /* Set 6 pixels in a frame and idle output low */
    LLSI0->PCNT = 6;
    LLSI0->OCTL = LLSI_IDLE_LOW;

    /* Set GRB output format, software mode, enable reset command function */
    LLSI0->CTL = LLSI_FORMAT_GRB | LLSI_MODE_SW | LLSI_CTL_RSTCEN_Msk;
    /* Set TX FIFO threshold and enable TX FIFO threshold interrupt */
    LLSI0->CTL = (LLSI0->CTL & ~LLSI_CTL_TXTH_Msk) | (2 << LLSI_CTL_TXTH_Pos) | LLSI_CTL_TXTHIEN_Msk;
    /* Enable LLSI */
    LLSI0->CTL |= LLSI_CTL_LLSIEN_Msk;

    NVIC_EnableIRQ(LLSI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    LLSI Marquee Sample Code (Software Mode)    |\n");
    printf("+------------------------------------------------+\n");
    printf("The first to sixth LEDs will flash red in sequence.\n\n");

    /* Init LLSI */
    LLSI_Init();

    /* Write 4 word data to LLSI_DATA */
    LLSI0->DATA = 0x000000FF;
    LLSI0->DATA = 0x00000000;
    LLSI0->DATA = 0x00000000;
    LLSI0->DATA = 0x00000000;

    while(g_u32PatternToggle < 7)
    {
        CLK_SysTickDelay(100000);

        g_u32DataCount = 0;
        g_u32PatternToggle++;

        if(g_u32PatternToggle == 1)
        {
            LLSI0->DATA = 0xFF000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->CTL |= LLSI_CTL_TXTHIEN_Msk;
        }
        else if(g_u32PatternToggle == 2)
        {
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00FF0000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->CTL |= LLSI_CTL_TXTHIEN_Msk;
        }
        else if(g_u32PatternToggle == 3)
        {
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x0000FF00;
            LLSI0->DATA = 0x00000000;
            LLSI0->CTL |= LLSI_CTL_TXTHIEN_Msk;
        }
        else if(g_u32PatternToggle == 4)
        {
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x000000FF;
            LLSI0->CTL |= LLSI_CTL_TXTHIEN_Msk;
        }
        else if(g_u32PatternToggle == 5)
        {
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0xFF000000;
            LLSI0->CTL |= LLSI_CTL_TXTHIEN_Msk;
        }
        else if(g_u32PatternToggle == 6)
        {
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->DATA = 0x00000000;
            LLSI0->CTL |= LLSI_CTL_TXTHIEN_Msk;
        }
    }

    /* Close LLSI0 */
    SYS->IPRST2 |= SYS_IPRST2_LLSI0RST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_LLSI0RST_Msk;

    printf("Exit LLSI sample code.\n");

    while(1);
}
