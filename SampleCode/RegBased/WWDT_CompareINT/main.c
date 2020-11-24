/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:28p $
 * @brief    Show how to reload the WWDT counter value.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u32WWDTINTCounts;


/**
 * @brief       IRQ Handler for WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WWDT.
 */
void WDT_IRQHandler(void)
{
    if(WWDT_GET_INT_FLAG() == 1)
    {
        /* Clear WWDT compare match interrupt flag */
        WWDT_CLEAR_INT_FLAG();

        PA0 ^= 1;

        g_u32WWDTINTCounts++;

        if(g_u32WWDTINTCounts < 10)
        {
            /* To reload the WWDT counter value to 0x3F */
            WWDT_RELOAD_COUNTER();
        }

        printf("WWDT compare match interrupt occurred. (%d)\n", g_u32WWDTINTCounts);
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | (UART0_RXD_PB12 | UART0_TXD_PB13);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_WDTCKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_WWDTSEL_Msk)) | CLK_CLKSEL2_WWDTSEL_HCLK_DIV2048;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    double dPeriodTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    WWDT Compare March Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /* To check if system has been reset by WWDT time-out reset or not */
    if(WWDT_GET_RESET_FLAG() == 1)
    {
        printf("*** System has been reset by WWDT time-out reset event. [WWDT_CTL: 0x%08X] ***\n\n", WWDT->CTL);
        WWDT_CLEAR_RESET_FLAG();
        while(1);
    }

    dPeriodTime = (((double)(1000000 * 2048) / (double)SystemCoreClock) * 1024) * 32;

    printf("# WWDT Settings: \n");
    printf("    - Clock source is PCLK0/2048 (%d Hz)    \n", SystemCoreClock / 2048);
    printf("    - WWDT counter prescale period is 1024, \n");
    printf("        and max WWDT time-out period is 1024 * (64 * WWDT_CLK)\n");
    printf("    - Interrupt enable                      \n");
    printf("    - Window Compare value is 32            \n");
    printf("# System will generate first WWDT compare match interrupt event after %.2f us.\n", dPeriodTime);
    printf("    1.) use PA.0 high/low period to check WWDT compare match interrupt period time\n");
    printf("    2.) reload WWDT counter value to avoid WWDT time-out reset system occurred\n");
    printf("        when interrupt counts less than 11.\n");
    printf("    3.) do not reload WWDT counter value to generate WWDT time-out reset system event\n");
    printf("        when interrupt counts large than 10.\n\n");

    /* Use PA.0 to check WWDT compare match interrupt period time */
    PA->MODE = 0xFFFFFFFD;
    PA0 = 1;

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    g_u32WWDTINTCounts = 0;

    /*
        Max time-out period is 1024*(64*WWDT_CLK);
        WWDT compare value is 32;
        Enable WWDT compare match interrupt;
    */
    /* Note: WWDT_CTL register can be written only once after chip is powered on or reset */
    WWDT->CTL = WWDT_PRESCALER_1024 |
                (32 << WWDT_CTL_CMPDAT_Pos) |
                WWDT_CTL_INTEN_Msk |
                WWDT_CTL_WWDTEN_Msk;

    printf("[WWDT_CTL: 0x%08X]\n\n", WWDT->CTL);

    while(1);
}
