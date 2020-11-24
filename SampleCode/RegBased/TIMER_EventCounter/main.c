/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 17/05/04 1:21p $
 * @brief    Implement timer2 event counter function to count the external input event.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/*  Generate Event Counter Source by specify GPIO pin                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void GenerateEventCounterSource(uint32_t u32Port, uint32_t u32Pin, uint32_t u32Counts)
{
    while(u32Counts--)
    {
        GPIO_PIN_DATA(u32Port, u32Pin) = 1;
        GPIO_PIN_DATA(u32Port, u32Pin) = 0;
    }
}

/**
 * @brief       Timer2 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer2 default IRQ.
 */
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
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
    CLK->APBCLK0 |= CLK_APBCLK0_TMR2CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR2SEL_Msk)) | CLK_CLKSEL1_TMR2SEL_PCLK1;

    /* Set Timer2 event counter pin */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~TM2_PD0_Msk)) | TM2_PD0;
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
    volatile uint32_t u32InitCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------------+\n");
    printf("|    Timer2 Event Counter Input Sample Code    |\n");
    printf("+----------------------------------------------+\n\n");

    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HCLK      \n");
    printf("    - Continuous counting mode  \n");
    printf("    - Interrupt enable          \n");
    printf("    - Event counter mode enable \n");
    printf("    - Compared value is 56789   \n");
    printf("# Connect PD.3 pin to event counter pin TM2(PD.0) and pull PD.3 High/Low to generate TM2 event input source.\n\n");

    /* Configure PD.3 as GPIO output pin and pull initial pin status to Low */
    PD->MODE = 0xFFFFFF7F;
    PD3 = 0;

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Clear Timer2 interrupt counts to 0 */
    g_au32TMRINTCount[2] = 0;

    /* Reset Timer2 counter value */
    TIMER2->CTL = 0x0;

    /* Enable Timer2 external event counter input function */
    TIMER2->CMP = 56789;
    TIMER2->CTL |= TIMER_CTL_RSTCNT_Msk;
    while(TIMER2->CNT != 0x0ul) {}
    TIMER2->CTL = TIMER_CTL_CNTEN_Msk | TIMER_CTL_INTEN_Msk | TIMER_CTL_EXTCNTEN_Msk | TIMER_CONTINUOUS_MODE;
    while(!(TIMER2->CTL & TIMER_CTL_ACTSTS_Msk));

    /* To check if counter value of Timer2 should be 0 while event counter mode is enabled */
    if(TIMER_GetCounter(TIMER2) != 0)
    {
        printf("Default counter value is not 0. (%d)\n", TIMER_GetCounter(TIMER2));

        /* Stop Timer2 counting */
        TIMER2->CTL = 0;
        while(1);
    }

    printf("Start to check Timer2 counter value ......\n\n");

    /* To generate one counter event from PD.3 to TM2 pin */
    GenerateEventCounterSource(3, 3, 1);

    /* To check if counter value of Timer2 should be 1 */
    while(TIMER_GetCounter(TIMER2) == 0);
    if(TIMER_GetCounter(TIMER2) != 1)
    {
        printf("Get unexpected counter value. (%d)\n", TIMER_GetCounter(TIMER2));

        /* Stop Timer2 counting */
        TIMER2->CTL = 0;
        while(1);
    }

    /* To generate remains counts to TM2 pin */
    GenerateEventCounterSource(3, 3, (56789 - 1));

    while(1)
    {
        if(g_au32TMRINTCount[2] == 1)
        {
            printf("# Timer2 interrupt event occurred.\n");
            break;
        }
    }

    printf("# Get Timer2 event counter value is %d .... ", TIMER_GetCounter(TIMER2));
    if(TIMER_GetCounter(TIMER2) == 56789)
    {
        printf("PASS.\n");
    }
    else
    {
        printf("FAIL.\n");
    }

    /* Stop Timer2 counting */
    TIMER2->CTL = 0;

    while(1);
}
