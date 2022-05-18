/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 16/10/25 4:27p $
 * @brief    Use timer0 periodic time-out interrupt event to wake up system.
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
volatile uint8_t g_u8IsTMR0WakeupFlag = 0;
volatile uint32_t g_au32TMRINTCount[4] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    printf("\nSystem enter to power-down mode ...\n\n");

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    SCB->SCR = 4;

    /* To program PWRCTL register, it needs to disable register protection first. */
    CLK->PWRCTL &= ~CLK_PWRCTL_PDEN_Msk;
    CLK->PWRCTL |= (CLK_PWRCTL_PDEN_Msk | CLK_PWRCTL_PDWKIEN_Msk);

    __WFI();
}

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
    }

    if(TIMER_GetWakeupFlag(TIMER0) == 1)
    {
        /* Clear Timer0 wake-up flag */
        TIMER_ClearWakeupFlag(TIMER0);

        g_u8IsTMR0WakeupFlag = 1;
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
    /* Enable LIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_LIRCEN_Msk;

    /* Wait for LIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_LIRCSTB_Msk));
    
    /* Enable peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_LIRC;    
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
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Use PA.0 to measure enable/disable CNTEN bit active time */
    PA->MODE = 0x1;
    PA0 = 1;

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------+\n");
    printf("|    Timer0 Time-out Wake-up Sample Code    |\n");
    printf("+-------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is LIRC          \n");
    printf("    - Time-out frequency is 1 Hz    \n");
    printf("    - Periodic mode                 \n");
    printf("    - Interrupt enable              \n");
    printf("    - Wake-up function enable       \n");
    printf("# System will enter to Power-down mode while Timer0 interrupt counts is reaching 3.\n");
    printf("  And will be wakeup while Timer0 interrupt counts is reaching 4.\n");
    printf("# Measure PA.0 low period to check timer counter start/stop sync time should be around 1~2 TMR_CLK.\n\n");

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Open Timer0 time-out frequency to 1 Hz in periodic mode */
    TIMER0->CMP = __LIRC;
    TIMER0->CTL = TIMER_PERIODIC_MODE | TIMER_CTL_INTEN_Msk | TIMER_CTL_WKEN_Msk;

    PA0 = 0;
    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(TIMER_IS_ACTIVE(TIMER0) == 0)
        if(--u32TimeOutCnt == 0) break;
    PA0 = 1; // Around 1~2 TMR_CLK

    u32InitCount = g_u8IsTMR0WakeupFlag = g_au32TMRINTCount[0] = 0;
    while(g_au32TMRINTCount[0] < 10)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            printf("Timer0 interrupt counts - %d\n", g_au32TMRINTCount[0]);
            if(g_au32TMRINTCount[0] == 3)
            {
                /* System enter to Power-down */
                /* To program PWRCTL register, it needs to disable register protection first. */
                SYS_UnlockReg();
                PowerDownFunction();

                /* Check if Timer0 time-out interrupt and wake-up flag occurred */
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(1)
                {
                    if((CLK->PWRCTL & CLK_PWRCTL_PDWKIF_Msk) && (g_u8IsTMR0WakeupFlag == 1))
                        break;
                    if(--u32TimeOutCnt == 0)
                        break;
                }
                printf("System has been waken-up done. (Timer0 interrupt counts is %d)\n\n", g_au32TMRINTCount[0]);
            }
            u32InitCount = g_au32TMRINTCount[0];
        }
    }

    /* Stop Timer0 counting */
    PA0 = 0;
    TIMER_Stop(TIMER0);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(TIMER_IS_ACTIVE(TIMER0))
        if(--u32TimeOutCnt == 0) break;
    PA0 = 1; // Around 1~2 TMR_CLK

    printf("*** PASS ***\n");

    while(1);
}
