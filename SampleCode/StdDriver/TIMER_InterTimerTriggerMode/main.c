/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 17/05/04 1:55p $
 * @brief    Demonstrate how to use Inter-Timer trigger function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


int volatile complete = 0;

/**
 * @brief       Timer1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer1 default IRQ.
 */
void TMR1_IRQHandler(void)
{
    printf("Event frequency is %d Hz\n", ((12000000 * 100) / TIMER_GetCounter(TIMER1)));
    TIMER_ClearCaptureIntFlag(TIMER1);
    complete = 1;
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | (UART0_RXD_PB12 | UART0_TXD_PB13);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable GPIO Port F module clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOFCKEN_Msk;

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);

    /* Set Timer0 event counting pin PB.5 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~TM0_PB5_Msk)) | TM0_PB5;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32DelayTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------+\n");
    printf("|    Inter-Timer Trigger Sample Code    |\n");
    printf("+---------------------------------------+\n\n");

    printf("This sample code demonstrate inter-timer trigger mode on TIMER0 and TIMER1.\n\n");
    printf("Please connect input source with Timer 0 counter pin PB.5, press any key to continue.\n");
    getchar();
    /* This sample code demonstrate inter timer trigger mode using Timer0 and Timer1
     * In this mode, Timer0 is working as counter, and triggers Timer1. Using Timer1
     * to calculate the amount of time used by Timer0 to count specified amount of events.
     * By dividing the time period recorded in Timer1 by the event counts, we get
     * the event frequency.
     */

    /* Give a dummy target frequency here. Will over write prescale and compare value with macro */
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);

    /* Update prescale and compare value. Calculate average frequency every 1000 events */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 100);

    /* Update Timer 1 prescale value. So Timer 0 clock is 12MHz */
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);

    /* We need capture interrupt */
    NVIC_EnableIRQ(TMR1_IRQn);

    while(1)
    {
        complete = 0;
        /* Count event by timer 0, disable drop count (set to 0), disable timeout (set to 0). Enable interrupt after complete */
        TIMER_EnableFreqCounter(TIMER0, 0, 0, TRUE);
        while(complete == 0);
    }
}
