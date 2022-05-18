/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 17/05/04 1:54p $
 * @brief    Show how to use the timer2 capture function to capture timer2 counter value.
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


/**
  * @brief      Timer2 IRQ
  *
  * @param      None
  *
  * @return     None
  *
  * @details    The Timer2 default IRQ.
  */
void TMR2_IRQHandler(void)
{
    if(TIMER_GetCaptureIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 capture trigger interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
    }
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
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2SEL_PCLK1, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, 0);

    /* Select Timer2 event counter pin and external capture pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~TM2_PB3_Msk)) | TM2_PB3;
    SYS->GPA_MFPH = (SYS->GPA_MFPH & (~TM2_EXT_PA9_Msk)) | TM2_EXT_PA9;
   
    /* Select Timer0 and Timer3 toggle-output pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~(TM0_PB5_Msk | TM3_PB2_Msk))) | (TM0_PB5 | TM3_PB2);
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
    volatile uint32_t u32InitCount;
    uint32_t au32CAPValue[10], u32CAPDiff;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer2 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 1000 Hz    			\n");
    printf("    - Toggle-output mode and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 2 Hz    			\n");
    printf("    - Toggle-output mode and frequency is 1 Hz	\n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is PCLK1             \n");
    printf("    - Continuous counting mode          \n");
    printf("    - Interrupt enable                  \n");
    printf("    - Compared value is 0xFFFFFF        \n");
    printf("    - Event counter mode enable         \n");
    printf("    - External capture mode enable      \n");
    printf("    - Capture trigger interrupt enable  \n");
    printf("# Connect TM0(PB.5) toggle-output pin to TM2(PB.3) event counter pin.\n");
    printf("# Connect TM3(PB.2) toggle-output pin to TM2_EXT(PA.9) external capture pin.\n\n");

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Open Timer0 in toggle-output mode and toggle-output frequency is 500 Hz */
    TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1000);

    /* Open Timer3 in toggle-output mode and toggle-output frequency is 1 Hz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Enable Timer2 event counter input and external capture function */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER2, TIMER_COUNTER_EVENT_FALLING);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_FALLING);
    TIMER_EnableInt(TIMER2);
    TIMER_EnableCaptureInt(TIMER2);

    printf("# Period between two falling edge captured event should be 500 counts.\n");

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Start Timer0, Timer3 and Timer2 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER3);
    TIMER_Start(TIMER2);

    /* Check Timer2 capture trigger interrupt counts */
    while(g_au32TMRINTCount[2] < 10)
    {
        if(g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValue[u32InitCount] = TIMER_GetCaptureData(TIMER2);
            if(u32InitCount ==  0)
            {
                printf("    [%2d]: %4d. (1st captured value)\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
            }
            else if(u32InitCount ==  1)
            {
                printf("    [%2d]: %4d. (2nd captured value) \n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount]);
            }
            else
            {
                u32CAPDiff = au32CAPValue[u32InitCount] - au32CAPValue[u32InitCount - 1];
                printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[2], au32CAPValue[u32InitCount], u32CAPDiff);
                if(u32CAPDiff != 500)
                {
                    printf("*** FAIL ***\n");
                    return -1;
                }
            }
            u32InitCount = g_au32TMRINTCount[2];
        }
    }
    printf("*** PASS ***\n\n");

    /* Stop Timer0, Timer2 and Timer3 counting */
    TIMER_Stop(TIMER0);
    TIMER_Stop(TIMER2);
    TIMER_Stop(TIMER3);

    while(1);
}
