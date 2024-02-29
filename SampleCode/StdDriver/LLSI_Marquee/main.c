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
    if(LLSI_GetIntFlag(LLSI0, LLSI_TXTH_INT_MASK))
    {
        while(g_u32DataCount < TEST_COUNT)
        {
            if(g_u32DataCount == (TEST_COUNT - 1))
                LLSI_SET_LAST_DATA(LLSI0);
            LLSI_WRITE_DATA(LLSI0, g_au32RED_Marquee[g_u32DataCount++]);
        }
        if(g_u32DataCount >= TEST_COUNT)
        {
            LLSI_DisableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
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

    /* Enable LLSI0 module clock */
    CLK_EnableModuleClock(LLSI0_MODULE);

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
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void LLSI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as software mode, GRB output format, 6 pixels in a frame and idle output low */
    /* Set clock divider. LLSI clock rate = 72MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 50000ns */
    LLSI_Open(LLSI0, LLSI_MODE_SW, LLSI_FORMAT_GRB, 72000000, 1250, 400, 850, 50000, 6, LLSI_IDLE_LOW);

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt */
    LLSI_SetFIFO(LLSI0, 2);
    LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
    /* Enable reset command function */
    LLSI_ENABLE_RESET_COMMAND(LLSI0);

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
    LLSI_WRITE_DATA(LLSI0, 0x000000FF);
    LLSI_WRITE_DATA(LLSI0, 0x00000000);
    LLSI_WRITE_DATA(LLSI0, 0x00000000);
    LLSI_WRITE_DATA(LLSI0, 0x00000000);

    while(g_u32PatternToggle < 7)
    {
        CLK_SysTickDelay(100000);

        g_u32DataCount = 0;
        g_u32PatternToggle++;

        if(g_u32PatternToggle == 1)
        {
            LLSI_WRITE_DATA(LLSI0, 0xFF000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
        else if(g_u32PatternToggle == 2)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00FF0000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
        else if(g_u32PatternToggle == 3)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x0000FF00);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
        else if(g_u32PatternToggle == 4)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x000000FF);
            LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
        else if(g_u32PatternToggle == 5)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0xFF000000);
            LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
        else if(g_u32PatternToggle == 6)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
    }

    /* Close LLSI0 */
    LLSI_Close(LLSI0);

    printf("Exit LLSI sample code.\n");

    while(1);
}
