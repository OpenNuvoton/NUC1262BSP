/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Configure SPI1 as I2S Slave mode and demonstrate how I2S works in Slave mode.
 *           This sample code needs to work with I2S_Master.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

volatile uint32_t g_u32TxValue;
volatile uint32_t g_u32DataCount;

/* Function prototype declaration */
void SYS_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32RxValue1, u32RxValue2;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);
    /* Init UART0 to 115200-8n1 for print messages */
    UART_Open(UART0, 115200);

    printf("+----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (slave mode)           |\n");
    printf("+----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0xAA00AA01, 0xAA02AA03, ..., 0xAAFEAAFF, wraparound\n");
    printf("  The I/O connection for I2S1 (SPI1):\n");
    printf("      I2S1_LRCLK (PC.0)\n      I2S1_BCLK (PC.1)\n");
    printf("      I2S1_DI (PC.3)\n      I2S1_DO (PC.2)\n\n");
    printf("  NOTE: Connect with a I2S master.\n");
    printf("        This sample code will transmit a TX value 50000 times, and then change to the next TX value.\n");
    printf("        When TX value or the received value changes, the new TX value or the current TX value and the new received value will be printed.\n");
    printf("  Press any key to start ...");
    getchar();
    printf("\n");

    /* Slave mode, 16-bit word width, stereo mode, I2S format. Set TX FIFO threshold to 2 and RX FIFO threshold to 1. */
    /* I2S peripheral clock rate is equal to PCLK1 clock rate. */
    SPII2S_Open(SPI1, SPII2S_MODE_SLAVE, 0, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);

    /* Initiate data counter */
    g_u32DataCount = 0;
    /* Initiate TX value and RX value */
    g_u32TxValue = 0xAA00AA01;
    u32RxValue1 = 0;
    u32RxValue2 = 0;
    /* Enable TX threshold level interrupt */
    SPII2S_EnableInt(SPI1, SPII2S_FIFO_TXTH_INT_MASK);
    NVIC_EnableIRQ(SPI1_IRQn);

    printf("Start I2S ...\nTX value: 0x%X\n", g_u32TxValue);

    while(1)
    {
        /* Check RX FIFO empty flag */
        if((SPI1->I2SSTS & SPI_I2SSTS_RXEMPTY_Msk) == 0)
        {
            /* Read RX FIFO */
            u32RxValue2 = SPII2S_READ_RX_FIFO(SPI1);
            if(u32RxValue1 != u32RxValue2)
            {
                u32RxValue1 = u32RxValue2;
                /* If received value changes, print the current TX value and the new received value. */
                printf("TX value: 0x%X;  RX value: 0x%X\n", g_u32TxValue, u32RxValue1);
            }
        }
        if(g_u32DataCount >= 50000)
        {
            g_u32TxValue = 0xAA00AA00 | ((g_u32TxValue + 0x00020002) & 0x00FF00FF); /* g_u32TxValue: 0xAA00AA01, 0xAA02AA03, ..., 0xAAFEAAFF */
            printf("TX value: 0x%X\n", g_u32TxValue);
            g_u32DataCount = 0;
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

    /* Enable SPI1 module clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select SPI1 module clock source as PCLK0 */
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Configure SPI1 related multi-function pins. */
    /* GPC[3:0] : SPI1_MISO (I2S1_DI), SPI1_MOSI (I2S1_DO), SPI1_CLK (I2S1_BCLK), SPI1_SS (I2S1_LRCLK). */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_SPI1_SS | SYS_GPC_MFPL_PC1MFP_SPI1_CLK | SYS_GPC_MFPL_PC2MFP_SPI1_MOSI | SYS_GPC_MFPL_PC3MFP_SPI1_MISO;
}

void SPI1_IRQHandler()
{
    /* Write 2 TX values to TX FIFO */
    SPII2S_WRITE_TX_FIFO(SPI1, g_u32TxValue);
    SPII2S_WRITE_TX_FIFO(SPI1, g_u32TxValue);
    g_u32DataCount += 2;
}
