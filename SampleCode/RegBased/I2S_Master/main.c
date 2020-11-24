/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Configure SPI1 as I2S Master mode and demonstrate how I2S works in Master mode.
 *           This sample code needs to work with I2S_Slave sample code.
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
void UART_Init(void);

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

    /* Init UART for print message */
    UART_Init();

    printf("+-----------------------------------------------------------+\n");
    printf("|            I2S Driver Sample Code (master mode)           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x55005501, 0x55025503, ..., 0x55FE55FF, wraparound\n");
    printf("  The I/O connection for I2S1 (SPI1):\n");
    printf("      I2S1_LRCLK (PC.0)\n      I2S1_BCLK (PC.1)\n");
    printf("      I2S1_DI (PC.3)\n      I2S1_DO (PC.2)\n\n");
    printf("  NOTE: Connect with a I2S slave device.\n");
    printf("        This sample code will transmit a TX value 50000 times, and then change to the next TX value.\n");
    printf("        When TX value or the received value changes, the new TX value or the current TX value and the new received value will be printed.\n");
    printf("  Press any key to start ...");
    getchar();
    printf("\n");

    /* Master mode, 16-bit word width, stereo mode, I2S format. */
    SPI1->I2SCTL = SPII2S_MODE_MASTER | SPII2S_DATABIT_16 | SPII2S_STEREO | SPII2S_FORMAT_I2S;
    /* Set TX FIFO threshold to 2 and RX FIFO threshold to 1 */
    SPI1->FIFOCTL = SPII2S_FIFO_TX_LEVEL_WORD_2 | SPII2S_FIFO_RX_LEVEL_WORD_2;
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    SPI1->I2SCLK = (SPI1->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (71 << SPI_I2SCLK_BCLKDIV_Pos);
    /* Enable I2S */
    SPI1->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;
    NVIC_EnableIRQ(SPI1_IRQn);

    /* Initiate data counter */
    g_u32DataCount = 0;
    /* Initiate TX value and RX value */
    g_u32TxValue = 0x55005501;
    u32RxValue1 = 0;
    u32RxValue2 = 0;
    /* Enable TX threshold level interrupt */
    SPI1->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;

    /* Enable I2S TX function to transmit data */
    SPI1->I2SCTL |= SPI_I2SCTL_TXEN_Msk;
    /* Enable I2S RX function to receive data */
    SPI1->I2SCTL |= SPI_I2SCTL_RXEN_Msk;

    printf("Start I2S ...\nTX value: 0x%X\n", g_u32TxValue);

    while(1)
    {
        /* Check RX FIFO empty flag */
        if((SPI1->I2SSTS & SPI_I2SSTS_RXEMPTY_Msk) == 0)
        {
            /* Read RX FIFO */
            u32RxValue2 = SPI1->RX;
            if(u32RxValue1 != u32RxValue2)
            {
                u32RxValue1 = u32RxValue2;
                /* If received value changes, print the current TX value and the new received value. */
                printf("TX value: 0x%X;  RX value: 0x%X\n", g_u32TxValue, u32RxValue1);
            }
        }
        if(g_u32DataCount >= 50000)
        {
            g_u32TxValue = 0x55005500 | ((g_u32TxValue + 0x00020002) & 0x00FF00FF); /* g_u32TxValue: 0x55005501, 0x55025503, ..., 0x55FE55FF */
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

    /* Enable SPI1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;

    /* Select SPI1 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;

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

void UART_Init(void)
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

void SPI1_IRQHandler()
{
    /* Write 2 TX values to TX FIFO */
    SPI1->TX = g_u32TxValue;
    SPI1->TX = g_u32TxValue;
    g_u32DataCount += 2;
}
