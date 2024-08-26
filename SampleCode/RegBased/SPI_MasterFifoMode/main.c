/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure SPI0 as Master mode and demonstrate how to communicate
 *           with an off-chip SPI Slave device with FIFO mode. This sample
 *           code needs to work with SPI_SlaveFifoMode sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Basic <1=> Higher
#define SlewRateMode    0
// *** <<< end of configuration section >>> ***

#define TEST_COUNT      16

uint32_t g_au32SourceData[TEST_COUNT];
uint32_t g_au32DestinationData[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      SPI Master Mode Sample Code                     |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPI0_SS(PA.3)\n    SPI0_CLK(PA.2)\n");
    printf("    SPI0_MISO(PA.1)\n    SPI0_MOSI(PA.0)\n\n");
    printf("SPI controller will enable FIFO mode and transfer %d data to a off-chip slave device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip slave device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);
    printf("The SPI master configuration is ready.\n");

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        g_au32SourceData[u32DataCount] = 0x00550000 + u32DataCount;
        /* Clear destination buffer */
        g_au32DestinationData[u32DataCount] = 0;
    }

    printf("Before starting the data transfer, make sure the slave device is ready. Press any key to start the transfer.\n");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFOCTL = (2 << SPI_FIFOCTL_TXTH_Pos) | SPI_FIFOCTL_RXTOIEN_Msk | SPI_FIFOCTL_TXTHIEN_Msk;
    g_u32TxDataCount = 0;
    g_u32RxDataCount = 0;
    NVIC_EnableIRQ(SPI0_IRQn);

    /* Wait for transfer done */
    while(g_u32RxDataCount < TEST_COUNT);

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, g_au32DestinationData[u32DataCount]);
    }
    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI0->FIFOCTL = 0;
    NVIC_DisableIRQ(SPI0_IRQn);
    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 peripheral clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_SPI0CKEN_Msk;
    while(1);
}

void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 144MHz from HIRC/2 */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HIRC_DIV2;

    /* Wait for PLL clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
		if(--u32TimeOutCnt == 0) break;

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

    /* Enable SPI0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI0CKEN_Msk;

    /* Select SPI0 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS);

#if (SlewRateMode == 0)
    /* Enable SPI0 I/O basic slew rate */
    PA->SLEWCTL &= ~(GPIO_SLEWCTL_HSREN0_Msk | GPIO_SLEWCTL_HSREN1_Msk | GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk);
#elif (SlewRateMode == 1)
    /* Enable SPI0 I/O higher slew rate */
    PA->SLEWCTL |= (GPIO_SLEWCTL_HSREN0_Msk | GPIO_SLEWCTL_HSREN1_Msk | GPIO_SLEWCTL_HSREN2_Msk | GPIO_SLEWCTL_HSREN3_Msk);
#endif
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

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI0->CTL = SPI_MASTER | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSCTL = SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk;
    /* Set IP clock divider. SPI clock rate = f_PCLK0 / (35+1) */
    SPI0->CLKDIV = (SPI0->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | 35;
}

void SPI0_IRQHandler(void)
{
    /* Check RX EMPTY flag */
    while((SPI0->STATUS & SPI_STATUS_RXEMPTY_Msk) == 0)
    {
        /* Read RX FIFO */
        g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX;
    }
    /* Check TX FULL flag and TX data count */
    while(((SPI0->STATUS & SPI_STATUS_TXFULL_Msk) == 0) && (g_u32TxDataCount < TEST_COUNT))
    {
        /* Write to TX FIFO */
        SPI0->TX = g_au32SourceData[g_u32TxDataCount++];
    }
    if(g_u32TxDataCount >= TEST_COUNT)
        SPI0->FIFOCTL &= (~SPI_FIFOCTL_TXTHIEN_Msk); /* Disable TX FIFO threshold interrupt */

    /* Check the RX FIFO time-out interrupt flag */
    if(SPI0->STATUS & SPI_STATUS_RXTOIF_Msk)
    {
        /* If RX FIFO is not empty, read RX FIFO. */
        while((SPI0->STATUS & SPI_STATUS_RXEMPTY_Msk) == 0)
            g_au32DestinationData[g_u32RxDataCount++] = SPI0->RX;
    }
}
