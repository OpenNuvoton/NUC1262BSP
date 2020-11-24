/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    NUC1262 SPI Driver Sample Code
 *           This is a I2S demo for recording data and demonstrate how I2S works with PDMA.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define I2S_TXData_DMA_CH 1
#define I2S_RX_DMA_CH 2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN 16
#define TX_BUFF_LEN 32

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

DESC_TABLE_T g_asDescTable_DataTX[1], g_asDescTable_RX[2];

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);

/* Global variable declaration */
volatile uint8_t u8RxIdx = 0;
uint32_t PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t PcmTxDataBuff[1][TX_BUFF_LEN] = {0};

/* Since ping-pong buffer would record infinitely, in this case we only want to make sure the first buffer of RX Buffer1 and Buffer2 are correct.
   Hence use g_PcmRxBuff and g_count to check and record the first buffer of RX Buffer1 and Buffer2*/
uint32_t g_PcmRxBuff[2][BUFF_LEN] = {0};
volatile uint8_t g_count = 0;

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t id)
{
    g_asDescTable_RX[id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_RX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32InitValue, u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    printf("\n");
    printf("+----------------------------------------------+\n");
    printf("|        I2S + PDMA  Record Sample Code        |\n");
    printf("+----------------------------------------------+\n");
    printf("  I2S configuration:\n");
    printf("      Sample rate 16 kHz\n");
    printf("      Word width 16 bits\n");
    printf("      Stereo mode\n");
    printf("      I2S format\n");
    printf("      TX value: 0x50005000, 0x50015001, ... \n");
    printf("  The I/O connection for I2S:\n");
    printf("      I2S_LRCLK (PC.0)\n      I2S_BCLK (PC.1)\n");
    printf("      I2S_DI (PC.3)\n      I2S_DO (PC.2)\n\n");
    printf("      This sample code will transmit and receive data with PDMA transfer.\n");
    printf("      Connect I2S_DI and I2S_DO to check if the data which stored in two receive\n buffers are the same with the transmitted values.\n");
    printf("      After PDMA transfer is finished, the received values will be printed.\n\n");

    /* Select SPI1 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;

    /* Reset SPI/I2S */
    SYS->IPRST1 |= SYS_IPRST1_SPI1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_SPI1RST_Msk;

    /* Master mode, 16-bit word width, stereo mode, I2S format. */
    SPI1->I2SCTL = SPII2S_MODE_MASTER | SPII2S_DATABIT_16 | SPII2S_STEREO | SPII2S_FORMAT_I2S;
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    SPI1->I2SCLK = (SPI1->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (11 << SPI_I2SCLK_BCLKDIV_Pos);
    /* Enable I2S */
    SPI1->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;

    /* Data initiation */
    u32InitValue = 0x50005000;
    for(u32DataCount = 0; u32DataCount < TX_BUFF_LEN; u32DataCount++)
    {
        PcmTxDataBuff[0][u32DataCount] = u32InitValue;
        u32InitValue += 0x00010001;
    }

    /* Enable PDMA channels */
    PDMA->DSCT[1].CTL = 0;
    PDMA->DSCT[2].CTL = 0;
    PDMA->CHCTL |= (1 << I2S_TXData_DMA_CH) | (1 << I2S_RX_DMA_CH);

    /* Tx description */
    g_asDescTable_DataTX[0].CTL = ((TX_BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_BASIC;
    g_asDescTable_DataTX[0].SA = (uint32_t)&PcmTxDataBuff[0];
    g_asDescTable_DataTX[0].DA = (uint32_t)&SPI1->TX;

    /* Rx(Record) description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI1->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI1->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&PcmRxBuff[1];
    g_asDescTable_RX[1].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);   //link to first description

    /* Configure PDMA transfer mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 8)) | (PDMA_SPI1_TX << 8);
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 16)) | (PDMA_SPI1_RX << 16);

    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[1].FIRST = (uint32_t)&g_asDescTable_DataTX[0] - (PDMA->SCATBA);

    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[2].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);

    /* Enable PDMA channel 2 interrupt */
    PDMA->INTEN |= (1 << 2);

    NVIC_EnableIRQ(PDMA_IRQn);

    /* Clear RX FIFO */
    SPII2S_CLR_RX_FIFO(SPI1);

    /* Enable RX function and TX function */
    SPI1->I2SCTL |= (SPI_I2SCTL_RXEN_Msk | SPI_I2SCTL_TXEN_Msk);
    /* Enable RX PDMA and TX PDMA function */
    SPI1->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);

    /* wait RX Buffer 1 and RX Buffer 2 get first buffer */
    while(g_count != 2);

    /* Once I2S is enabled, CLK would be sent immediately, we still have chance to get zero data at beginning,
       in this case we only need to make sure the on-going data is correct */
    printf("RX Buffer 1\tRX Buffer 2\n");

    /* Print the received data */
    for(u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
    {
        printf("0x%X\t\t0x%X\n", g_PcmRxBuff[0][u32DataCount], g_PcmRxBuff[1][u32DataCount]);
    }

    printf("\n\nExit I2S sample code.\n");

    while(1);

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

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

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

void PDMA_IRQHandler(void)
{
    uint32_t u32DataCount = 0;
    uint32_t u32Status = PDMA_GET_INT_STATUS();

    if(u32Status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS() & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & 0x2)
    {
        if(PDMA_GET_TD_STS() & 0x4)             /* channel 2 done */
        {
            /* record the first buffer */
            if(g_count == 0)
            {
                for(u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    g_PcmRxBuff[0][u32DataCount] = PcmRxBuff[0][u32DataCount];
            }
            else if(g_count == 1)
            {
                for(u32DataCount = 0; u32DataCount < BUFF_LEN; u32DataCount++)
                    g_PcmRxBuff[1][u32DataCount] = PcmRxBuff[1][u32DataCount];
            }

            ++g_count;
            /* Reset PDMA Scatter-Gather table */
            PDMA_ResetRxSGTable(u8RxIdx);
            u8RxIdx ^= 1;
        }
        PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}
