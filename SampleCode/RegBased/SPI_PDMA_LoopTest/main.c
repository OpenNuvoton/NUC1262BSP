/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate SPI data transfer with PDMA.
 *           SPI0 will be configured as Master mode and SPI1 will be configured as Slave mode.
 *           Both TX PDMA function and RX PDMA function will be enabled.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SPI_MASTER_TX_DMA_CH 0
#define SPI_MASTER_RX_DMA_CH 1
#define SPI_SLAVE_TX_DMA_CH  2
#define SPI_SLAVE_RX_DMA_CH  3

#define TEST_COUNT 64

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void SPI_Init(void);
void SpiLoopTest_WithPDMA(void);

/* Global variable declaration */
uint32_t g_au32MasterToSlaveTestPattern[TEST_COUNT];
uint32_t g_au32SlaveToMasterTestPattern[TEST_COUNT];
uint32_t g_au32MasterRxBuffer[TEST_COUNT];
uint32_t g_au32SlaveRxBuffer[TEST_COUNT];

int main(void)
{
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
    printf("+--------------------------------------------------------------+\n");
    printf("|                    SPI + PDMA Sample Code                    |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a master and SPI1 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0/SPI1 loopback:\n");
    printf("    SPI0_SS  (PA.3) <--> SPI1_SS  (PC.0)\n    SPI0_CLK (PA.2) <--> SPI1_CLK (PC.1)\n");
    printf("    SPI0_MISO(PA.1) <--> SPI1_MISO(PC.3)\n    SPI0_MOSI(PA.0) <--> SPI1_MOSI(PC.2)\n\n");
    printf("Please connect SPI0 with SPI1, and press any key to start transmission ...");
    getchar();
    printf("\n");

    SpiLoopTest_WithPDMA();

    printf("\n\nExit SPI driver sample code.\n");

    /* Disable SPI0 and SPI1 peripheral clock */
    CLK->APBCLK0 &= ~(CLK_APBCLK0_SPI0CKEN_Msk | CLK_APBCLK0_SPI1CKEN_Msk);
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

    /* Enable SPI0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI0CKEN_Msk;

    /* Select SPI0 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;

    /* Enable SPI1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;

    /* Select SPI1 module clock source as PCLK0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Configure SPI0 related multi-function pins. GPA[3:0] : SPI0_SS, SPI0_CLK, SPI0_MISO, SPI0_MOSI. */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;

    /* Configure SPI1 related multi-function pins. GPC[3:0] : SPI1_MISO, SPI1_MOSI, SPI1_CLK, SPI1_SS. */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk);
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_SPI1_SS | SYS_GPC_MFPL_PC1MFP_SPI1_CLK | SYS_GPC_MFPL_PC2MFP_SPI1_MOSI | SYS_GPC_MFPL_PC3MFP_SPI1_MISO;
}

void UART_Init()
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
    /* Configure SPI0 */
    /* Configure SPI0 as a master, clock idle low, TX on falling clock edge, RX on rising edge and 32-bit transaction. */
    SPI0->CTL = SPI_MASTER | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI0->SSCTL = SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS_Msk;
    /* Set SPI0 clock divider. SPI clock rate = PCLK / (35+1) */
    SPI0->CLKDIV = (SPI0->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (35 << SPI_CLKDIV_DIVIDER_Pos);

    /* Configure SPI1 */
    /* Configure SPI1 as a slave, clock idle low, TX on falling clock edge, RX on rising edge and 32-bit transaction */
    SPI1->CTL = SPI_SLAVE | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Configure SPI1's slave select signal as a low level active device. */
    SPI1->SSCTL = 0;
    /* Set SPI1 clock divider. SPI peripheral clock rate = f_PCLK0 */
    SPI1->CLKDIV = 0;
}

void SpiLoopTest_WithPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;


    printf("\nSPI0/1 Loop test with PDMA ");

    /* Source data initiation */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        g_au32MasterToSlaveTestPattern[u32DataCount] = 0x55000000 | (u32DataCount + 1);
        g_au32SlaveToMasterTestPattern[u32DataCount] = 0xAA000000 | (u32DataCount + 1);
    }

    /* SPI master PDMA TX channel configuration */
    PDMA->CHCTL |= (1 << SPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128   | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].SA = (uint32_t)g_au32MasterToSlaveTestPattern;
    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].DA = (uint32_t)&SPI0->TX;
#if(SPI_MASTER_TX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (SPI_MASTER_TX_DMA_CH % 4))))) |
                      ((0 + 16) << (8 * (SPI_MASTER_TX_DMA_CH % 4))); /* SPIx_TX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (SPI_MASTER_TX_DMA_CH % 4))))) |
                    ((0 + 16) << (8 * (SPI_MASTER_TX_DMA_CH % 4))); /* SPIx_TX */
#endif

    /* SPI master PDMA RX channel configuration */
    PDMA->CHCTL |= (1 << SPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].SA = (uint32_t)&SPI0->RX;
    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].DA = (uint32_t)g_au32MasterRxBuffer;
#if(SPI_MASTER_RX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (SPI_MASTER_RX_DMA_CH % 4))))) |
                      ((0 + 17) << (8 * (SPI_MASTER_RX_DMA_CH % 4))); /* SPIx_RX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (SPI_MASTER_RX_DMA_CH % 4))))) |
                    ((0 + 17) << (8 * (SPI_MASTER_RX_DMA_CH % 4))); /* SPIx_RX */
#endif

    /* SPI slave PDMA RX channel configuration */
    PDMA->CHCTL |= (1 << SPI_SLAVE_RX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].SA = (uint32_t)&SPI1->RX;
    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].DA = (uint32_t)g_au32SlaveRxBuffer;
#if(SPI_SLAVE_RX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (SPI_SLAVE_RX_DMA_CH % 4))))) |
                      ((2 + 17) << (8 * (SPI_SLAVE_RX_DMA_CH % 4))); /* SPIx_RX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (SPI_SLAVE_RX_DMA_CH % 4))))) |
                    ((2 + 17) << (8 * (SPI_SLAVE_RX_DMA_CH % 4))); /* SPIx_RX */
#endif

    /* SPI slave PDMA TX channel configuration */
    PDMA->CHCTL |= (1 << SPI_SLAVE_TX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_32 | /* Transfer width 32 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].SA = (uint32_t)g_au32SlaveToMasterTestPattern;
    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].DA = (uint32_t)&SPI1->TX;
#if(SPI_SLAVE_TX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (SPI_SLAVE_TX_DMA_CH % 4))))) |
                      ((2 + 16) << (8 * (SPI_SLAVE_TX_DMA_CH % 4))); /* SPIx_TX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (SPI_SLAVE_TX_DMA_CH % 4))))) |
                    ((2 + 16) << (8 * (SPI_SLAVE_TX_DMA_CH % 4))); /* SPIx_TX */
#endif

    /* Enable SPI slave DMA function */
    SPI1->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);
    /* Enable SPI master DMA function */
    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);


    i32Err = 0;
    for(u32TestCycle = 0; u32TestCycle < 10000; u32TestCycle++)
    {
        if((u32TestCycle & 0x1FF) == 0)
            putchar('.');

        while(1)
        {
            u32RegValue = PDMA->INTSTS;
            /* Check the DMA transfer done interrupt flag */
            if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                if((PDMA->TDSTS & ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH))) ==
                        ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH)))
                {
                    /* Clear the DMA transfer done flag */
                    PDMA->TDSTS = ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH));
                    /* Disable SPI master's DMA transfer function */
                    SPI0->PDMACTL = 0;
                    /* Check the transfer data */
                    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        if(g_au32MasterToSlaveTestPattern[u32DataCount] != g_au32SlaveRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                        if(g_au32SlaveToMasterTestPattern[u32DataCount] != g_au32MasterRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if(u32TestCycle >= 10000)
                        break;

                    /* Source data initiation */
                    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        g_au32MasterToSlaveTestPattern[u32DataCount]++;
                        g_au32SlaveToMasterTestPattern[u32DataCount]++;
                    }
                    /* Re-trigger */
                    /* Slave PDMA TX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_SLAVE_TX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_SLAVE_TX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Slave PDMA RX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_SLAVE_RX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_SLAVE_RX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Master PDMA TX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_MASTER_TX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Master PDMA RX channel configuration */
                    PDMA->CHCTL |= (1 << SPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL =
                        (PDMA->DSCT[SPI_MASTER_RX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Enable master's DMA transfer function */
                    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);
                    break;
                }
            }
            /* Check the DMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA->ABTSTS;
                /* Clear the target abort flag */
                PDMA->ABTSTS = u32Abort;
                i32Err = 1;
                break;
            }
            /* Check the DMA time-out interrupt flag */
            if(u32RegValue & 0x00000300)
            {
                /* Clear the time-out flag */
                PDMA->INTSTS = u32RegValue & 0x00000300;
                i32Err = 1;
                break;
            }
        }

        if(i32Err)
            break;
    }

    /* Disable PDMA clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_PDMACKEN_Msk;

    if(i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}
