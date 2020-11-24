/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    NUC1262 I2S Driver Sample Code
 *           This is a I2S demo with PDMA function connected with NAU8822 codec.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

// use LIN as source, undefine it if MIC is used
#define INPUT_IS_LIN

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        512
#define BUFF_HALF_LEN   (BUFF_LEN/2)

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

#define NAU8822_ADDR    0x1A                /* NAU8822 Device ID */
volatile uint8_t u8TxIdx = 0, u8RxIdx = 0;
uint32_t PcmRxBuff[2][BUFF_LEN] = {0};
uint32_t PcmTxBuff[2][BUFF_LEN] = {0};
uint32_t volatile u32BuffPos = 0;
DESC_TABLE_T g_asDescTable_TX[2], g_asDescTable_RX[2];

void Delay(int count)
{
    volatile uint32_t i;
    for(i = 0; i < count ; i++);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C0                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8addr, uint16_t u16data)
{

    I2C_START(I2C0);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, NAU8822_ADDR << 1);
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)((u8addr << 1) | (u16data >> 8)));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    I2C_SET_DATA(I2C0, (uint8_t)(u16data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    I2C_WAIT_READY(I2C0);

    I2C_STOP(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup()
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    Delay(0x200);

#ifdef INPUT_IS_LIN //input source is LIN
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1FF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1FF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x060);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x060);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else   //input source is MIC
    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x1AD);   /* Divide by 6, 16K */
    I2C_WriteNAU8822(7,  0x006);   /* 16K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */
    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    printf("[OK]\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable GPIO Port F module clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOFCKEN_Msk;

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC and HXT clock */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    while (!(CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk)));

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

    /* Enable I2C0 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_I2C0CKEN_Msk;

    /* Enable PDMA peripheral clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PC multi-function pins for I2C0 */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk);
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL;

    /* Set PB multi-function pins for SPI0_I2SMCLK. */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_SPI0_I2SMCLK;
    /* Configure SPI0 related multi-function pins. */
    /* GPA[3:0] : SPI0_SS (I2S_LRCLK), SPI0_CLK (I2S_BCLK), SPI0_MISO (I2S_DI), SPI0_MOSI (I2S_DO). */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;
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

// Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    /* Tx description */
    g_asDescTable_TX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[0].SA = (uint32_t)&PcmTxBuff[0];
    g_asDescTable_TX[0].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[0].FIRST = (uint32_t)&g_asDescTable_TX[1] - (PDMA->SCATBA);

    g_asDescTable_TX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_TX[1].SA = (uint32_t)&PcmTxBuff[1];
    g_asDescTable_TX[1].DA = (uint32_t)&SPI0->TX;
    g_asDescTable_TX[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);   //link to first description

    /* Rx description */
    g_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[0].DA = (uint32_t)&PcmRxBuff[0];
    g_asDescTable_RX[0].FIRST = (uint32_t)&g_asDescTable_RX[1] - (PDMA->SCATBA);

    g_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    g_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    g_asDescTable_RX[1].DA = (uint32_t)&PcmRxBuff[1];
    g_asDescTable_RX[1].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);   //link to first description

    /* Open PDMA channel 1 for SPI TX and channel 2 for SPI RX */
    PDMA->DSCT[1].CTL = 0;
    PDMA->DSCT[2].CTL = 0;
    PDMA->CHCTL |= (1 << 1) | (1 << 2);

    /* Configure PDMA transfer mode */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 8)) | (PDMA_SPI0_TX << 8);
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~(0x3Ful << 16)) | (PDMA_SPI0_RX << 16);

    PDMA->DSCT[1].CTL = (PDMA->DSCT[1].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[1].FIRST = (uint32_t)&g_asDescTable_TX[0] - (PDMA->SCATBA);

    PDMA->DSCT[2].CTL = (PDMA->DSCT[2].CTL & ~PDMA_DSCT_CTL_OPMODE_Msk) | PDMA_OP_SCATTER;
    PDMA->DSCT[2].FIRST = (uint32_t)&g_asDescTable_RX[0] - (PDMA->SCATBA);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA->INTEN |= (1 << 1) | (1 << 2);

    NVIC_EnableIRQ(PDMA_IRQn);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetTxSGTable(uint8_t id)
{
    g_asDescTable_TX[id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_TX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Once PDMA has transferred, software need to reset Scatter-Gather table */
void PDMA_ResetRxSGTable(uint8_t id)
{
    g_asDescTable_RX[id].CTL |= PDMA_OP_SCATTER;
    g_asDescTable_RX[id].CTL |= ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
}

/* Init I2C interface */
void I2C0_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C0 */
    SYS->IPRST1 |=  SYS_IPRST1_I2C0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->CTL |= I2C_CTL_I2CEN_Msk;

    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C0->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                   SPI Driver Sample Code with NAU8822                  |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with NAU8822.\n");

    /* Init I2C0 to access NAU8822 */
    I2C0_Init();

#ifdef INPUT_IS_LIN
    /* Slave mode, 16-bit word width, stereo mode, I2S format. */
    SPI0->I2SCTL = SPII2S_MODE_SLAVE | SPII2S_DATABIT_16 | SPII2S_STEREO | SPII2S_FORMAT_I2S;
    /* Set TX FIFO threshold to 2 and RX FIFO threshold to 1 */
    SPI0->FIFOCTL = SPII2S_FIFO_TX_LEVEL_WORD_2 | SPII2S_FIFO_RX_LEVEL_WORD_2;
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    SPI0->I2SCLK = (SPI0->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (11 << SPI_I2SCLK_BCLKDIV_Pos);
    /* Enable I2S */
    SPI0->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;
#else
    /* Slave mode, 16-bit word width, mono mode, I2S format. */
    SPI0->I2SCTL = SPII2S_MODE_SLAVE | SPII2S_DATABIT_16 | SPII2S_MONO | SPII2S_FORMAT_I2S;
    /* Set TX FIFO threshold to 2 and RX FIFO threshold to 1 */
    SPI0->FIFOCTL = SPII2S_FIFO_TX_LEVEL_WORD_2 | SPII2S_FIFO_RX_LEVEL_WORD_2;
    /* Sampling rate 16000 Hz; bit clock rate 512 kHz. */
    SPI0->I2SCLK = (SPI0->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (11 << SPI_I2SCLK_BCLKDIV_Pos);
    /* Enable I2S */
    SPI0->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;
#endif

    /* Select SPI0 module clock source as HXT */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_HXT;

    /* Initialize NAU8822 codec */
    NAU8822_Setup();

    /* Set MCLK and enable MCLK */
    SPI0->I2SCLK = (SPI0->I2SCLK & ~SPI_I2SCLK_MCLKDIV_Msk) | (0 << SPI_I2SCLK_MCLKDIV_Pos);
    SPI0->I2SCTL |= SPI_I2SCTL_MCLKEN_Msk;

#ifndef INPUT_IS_LIN
    SPII2S_SET_MONO_RX_CHANNEL(SPI0, SPII2S_MONO_LEFT);       //NAU8822 will store data in left channel
#endif

    PDMA_Init();

    /* Enable RX PDMA and TX PDMA function */
    SPI0->PDMACTL = (SPI_PDMACTL_RXPDMAEN_Msk | SPI_PDMACTL_TXPDMAEN_Msk);
    /* Enable RX function and TX function */
    SPI0->I2SCTL |= (SPI_I2SCTL_RXEN_Msk | SPI_I2SCTL_TXEN_Msk);

    while(1);
}

void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA->INTSTS;

    if(u32Status & 0x1)    /* abort */
    {
        if((PDMA->ABTSTS) & 0x4)
            PDMA->ABTSTS |= PDMA_ABTSTS_ABTIF1_Msk;
        PDMA->ABTSTS |= PDMA_ABTSTS_ABTIF2_Msk;
    }
    else if(u32Status & 0x2)
    {
        if((PDMA->TDSTS) & 0x4)             /* channel 2 done */
        {
            /* Copy RX data to TX buffer */
            memcpy(&PcmTxBuff[u8TxIdx ^ 1], &PcmRxBuff[u8RxIdx], BUFF_LEN * 4);
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetRxSGTable(u8RxIdx);
            u8RxIdx ^= 1;
            PDMA->TDSTS |= PDMA_TDSTS_TDIF2_Msk;
        }
        if((PDMA->TDSTS) & 0x2)             /* channel 1 done */
        {
            /* Reset PDMA Scater-Gatter table */
            PDMA_ResetTxSGTable(u8TxIdx);
            u8TxIdx ^= 1;
            PDMA->TDSTS |= PDMA_TDSTS_TDIF1_Msk;
        }
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}
