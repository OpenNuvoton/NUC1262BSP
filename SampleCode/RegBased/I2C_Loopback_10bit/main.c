/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 20/11/27 $
 * @brief
 *           Demonstrate how to set I2C Master mode and Slave mode.
 *           And show how a master access a slave on a chip.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define SLV_10BIT_ADDR (0x1E<<2)
#define SLV_10BIT_GC_ADDR (0x00<<2)

#if 0
#define DbgPrintf printf
#else
#define DbgPrintf(...)
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32PCLKClock = 0;
volatile uint32_t slave_buff_addr;
volatile uint8_t g_au8SlvData[256];
volatile uint8_t g_au8SlvRxData[3];
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[100];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8SlvDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint8_t g_u8SLARetryCnt, g_u8SLARetryFail;
volatile uint8_t g_u8DeviceHAddr;
volatile uint8_t g_u8DeviceLAddr;
volatile uint32_t g_u32I2CMPort, g_u32I2CSPort;

typedef void (*I2C_FUNC)(I2C_T* tI2C, uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C0->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(I2C0, u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if(I2C1->TOCTL & I2C_TOCTL_TOIF_Msk)
    {
        /* Clear I2C1 Timeout Flag */
        I2C1->TOCTL |= I2C_TOCTL_TOIF_Msk;
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(I2C1, u32Status);
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
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Enable I2C controller */
    CLK->APBCLK0 |= (CLK_APBCLK0_I2C0CKEN_Msk | CLK_APBCLK0_I2C1CKEN_Msk);

    /* Set PA/PB multi-function pins for I2C0/I2C1 SDA and SCL */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB4MFP_I2C0_SDA);

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk | SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_I2C0_SCL | SYS_GPA_MFPL_PA6MFP_I2C1_SDA | SYS_GPA_MFPL_PA7MFP_I2C1_SCL);

    /* I2C pins enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk;
    PA->SMTEN |= (GPIO_SMTEN_SMTEN5_Msk | GPIO_SMTEN_SMTEN6_Msk | GPIO_SMTEN_SMTEN7_Msk);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C0_Init(void)
{
    /* Enable I2C0 Controller */
    I2C0->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (100000 * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    I2C0->CTL |= I2C_CTL_I2CEN_Msk;
    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", (SystemCoreClock / (((I2C0->CLKDIV) + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->ADDR0 = (I2C0->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x115 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->ADDR1 = (I2C0->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x135 << I2C_ADDR1_ADDR_Pos);

    /* Set I2C0 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x01 */
    I2C0->ADDRMSK0 = (I2C0->ADDRMSK0 & ~I2C_ADDRMSK0_ADDRMSK_Msk) | (0x01 << I2C_ADDRMSK0_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->ADDRMSK1 = (I2C0->ADDRMSK1 & ~I2C_ADDRMSK1_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK1_ADDRMSK_Pos);


    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    /* Open I2C1 module and set bus clock */
    I2C1->CLKDIV = (uint32_t)(((SystemCoreClock * 10) / (100000 * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    I2C1->CTL |= I2C_CTL_I2CEN_Msk;
    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", (SystemCoreClock / (((I2C1->CLKDIV) + 1) << 2)));

    /* Slave Address : 0x16 */
    I2C1->ADDR0 = (I2C1->ADDR0 & ~I2C_ADDR0_ADDR_Msk) | (0x116 << I2C_ADDR0_ADDR_Pos);
    /* Slave Address : 0x36 */
    I2C1->ADDR1 = (I2C1->ADDR1 & ~I2C_ADDR1_ADDR_Msk) | (0x336 << I2C_ADDR1_ADDR_Pos);

    /* Set I2C1 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x04 */
    I2C1->ADDRMSK0 = (I2C1->ADDRMSK0 & ~I2C_ADDRMSK0_ADDRMSK_Msk) | (0x04 << I2C_ADDRMSK0_ADDRMSK_Pos);
    /* Slave Address Mask Bits: 0x02 */
    I2C1->ADDRMSK1 = (I2C1->ADDRMSK1 & ~I2C_ADDRMSK1_ADDRMSK_Msk) | (0x02 << I2C_ADDRMSK1_ADDRMSK_Pos);


    /* Enable I2C interrupt */
    I2C1->CTL |= I2C_CTL_INTEN_Msk;
    NVIC_EnableIRQ(I2C1_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->CTL &= ~I2C_CTL_INTEN_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C0CKEN_Msk;
}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C1->CTL &= ~I2C_CTL_INTEN_Msk;
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C1->CTL &= ~I2C_CTL_I2CEN_Msk;
    CLK->APBCLK0 &= ~CLK_APBCLK0_I2C1CKEN_Msk;
}



void I2CM_Init_10bit(I2C_T *i2c, uint32_t u32BusClock)
{
    uint32_t u32CLK;

    if(i2c == I2C0)
    {
        CLK->APBCLK0 |= (CLK_APBCLK0_I2C0CKEN_Msk);
        SYS->IPRST1 |= (1<<SYS_IPRST1_I2C0RST_Pos);
        SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;
    }
    else if(i2c == I2C1)
    {
        CLK->APBCLK0 |= (CLK_APBCLK0_I2C1CKEN_Msk);
        SYS->IPRST1 |= (1<<SYS_IPRST1_I2C1RST_Pos);
        SYS->IPRST1 &= ~SYS_IPRST1_I2C1RST_Msk;
    }


    /* Enable I2C0 Controller */
    i2c->CTL |= I2C_CTL_I2CEN_Msk;

    /* Enable I2C1 10-bit address mode */
    i2c->CTL1 |= I2C_CTL1_ADDR10EN_Msk;

    SystemCoreClockUpdate();
    g_u32PCLKClock = SystemCoreClock ;

    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32CLK = (uint32_t)(((g_u32PCLKClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    if(u32CLK<4)
        u32CLK = 4;
    i2c->CLKDIV = u32CLK;

    /* Get I2C0 Bus Clock */
    printf("I2CM clock %d Hz\n", (g_u32PCLKClock / (((i2c->CLKDIV) + 1) << 2)));

}
void I2CS_Init_10bit(I2C_T *i2c, uint32_t u32BusClock)
{
    uint32_t u32CLK;

    if(i2c == I2C0)
    {
        CLK->APBCLK0 |= (CLK_APBCLK0_I2C0CKEN_Msk);
        SYS->IPRST1 |= (1<<SYS_IPRST1_I2C0RST_Pos);
        SYS->IPRST1 &= ~SYS_IPRST1_I2C0RST_Msk;
    }
    else if(i2c == I2C1)
    {
        CLK->APBCLK0 |= (CLK_APBCLK0_I2C1CKEN_Msk);
        SYS->IPRST1 |= (1<<SYS_IPRST1_I2C1RST_Pos);
        SYS->IPRST1 &= ~SYS_IPRST1_I2C1RST_Msk;
    }


    /* Enable I2C1 Controller */
    i2c->CTL |= I2C_CTL_I2CEN_Msk;

    /* Enable I2C1 10-bit address mode */
    i2c->CTL1 |= I2C_CTL1_ADDR10EN_Msk;

    SystemCoreClockUpdate();

    /* I2C1 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32CLK = (uint32_t)(((g_u32PCLKClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */
    if(u32CLK<4)
        u32CLK = 4;
    i2c->CLKDIV = u32CLK;
    /* Get I2C0 Bus Clock */
    printf("I2CS clock %d Hz\n", (g_u32PCLKClock / (((i2c->CLKDIV) + 1) << 2)));


}

void I2C_10bit_LB_SlaveTRx(I2C_T* tI2CS, uint32_t u32Status)
{
    DbgPrintf("I2C_10bit_LB_SlaveTRx status(0x%X)\n", u32Status);
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlvDataLen = 0;
        DbgPrintf("S << SLV address(0x%X)\n", (unsigned char)(tI2CS->DAT));
        I2C_SET_CONTROL_REG(tI2CS, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        g_au8SlvRxData[g_u8SlvDataLen] = (unsigned char)(tI2CS->DAT);
        DbgPrintf("S << SLV Receive Data(0x%X)\n", g_au8SlvRxData[g_u8SlvDataLen]);
        g_u8SlvDataLen++;

        if(g_u8SlvDataLen == 2)
        {
            slave_buff_addr = (g_au8SlvRxData[0] << 8) + g_au8SlvRxData[1];
        }
        if(g_u8SlvDataLen == 3)
        {
            g_au8SlvData[slave_buff_addr] = g_au8SlvRxData[2];
            g_u8SlvDataLen = 0;
        }
        I2C_SET_CONTROL_REG(tI2CS, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
        DbgPrintf("S << SLV address(0x%X)\n", (unsigned char)(tI2CS->DAT));
        tI2CS->DAT = g_au8SlvData[slave_buff_addr];
        DbgPrintf("S >> SLV send Data(0x%X)\n", (unsigned char)(tI2CS->DAT));
        slave_buff_addr++;
        I2C_SET_CONTROL_REG(tI2CS, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(tI2CS, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(tI2CS, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(tI2CS, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}


void I2C_10bit_LB_MasterRx(I2C_T* tI2CM, uint32_t u32Status)
{
    DbgPrintf("I2C_10bit_LB_MasterRx status(0x%X)\n", u32Status);
    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        tI2CM->DAT = g_u8DeviceHAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        tI2CM->DAT = g_au8MstTxData[g_u8MstDataLen++];
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(tI2CM);
        I2C_START(tI2CM);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 3)
        {
            tI2CM->DAT = g_au8MstTxData[g_u8MstDataLen++];
            I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        tI2CM->DAT = ((g_u8DeviceHAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData = tI2CM->DAT;
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_10bit_LB_MasterTx(I2C_T *tI2CM, uint32_t u32Status)
{
    DbgPrintf("I2C_10bit_LB_MasterTx status(0x%X)\n", u32Status);
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        g_u8SLARetryCnt = 0;
        tI2CM->DAT = g_u8DeviceHAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        tI2CM->DAT = g_au8MstTxData[g_u8MstDataLen++];
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        if(g_u8SLARetryCnt++>2)
        {
            g_u8SLARetryCnt = 0;
            I2C_STOP(tI2CM);
            g_u8SLARetryFail=1;
        }
        else
        {
            //I2C_STOP(tI2CM);
            I2C_START(tI2CM);
        }
    }
    else if(u32Status == 0x10)                  /* SLA+W has been transmitted and NACK has been received */
    {
        g_u8MstDataLen = 0;
        tI2CM->DAT = g_u8DeviceHAddr << 1;     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 4)
        {
            tI2CM->DAT = g_au8MstTxData[g_u8MstDataLen++];
            I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(tI2CM, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
    }
}


int32_t I2C_10bit_Read_Write_SLAVE(I2C_T *tI2CM, I2C_T *tI2CS, uint16_t u16slvaddr)
{
    uint32_t i, u32TimeOutCnt;

    g_u32I2CMPort = (uint32_t) tI2CM;
    g_u32I2CSPort = (uint32_t) tI2CS;

    /* Init Send 10-bit Addr */
    g_u8DeviceHAddr = (u16slvaddr>>8)| SLV_10BIT_ADDR;
    g_u8DeviceLAddr = u16slvaddr&0xFF;

    g_u8SLARetryFail = 0;
    g_u8MstEndFlag = 0;

    for(i = 0; i < 0x100; i++)
    {
        g_au8MstTxData[0] = (uint8_t)g_u8DeviceLAddr;
        g_au8MstTxData[1] = (uint8_t)((i & 0xFF00) >> 8);
        g_au8MstTxData[2] = (uint8_t)(i & 0x00FF);
        g_au8MstTxData[3] = (uint8_t)(g_au8MstTxData[2] + 3);

        g_u8MstDataLen = 0;
        g_u8MstEndFlag = 0;

        /* I2C function to write data to slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_10bit_LB_MasterTx;

        /* I2C as master sends START signal */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

        /* Wait I2C Tx Finish */
        while((g_u8MstEndFlag == 0)&&(g_u8SLARetryFail==0));
        g_u8MstEndFlag = 0;

        if(g_u8SLARetryFail>0)
        {
            g_u8SLARetryFail = 0;
            return -1;
        }
        g_u8MstDataLen = 0;
        /* I2C function to read data from slave */
        s_I2C0HandlerFn = (I2C_FUNC)I2C_10bit_LB_MasterRx;

        g_u8MstDataLen = 0;
        g_u8DeviceAddr = u16slvaddr;

        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

        /* Wait I2C Rx Finish */
        u32TimeOutCnt = I2C_TIMEOUT;
        while(g_u8MstEndFlag == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for I2C Rx finish time-out!\n");
                return -1;
            }
        }

        /* Compare data */
        if(g_u8MstRxData != g_au8MstTxData[3])
        {
            printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
            return -1;
        }
    }
    printf("Master Access Slave (0x%X) Test OK\n", u16slvaddr);
    return 0;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for loopback test              |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master, and I2C1 as a slave.\n");
    printf("The I/O connection I2C0 to I2C1:\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PA.5)\n");
    printf("I2C1_SDA(PA.6), I2C1_SCL(PA.7)\n\n");

    I2CM_Init_10bit(I2C0, 100000);
    I2CS_Init_10bit(I2C1, 100000);

    /* Init I2C0 */
    I2C0_Init();

    /* Init I2C1 */
    I2C1_Init();



    /* I2C1 enter non address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

    for(i = 0; i < 0x100; i++)
    {
        g_au8SlvData[i] = 0;
    }

    /* I2C1 function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_10bit_LB_SlaveTRx;

    printf("\n");
    printf("I2C1 Slave Mode is Running.\n");

    /* Master Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
    I2C_10bit_Read_Write_SLAVE(I2C0, I2C1, 0x116);
    I2C_10bit_Read_Write_SLAVE(I2C0, I2C1, 0x336);
    printf("SLAVE Address test OK.\n");

    /* Master Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
    I2C_10bit_Read_Write_SLAVE(I2C0, I2C1, 0x116 & ~0x04);
    I2C_10bit_Read_Write_SLAVE(I2C0, I2C1, 0x336 & ~0x02);
    printf("SLAVE Address Mask test OK.\n");

    s_I2C0HandlerFn = NULL;
    s_I2C1HandlerFn = NULL;

    /* Close I2C0,1 */
    I2C0_Close();
    I2C1_Close();

    while(1);
}
