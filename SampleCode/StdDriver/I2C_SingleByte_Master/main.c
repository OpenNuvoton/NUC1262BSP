/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 20/11/27 $
 * @brief
 *           Show how to use I2C Signle byte API Read and Write data to Slave
 *           Needs to work with I2C_Slave sample code.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;

/*---------------------------------------------------------------------------------------------------------*/
/* System Initial                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
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

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Set PA/PB multi-function pins for I2C0 SDA and SCL */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB4MFP_I2C0_SDA);

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA5MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA5MFP_I2C0_SCL);

    /* I2C pins enable schmitt trigger */
    PB->SMTEN |= GPIO_SMTEN_SMTEN4_Msk;
    PA->SMTEN |= GPIO_SMTEN_SMTEN5_Msk;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x15, 0);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */

    /* Set I2C0 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C0, 0, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 1, 0x04);
    I2C_SetSlaveAddrMask(I2C0, 2, 0x01);
    I2C_SetSlaveAddrMask(I2C0, 3, 0x04);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t i;
    uint8_t u8data, u8tmp, err;

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
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for Single Byte Read/Write Test |\n");
    printf("| Needs to work with I2C_Slave sample code               |\n");
    printf("|                                                        |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                |\n");
    printf("| !! This sample code requires two borads to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");
    printf("Configure I2C0 as Master\n");
    printf("The I/O connection to I2C0\n");
    printf("I2C0_SDA(PB.4), I2C0_SCL(PA.5)\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Slave Address */
    g_u8DeviceAddr = 0x15;

    err = 0;

    for(i = 0; i < 256; i++)
    {
        u8tmp = (uint8_t)i + 3;

        /* Single Byte Write (Two Registers) */
        while(I2C_WriteByteTwoRegs(I2C0, g_u8DeviceAddr, i, u8tmp));

        /* Single Byte Read (Two Registers) */
        u8data = I2C_ReadByteTwoRegs(I2C0, g_u8DeviceAddr, i);
        if(u8data != u8tmp)
        {
            err = 1;
            printf("%03d: Single byte write data fail,  W(0x%X)/R(0x%X) \n", i, u8tmp, u8data);
        }
    }

    printf("\n");

    if(err)
        printf("Single byte Read/Write access Fail.....\n");
    else
        printf("Single byte Read/Write access Pass.....\n");

    while(1);
}



