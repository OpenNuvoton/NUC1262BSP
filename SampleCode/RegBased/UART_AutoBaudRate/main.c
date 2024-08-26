/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use auto baud rate detection function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
void AutoBaudRate_Test(void);
void AutoBaudRate_TxTest(void);
void AutoBaudRate_RxTest(void);


void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;



    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
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

    /* Enable UART module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk);

    /* Select UART module clock source and UART module clock divider */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HIRC_DIV2;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_HIRC_DIV2;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | CLK_CLKDIV0_UART1(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PA multi-function pins for UART1 RXD(PA.2) and TXD(PA.3) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART1_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_UART1_TXD;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRST1 |=  SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;

    /* Configure UART1 and set UART1 baud rate */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for testing */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART auto baud rate sample function */
    AutoBaudRate_Test();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_Test(void)
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PA.3)  <==>  UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u32Item = (uint32_t)getchar();

    if(u32Item == '0')
        AutoBaudRate_TxTest();
    else
        AutoBaudRate_RxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_TxTest(void)
{
    uint32_t u32Item;

    do
    {

        printf("\n");
        printf("+-----------------------------------------------------------+\n");
        printf("|     Auto Baud Rate Function Test (Master)                 |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] baud rate 38400 bps                                   |\n");
        printf("| [2] baud rate 57600 bps                                   |\n");
        printf("| [3] baud rate 115200 bps                                  |\n");
        printf("|                                                           |\n");
        printf("| Select baud rate and master will send 0x1 to slave ...    |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| Quit                                              - [ESC] |\n");
        printf("+-----------------------------------------------------------+\n\n");
        u32Item = (uint32_t)getchar();
        if(u32Item == 27) break;
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch(u32Item)
        {
            case '1':
                UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 38400);
                break;
            case '2':
                UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 57600);
                break;
            default:
                UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
                break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        UART1->DAT = 0x1;

    }
    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UART Baud Rate Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUartBaudrate(UART_T* uart)
{
    uint8_t u8UartClkSrcSel, u8UartClkDivNum;
    uint32_t au32ClkTbl[4] = {__HXT, 0, __LXT, (__HIRC >> 1)};
    uint32_t u32Baud_Div;

    /* Get UART clock source selection and UART clock divider number */
    switch((uint32_t)uart)
    {
        case UART0_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
            break;

        case UART1_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
            break;

        default:
            return 0;
    }

    /* Get PLL clock frequency if UART clock source selection is PLL/2 */
    if(u8UartClkSrcSel == 1)
        au32ClkTbl[u8UartClkSrcSel] = (CLK_GetPLLClockFreq() >> 1);

    /* Get UART baud rate divider */
    u32Baud_Div = (uart->BAUD & UART_BAUD_BRD_Msk) >> UART_BAUD_BRD_Pos;

    /* Calculate UART baud rate if baud rate is set in MODE 0 */
    if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE0)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32Baud_Div + 2)) >> 4;

    /* Calculate UART baud rate if baud rate is set in MODE 2 */
    else if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE2)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32Baud_Div + 2));

    /* Calculate UART baud rate if baud rate is set in MODE 1 */
    else if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_BAUDM1_Msk)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32Baud_Div + 2)) / (((uart->BAUD & UART_BAUD_EDIVM1_Msk) >> UART_BAUD_EDIVM1_Pos) + 1);

    /* Unsupported baud rate setting */
    else
        return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_RxTest(void)
{

    /* Enable auto baud rate detect function */
    UART1->ALTCTL |= UART_ALTCTL_ABRDEN_Msk;

    printf("\nreceiving input pattern... ");

    /* Forces a write of all user-space buffered data for the given output */
    fflush(stdout);

    /* Wait until auto baud rate detect finished or time-out */
    while((UART1->ALTCTL & UART_ALTCTL_ABRIF_Msk) == 0);

    if(UART1->FIFOSTS & UART_FIFOSTS_ABRDIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDIF_Msk;
        printf("Baud rate is %dbps.\n", GetUartBaudrate(UART1));
    }
    else if(UART1->FIFOSTS & UART_FIFOSTS_ABRDTOIF_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDTOIF_Msk;
        printf("Time-out!\n");
    }

}
