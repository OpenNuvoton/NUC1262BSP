/**************************************************************************//**
 * @file     main_LD.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 20/11/27 $
 * @brief    Show how to call LDROM functions from APROM.
 *           The code in APROM will look up the table at 0x100E00 to get the address of function of LDROM and call the function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define KEY_ADDR            0x20000FFC  /* The location of signature */
#define SIGNATURE           0x21557899  /* The signature word is used by AP code to check if simple LD is finished */

#define CONFIG0_TEST_CODE   0x0F9000FF

#define FUN_TBL_BASE        0x00100E00

int32_t IAP_Func0(int32_t n);
int32_t IAP_Func1(int32_t n);
int32_t IAP_Func2(int32_t n);
int32_t IAP_Func3(int32_t n);

#if defined ( __ICCARM__ )
# pragma location = "FunTblSection" /* The location of FunTblSection is defined in FMC_IAP_LD.icf file. */
__root const uint32_t g_funcTable[4] =
{
    (uint32_t)IAP_Func0, (uint32_t)IAP_Func1, (uint32_t)IAP_Func2, (uint32_t)IAP_Func3
} ;
#else
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
const uint32_t __attribute__((section (".IAPFunTable"))) g_funcTable[4] =
{
    (uint32_t)IAP_Func0, (uint32_t)IAP_Func1, (uint32_t)IAP_Func2, (uint32_t)IAP_Func3
};
#else
const uint32_t * __attribute__((section(".ARM.__at_0x00100E00"))) g_funcTable[4] =
{
    (uint32_t *)IAP_Func0, (uint32_t *)IAP_Func1, (uint32_t *)IAP_Func2, (uint32_t *)IAP_Func3
};
#endif
#endif


void SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC >> 1, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

int32_t IAP_Func0(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else
	int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP0! #%d\n", i);
    }

    return n;
#endif
}

int32_t IAP_Func1(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else 
	int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP1! #%d\n", i);
    }

    return n;
#endif
}
int32_t IAP_Func2(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP2! #%d\n", i);
    }

    return n;
#endif
}
int32_t IAP_Func3(int32_t n)
{
#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
    return (n * 1);
#else
    int32_t i;

    for(i = 0; i < n; i++)
    {
        printf("Hello IAP3! #%d\n", i);
    }

    return n;
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))

    // Delay 3 seconds
    for(i = 0; i < 30; i++)
    {
        SysTickDelay(10000);
    }

    while(SYS->PDID)__WFI();
#else
    /* Init UART0 for printf */
    UART0_Init();

    /*
        This is a simple sample code for LDROM in new IAP mode.
        The base address is 0x100000.
        The base address for function table is defined by FUN_TBL_BASE.
    */

    printf("+------------------------------------------------------------------+\n");
    printf("|        Flash Memory Controller Driver Sample Code for LDROM      |\n");
    printf("+------------------------------------------------------------------+\n");

    printf("\nCPU @ %dHz\n\n", SystemCoreClock);

    // Delay 3 seconds
    for(i = 0; i < 30; i++)
    {
        printf(".");
        SysTickDelay(10000);
    }
    printf("\n");

    printf("Function table @ 0x%08x\n", (uint32_t) g_funcTable);

    while(SYS->PDID)__WFI();
#endif
}
