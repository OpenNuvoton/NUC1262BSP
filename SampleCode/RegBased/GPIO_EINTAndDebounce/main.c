/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO external interrupt function and de-bounce function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



/**
 * @brief       External INT0/2/4 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0/2/4 default IRQ, declared in startup_NUC1262.s.
 */
void EINT024_IRQHandler(void)
{
    /* For PB.5, clear the INT flag */
    GPIO_CLR_INT_FLAG(PB, BIT5);

    printf("PB.5 EINT0 occurred.\n");
}

/**
 * @brief       External INT1/3/5 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1/3/5 default IRQ, declared in startup_NUC1262.s.
 */
void EINT135_IRQHandler(void)
{
    /* For PB.4, clear the INT flag */
    GPIO_CLR_INT_FLAG(PB, BIT4);

    printf("PB.4 EINT1 occurred.\n");
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

    /* Set PB multi-function pin for EINT0(PB.5) and EINT1(PB.4) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB5MFP_Msk)) | SYS_GPB_MFPL_PB5MFP_INT0;
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB4MFP_Msk)) | SYS_GPB_MFPL_PB4MFP_INT1;
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

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------------------+\n");
    printf("|    GPIO EINT0/EINT1 Interrupt and De-bounce Sample Code    |\n");
    printf("+------------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO External Interrupt Function Test                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("EINT0(PB.5) and EINT1(PB.4) are used to test interrupt \n");

    /* Configure PB.5 as EINT0 pin and enable interrupt by falling edge trigger */
    PB->MODE = (PB->MODE & (~GPIO_MODE_MODE5_Msk)) | (GPIO_MODE_INPUT << GPIO_MODE_MODE5_Pos);
    PB->INTTYPE |= (GPIO_INTTYPE_EDGE << GPIO_INTTYPE_TYPE5_Pos);
    PB->INTEN |= GPIO_INTEN_FLIEN5_Msk;
    NVIC_EnableIRQ(EINT024_IRQn);

    /* Configure PB.4 as EINT1 pin and enable interrupt by falling and rising edge trigger */
    PB->MODE = (PB->MODE & (~GPIO_MODE_MODE4_Msk)) | (GPIO_MODE_INPUT << GPIO_MODE_MODE4_Pos);
    PB->INTTYPE |= (GPIO_INTTYPE_EDGE << GPIO_INTTYPE_TYPE4_Pos);
    PB->INTEN |= (GPIO_INTEN_FLIEN4_Msk | GPIO_INTEN_RHIEN4_Msk);
    NVIC_EnableIRQ(EINT135_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO->DBCTL = (GPIO_DBCTL_ICLK_ON | GPIO_DBCTL_DBCLKSRC_LIRC | GPIO_DBCTL_DBCLKSEL_1024);
    PB->DBEN |= (BIT4 | BIT5);

    /* Waiting for interrupts */
    while(1);
}
