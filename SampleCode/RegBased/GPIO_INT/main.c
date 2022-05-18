/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO interrupt function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"





/**
 * @brief       PortA/PortB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortA/PortB default IRQ, declared in startup_NUC1262.s.
 */
void GPAB_IRQHandler(void)
{
    uint32_t u32PAINTSRC, u32PBINTSRC;

    /* To check if PB.3 interrupt occurred */
    if(PB->INTSRC & BIT3)
    {
        PB->INTSRC = BIT3;
        printf("PB.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTA, PORTB interrupts */
        u32PAINTSRC = PA->INTSRC;
        PA->INTSRC = u32PAINTSRC;
        u32PBINTSRC = PB->INTSRC;
        PB->INTSRC = u32PBINTSRC;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       PortC/PortD/PortF IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortC/PortD/PortF default IRQ, declared in startup_NUC1262.s.
 */
void GPCDF_IRQHandler(void)
{
    uint32_t u32PCINTSRC, u32PDINTSRC, u32PFINTSRC;

    /* To check if PC.4 interrupt occurred */
    if(PC->INTSRC & BIT4)
    {
        PC->INTSRC = BIT4;
        printf("PC.4 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTC, PORTD and PORTF interrupts */
        u32PCINTSRC = PC->INTSRC;
        PC->INTSRC = u32PCINTSRC;
        u32PDINTSRC = PD->INTSRC;
        PD->INTSRC = u32PDINTSRC;
        u32PFINTSRC = PF->INTSRC;
        PF->INTSRC = u32PFINTSRC;
        printf("Un-expected interrupts.\n");
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
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC>>1), 115200);
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
    printf("+------------------------------------------------+\n");
    printf("|    GPIO PB.3 and PC.4 Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("PB.3 and PC.4 are used to test interrupt ......\n");

    /* Configure PB.3 as Input mode and enable interrupt by rising edge trigger */
    PB->MODE = (PB->MODE & (~GPIO_MODE_MODE3_Msk)) | (GPIO_MODE_INPUT << GPIO_MODE_MODE3_Pos);
    PB->INTTYPE |= (GPIO_INTTYPE_EDGE << GPIO_INTTYPE_TYPE3_Pos);
    PB->INTEN |= GPIO_INTEN_RHIEN3_Msk;
    NVIC_EnableIRQ(GPAB_IRQn);

    /*  Configure PC.4 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    PC->MODE = (PC->MODE & (~GPIO_MODE_MODE4_Msk)) | (GPIO_MODE_QUASI << GPIO_MODE_MODE4_Pos);
    PC->INTTYPE |= (GPIO_INTTYPE_EDGE << GPIO_INTTYPE_TYPE4_Pos);
    PC->INTEN |= GPIO_INTEN_FLIEN4_Msk;
    NVIC_EnableIRQ(GPCDF_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO->DBCTL = (GPIO_DBCTL_ICLK_ON | GPIO_DBCTL_DBCLKSRC_LIRC | GPIO_DBCTL_DBCLKSEL_1024);
    PB->DBEN |= BIT3;
    PC->DBEN |= BIT4;

    /* Waiting for interrupts */
    while(1);
}
