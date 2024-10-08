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
    if(GPIO_GET_INT_FLAG(PB, BIT3))
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
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
    if(GPIO_GET_INT_FLAG(PC, BIT4))
    {
        GPIO_CLR_INT_FLAG(PC, BIT4);
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

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
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
    GPIO_SetMode(PB, BIT3, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPAB_IRQn);

    /*  Configure PC.4 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PC, BIT4, GPIO_MODE_QUASI);
    GPIO_EnableInt(PC, 4, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPCDF_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT3);
    GPIO_ENABLE_DEBOUNCE(PC, BIT4);

    /* Waiting for interrupts */
    while(1);
}
