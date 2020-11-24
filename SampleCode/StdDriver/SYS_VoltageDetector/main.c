/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use voltage detector to detect pin input voltage.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"




/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Check if voltage detector interrupt happen */
    if( SYS->BODCTL & SYS_BODCTL_VDETIF_Msk )
    {
        /* Clear voltage detector interrupt flag */
        SYS->BODCTL |= SYS_BODCTL_VDETIF_Msk;

        if(SYS->BODCTL & SYS_BODCTL_VDETOUT_Msk)
            printf("The input voltage is lower than Bandgap.\n");
        else
            printf("The input voltage is higher than Bandgap.\n");
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

    /* Set PB multi-function pins for VDET_P0(PB.0) */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_VDET_P0;

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
/*  Main Function                                                                                          */
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
    
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|          Voltage Detector Sample Code          |\n");
    printf("+------------------------------------------------+\n");

    printf("Change VDET_P0(PB.0) input voltage.\n");
    printf("The voltage detector interrupt is requested when the input voltage \nis dropped down or raised up through the Bandgap voltage(1.2V).\n\n");
    UART_WAIT_TX_EMPTY(DEBUG_PORT);    
    
    /* Select voltage detector external input voltage pin as VDET_P0(PB.0) */
    SYS->BODCTL &= ~SYS_BODCTL_VDETPINSEL_Msk;           

    /* Enable voltage detector function */
    SYS->BODCTL |= SYS_BODCTL_VDETEN_Msk;   

    /* Enable voltage detector interrupt function */
    SYS->BODCTL |= SYS_BODCTL_VDETIEN_Msk;
    NVIC_EnableIRQ(BOD_IRQn);    

    /* Wait for voltage detector interrupt happen */
    while(1);

}
