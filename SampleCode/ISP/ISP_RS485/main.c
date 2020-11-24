/**************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to update chip flash data through RS485 interface
 *           between chip RS485 and ISP Tool.
 *           Nuvoton NuMicro ISP Programming Tool is also required in this
 *           sample code to connect with chip RS485 and assign update file
 *           of Flash.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "targetdev.h"
#include "uart_transfer.h"


#define nRTSPin                 (PA0)
#define REVEIVE_MODE            (0)
#define TRANSMIT_MODE           (1)


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void ProcessHardFault(void);
void SH_Return(void);


void ProcessHardFault(void){}
void SH_Return(void){}


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
    PllClock        = 144000000;
    SystemCoreClock = 144000000 / 2;
    CyclesPerUs     = SystemCoreClock / 1000000;  /* For CLK_SysTickDelay() */

    /* Enable UART1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART1CKEN_Msk;

    /* Select UART1 module clock source as HIRC/2 and UART1 module clock divider as 1 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_HIRC_DIV2;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | CLK_CLKDIV0_UART1(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PA multi-function pins for UART1 RXD(PA.2), TXD(PA.3) and RTS(PA.0) */
    PA->MODE = (PA->MODE & (~GPIO_MODE_MODE0_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE0_Pos);
    nRTSPin = REVEIVE_MODE;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART1_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_UART1_TXD;
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
    
    /* Configure WDT */
    WDT->CTL &= ~(WDT_CTL_WDTEN_Msk | WDT_CTL_ICEDEBUG_Msk);
    WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_CTL_RSTEN_Msk);
    
    /* Init UART */
    UART_Init();

    /* Enable FMC ISP */    
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
    
    /* Get APROM size, data flash size and address */    
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    
    /* Set Systick time-out for 300ms */     
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;   /* Use CPU clock */

    /* Wait for CMD_CONNECT command until Systick time-out */
    while (1) {
        
        /* Wait for CMD_CONNECT command */         
        if ((bufhead >= 4) || (bUartDataReady == TRUE)) {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT) {
                break;
            } else {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        /* Systick time-out, then go to APROM */
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

    /* Prase command from master and send response back */      
    while (1) {
        
        if (bUartDataReady == TRUE) {

            WDT->CTL &= ~(WDT_CTL_WDTEN_Msk | WDT_CTL_ICEDEBUG_Msk);
            WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_CTL_RSTEN_Msk);

            bUartDataReady = FALSE;;        /* Reset UART data ready flag */
            ParseCmd(uart_rcvbuf, 64);      /* Parse command from master */
            NVIC_DisableIRQ(UART_T_IRQn);   /* Disable NVIC */
            nRTSPin = TRANSMIT_MODE;        /* Control RTS in transmit mode */
            PutString();                    /* Send response to master */

            /* Wait for data transmission is finished */
            while ((UART_T->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

            nRTSPin = REVEIVE_MODE;         /* Control RTS in reveive mode */
            NVIC_EnableIRQ(UART_T_IRQn);    /* Enable NVIC */

        }

    }

_APROM:

    /* Reset system and boot from APROM */
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk); /* Clear reset status flag */
    FMC->ISPCTL = FMC->ISPCTL & 0xFFFFFFFC;
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
