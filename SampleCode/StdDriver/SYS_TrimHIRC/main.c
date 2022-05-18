/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use LXT to trim HIRC.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"




/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void TrimHIRC(void);
void CLKDIRC_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/*--------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
void CLKDIRC_IRQHandler(void)
{
    if(SYS->IRCTISTS & SYS_IRCTISTS_TFAILIF_Msk)   /* Get Trim Failure Interrupt */
    {
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_TFAILIF_Msk;
    }
    if(SYS->IRCTISTS & SYS_IRCTISTS_CLKERRIF_Msk)   /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");
        /* Clear LXT Clock Error Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk;
    }

}

void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF4MFP_Msk)) | SYS_GPF_MFPL_PF4MFP_X32_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF5MFP_Msk)) | SYS_GPF_MFPL_PF5MFP_X32_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable GPIO Port F module clock */
    CLK_EnableModuleClock(GPIOF_MODULE);

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
    GPIO_SetMode(PF, BIT2|BIT3, GPIO_MODE_INPUT);

    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HXT and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PB multi-function pins for CLKO(PB.14) */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB14MFP_Msk)) | SYS_GPB_MFPH_PB14MFP_CLKO;

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

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /* Enable Interrupt */
    NVIC_EnableIRQ(CLKDIRC_IRQn);

    /* Trim HIRC to 48MHz */
    TrimHIRC();

    /* Disable IRC Trim */
    SYS->IRCTCTL = 0;
    printf("Disable IRC Trim\n");

    while(1);

}

void TrimHIRC(void)
{
    uint32_t u32TimeOutCnt;

    /* Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->IRCTIEN |= (SYS_IRCTIEN_CLKEIEN_Msk | SYS_IRCTIEN_TFAILIEN_Msk);
    SYS->IRCTCTL = (SYS->IRCTCTL & (~SYS_IRCTCTL_FREQSEL_Msk)) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while( (SYS->IRCTISTS & SYS_IRCTISTS_FREQLOCK_Msk) == 0 )
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("HIRC Trim failed\n");
            return;
        }
    }

    printf("HIRC Frequency Lock\n");

    /* Clear Trim Lock */
    SYS->IRCTISTS = SYS_IRCTISTS_FREQLOCK_Msk;

    /* Enable CLKO and output frequency = HIRC/4 */
    CLK_EnableCKO(CLK_CLKSEL2_CLKOSEL_HIRC, 1, 0);
}
