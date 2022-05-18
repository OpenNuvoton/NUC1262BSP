/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Demonstrate how to implement a USB mouse device.
 *           It use PC0 ~ PC5 to control mouse direction and mouse key.
 *           It also supports USB suspend and remote wakeup.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_mouse.h"

/* If crystal-less is enabled, system won't use any crystal as clock source
   If using crystal-less, system will be 48MHz, otherwise, system is 72MHz
*/
#define CRYSTAL_LESS        1
#define HIRC_AUTO_TRIM      0x611   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x110)

uint8_t volatile g_u8RemouteWakeup = 0;
int IsDebugFifoEmpty(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

#if (!CRYSTAL_LESS)
    /* Enable GPIO Port F module clock */
    CLK_EnableModuleClock(GPIOF_MODULE);

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
    GPIO_SetMode(PF, BIT2|BIT3, GPIO_MODE_INPUT);

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Select USB clock source as PLL and USB clock divider as 3 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));
#else
    /* Select HCLK clock source to HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Select USB clock source as HIRC and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));
#endif

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

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
    /* Reset IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC >> 1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void GPIO_Init(void)
{
    /* Enable PC0~5 interrupt for wakeup */

    PC->MODE = 0x0fff; /* PC0~5 be Quasi mode */
    PC->INTSRC |= 0x3f;
    PC->INTEN |= 0x3f | (0x3f << 16);
    PC->DBEN |= 0x3f;      // Enable key debounce
    GPIO->DBCTL = 0x16; // Debounce time is about 6ms
    NVIC_EnableIRQ(GPCDF_IRQn);
}


void GPCDF_IRQHandler(void)
{
    PC->INTSRC = 0x3f;
    g_u8RemouteWakeup = 1;
    //PB4 ^= 1;

}

void PowerDown()
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Enter power down ...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!IsDebugFifoEmpty())
        if(--u32TimeOutCnt == 0) break;

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Note HOST to resume USB tree if it is suspended and remote wakeup enabled */
    if(g_usbd_RemoteWakeupEn && g_u8RemouteWakeup)
    {
        /* Enable PHY before sending Resume('K') state */
        USBD->ATTR |= USBD_ATTR_PHYEN_Msk;

        /* Keep remote wakeup for 1 ms */
        USBD->ATTR |= USBD_ATTR_RWAKEUP_Msk;
        CLK_SysTickDelay(1000); /* Delay 1ms */
        USBD->ATTR ^= USBD_ATTR_RWAKEUP_Msk;
        g_u8RemouteWakeup = 0;
        printf("Remote Wakeup!!\n");
    }

    printf("device wakeup!\n");

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system and multi-funcition I/O */
    SYS_Init();

    /* Init UART for debug message */
    UART0_Init();

    GPIO_Init();

    printf("\n");
    printf("+-----------------------------------------------------+\n");
    printf("|          NuMicro USB HID Mouse Sample Code          |\n");
    printf("+-----------------------------------------------------+\n");

    /* This sample code is used to simulate a mouse with suspend and remote wakeup supported.
       User can use PC0~PC5 key to control the moviement of mouse.
       PB4 is used as a LED to show working status.
    */

    /* Open USB controller */
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();

    /* Start USB device */
    USBD_Start();

    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    PB4 = 0; // LED to show system is on line

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->IRCTCTL & SYS_IRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->IRCTCTL = HIRC_AUTO_TRIM | (8 << SYS_IRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->IRCTCTL = 0;

            /* Clear error flags */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk | SYS_IRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        /* Enter power down when USB suspend */
        if(g_u8Suspend)
        {
            PowerDown();

            /* Waiting for key release */
            while((GPIO_GET_IN_DATA(PC) & 0x3F) != 0x3F);
        }

        HID_UpdateMouseData();
    }
}
