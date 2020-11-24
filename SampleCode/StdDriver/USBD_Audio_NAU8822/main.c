/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief
 *           This sample code demonstrates how to implement a USB audio class device.
 *           NAU8822 is used in this sample code to play the audio data from Host.
 *           It also supports to record data from NAU8822 to Host.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

#define HIRC_AUTO_TRIM      0x411   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x118)

void EnableCLKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
    /* CLKO = clock source / 2^(u32ClkDiv + 1) */
    CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | u32ClkDiv;

    /* Enable CLKO clock source */
    CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

    /* Select CLKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | u32ClkSrc;
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

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select USB clock source as HIRC and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC, CLK_CLKDIV0_USB(1));

    /* Enable TMR0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TMR0 clock source as HIRC/2 */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC_DIV2, 0);

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select SPI0 clock source as PLL/2 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PLL_DIV2, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PB multi-function pins for CLKO */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB14MFP_Msk);
    SYS->GPB_MFPH |= SYS_GPB_MFPH_PB14MFP_CLKO;

    /* Set GPC0, GPC1 to be I2C */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk);
    SYS->GPC_MFPL |= SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL;

    /* Set I2S interface */
    /* GPB[0] : SPI0_I2SMCLK. */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_SPI0_I2SMCLK;

    /* GPA[3:0] : SPI0_SS (I2S_LRCLK), SPI0_CLK (I2S_BCLK), SPI0_MISO (I2S_DI), SPI0_MOSI (I2S_DO). */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA0MFP_Msk | SYS_GPA_MFPL_PA1MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk | SYS_GPA_MFPL_PA3MFP_Msk);
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;
    PA->SLEWCTL |= 0xF;

    /* Enable CLKO(PB14) for monitor HCLK. CLKO = HCLK/8 Hz */
    EnableCLKO((2 << CLK_CLKSEL2_CLKOSEL_Pos), 2);
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

void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100 kHz */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i;
    uint32_t u32TrimInit;

    /*
        This sample code is used to demo USB Audio Class + NAU8822.
        User can define PLAY_RATE in usbd_audio.h to support 48000Hz.

        The audio is input from NAU8822 AUXIN.
        The audio is output by NAU8822 Headphone output.

        NAU8822 is connect with I2S(PA0~3) and controlled by I2C0(PC0, PC1).
        NAU8822 clock source is also come from I2S (MCLK, PC5).

            PC0 <-> I2S0_SDA
            PC1 <-> I2S0_SCL

            PA0 <-> I2S0_DAC
            PA1 <-> I2S0_ADC
            PA2 <-> I2S0_BCLK
            PA3 <-> I2S0_LRCLK

            PB0 <-> I2S_MCLK

        PB14 is used to output clock (HCLK/8) to check HCLK frequency.

        Clock config (I2S Slave Mode):
            PLL = 144,000,000 Hz
            CPU = 72,000,000 Hz
            I2S Clock Src = PLL/2

            MCLK = 12,000,000 Hz
            Sample Rate = (Based on Codec. 48000 Hz)

        USB Clock Src = HIRC (48MHz)

    */

    /* Unlock Protected Regsiter */
    SYS_UnlockReg();

    /* Initial system & multi-function */
    SYS_Init();

    /* Initial UART0 for debug message */
    UART0_Init();

    /* Init I2C0 to access WAU8822 */
    I2C0_Init();

    /* Initialize WAU8822 codec */
    WAU8822_Setup();

    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, PLAY_RATE, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
    /* SPII2S driver will overwrite SPI clock source setting. Just re-set it here */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PLL_DIV2, 0);

    /* Set MCLK and enable MCLK */
    SPII2S_EnableMCLK(SPI0, 12000000);

    /* I2S clock divider is set again to fix BCLK/MCLK phase delay.
       Wrong phase delay may cause noise in codec.
    */
    {
        uint32_t u32Reg;

        u32Reg = SPI0->I2SCLK;
        SPI0->I2SCLK = 0;
        __NOP();
        __NOP();
        __NOP();
        SPI0->I2SCLK = u32Reg;
    }

    /* Fill dummy data to I2S Tx for start I2S iteration */
    for(i = 0; i < 4; i++)
        SPII2S_WRITE_TX_FIFO(SPI0, 0);

    /* Start I2S play iteration */
    SPII2S_EnableInt(SPI0, SPII2S_FIFO_TXTH_INT_MASK | SPII2S_FIFO_RXTH_INT_MASK);

    /* Open USB controller */
    USBD_Open(&gsInfo, UAC_ClassRequest, (SET_INTERFACE_REQ)UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();

    /* Start USB device */
    USBD_Start();

    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);

    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    NVIC_EnableIRQ(SPI0_IRQn);

    while(1)
    {
        uint8_t ch;
        uint32_t u32Reg, u32Data;
        extern int32_t kbhit(void);

        /* Start USB trim if it is not enabled. */
        if((SYS->IRCTCTL & SYS_IRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->IRCTCTL = HIRC_AUTO_TRIM;
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


        /* Adjust codec sampling rate to synch with USB. The adjustment range is +-0.005% */
        AdjFreq();

        /* Set audio volume according USB volume control settings */
        VolumnControl();

        /* User can change audio codec settings by I2C at run-time if necessary */
        if(!kbhit())
        {
            ch = getchar();
            if(ch == 'a')
            {
                if((SPI0->I2SCLK & 0x3f) == 5)
                    SPI0->I2SCLK = (SPI0->I2SCLK & (~0x3f)) | 6;
                else
                    SPI0->I2SCLK = (SPI0->I2SCLK & (~0x3f)) | 5;
            }
            else
            {


                printf("\nEnter codec setting:\n");
                // Get Register number
                ch = getchar();
                u32Reg = ch - '0';
                ch = getchar();
                u32Reg = u32Reg * 10 + (ch - '0');
                printf("%d\n", u32Reg);

                // Get data
                ch = getchar();
                u32Data = (ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10;
                ch = getchar();
                u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
                ch = getchar();
                u32Data = u32Data * 16 + ((ch >= '0' && ch <= '9') ? ch - '0' : ch - 'a' + 10);
                printf("%03x\n", u32Data);
                I2C_WriteWAU8822(u32Reg,  u32Data);
            }
        }

    }
}
