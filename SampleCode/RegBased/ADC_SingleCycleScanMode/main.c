/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 3 $
 * $Date: 17/05/04 1:13p $
 * @brief    Perform A/D Conversion with ADC single cycle scan mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcSingleCycleScanModeTest(void);


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

    /* Enable ADC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_ADCCKEN_Msk ;

    /* Select ADC module clock source as HIRC/2 and ADC module clock divider as 7 */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_ADCSEL_Msk)) | CLK_CLKSEL1_ADCSEL_HIRC_DIV2;
    CLK->CLKDIV0  = (CLK->CLKDIV0 & (~CLK_CLKDIV0_ADCDIV_Msk)) | CLK_CLKDIV0_ADC(7);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Disable digital input path of ADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, 0xF);

    /* Set multi-function pins for ADC channels */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB2MFP_ADC0_CH2 | SYS_GPB_MFPL_PB3MFP_ADC0_CH3;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC>>1, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcSingleCycleScanModeTest                                                                    */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC single cycle scan mode test.                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void AdcSingleCycleScanModeTest()
{
    uint8_t  u8Option;
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC single cycle scan mode sample code               |\n");
    printf("+----------------------------------------------------------------------+\n");

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit single cycle scan mode test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Set the ADC operation mode as single-cycle, input mode as single-end and enable the ADC converter  */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) | \
                (ADC_ADCR_ADMD_SINGLE_CYCLE | ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADEN_CONVERTER_ENABLE);

            /* Enable analog input channel 0, 1, 2 and 3 */
            ADC->ADCHER = ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0xF));

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait conversion done */
            while(!((ADC->ADSR0 & ADC_ADSR0_ADF_Msk) >> ADC_ADSR0_ADF_Pos));

            for(u32ChannelCount = 0; u32ChannelCount < 4; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[(u32ChannelCount)] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;
                printf("Conversion result of channel %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        }
        else if(u8Option == '2')
        {
            /* Set the ADC operation mode as single-cycle, input mode as differential and enable the ADC converter */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) | \
                (ADC_ADCR_ADMD_SINGLE_CYCLE | ADC_ADCR_DIFFEN_DIFFERENTIAL | ADC_ADCR_ADEN_CONVERTER_ENABLE);

            /* Enable analog input channel 0 and 2 */
            ADC->ADCHER = ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x5));

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            /* Wait conversion done */
            while(!((ADC->ADSR0 & ADC_ADSR0_ADF_Msk) >> ADC_ADSR0_ADF_Pos));

            for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
            {
                i32ConversionData = (ADC->ADDR[(u32ChannelCount * 2)] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;
                printf("Conversion result of differential input pair %d: 0x%X (%d)\n", u32ChannelCount, i32ConversionData, i32ConversionData);
            }
        }
        else
            return ;

    }
}




/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Single cycle scan mode test */
    AdcSingleCycleScanModeTest();

    /* Reset ADC module */
    SYS->IPRST1 |= (1 << SYS_IPRST1_ADCRST_Pos) ;
    SYS->IPRST1 &= ~(1 << (SYS_IPRST1_ADCRST_Pos)) ;

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_ADCCKEN_Msk;

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    while(1);

}
