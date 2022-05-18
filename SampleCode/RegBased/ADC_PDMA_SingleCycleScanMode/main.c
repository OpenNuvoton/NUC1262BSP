/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 3 $
 * $Date: 17/05/04 1:13p $
 * @brief    Perform A/D Conversion with ADC single cycle scan mode and transfer result by PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define PDMA_CH 2
#define ADC_TEST_COUNT 32

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#ifdef __ICCARM__
#pragma data_alignment=4
uint32_t g_au32RxPDMADestination[ADC_TEST_COUNT];
#else
__attribute__((aligned(4))) uint32_t g_au32RxPDMADestination[ADC_TEST_COUNT];
#endif
uint32_t au32AdcData[ADC_TEST_COUNT];

volatile uint32_t g_u32PdmaTDoneInt;
volatile uint32_t g_u32PdmaTAbortInt;

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void AdcSingleCycleScanModePDMATest(void);


void PDMA_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS();
    uint32_t u32TDStatus = PDMA_GET_TD_STS();

    if(u32Status & PDMA_INTSTS_TDIF_Msk)
    {
        if(u32TDStatus & PDMA_TDSTS_TDIF0_Msk) /* CH0 */
        {
            g_u32PdmaTDoneInt = 1;
            PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF0_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF1_Msk) /* CH1 */
        {
            g_u32PdmaTDoneInt = 2;
            PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF1_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF2_Msk) /* CH2 */
        {
            g_u32PdmaTDoneInt = 3;
            PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF2_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF3_Msk) /* CH3 */
        {
            g_u32PdmaTDoneInt = 4;
            PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF3_Msk);
        }
        else if(u32TDStatus & PDMA_TDSTS_TDIF4_Msk) /* CH4 */
        {
            g_u32PdmaTDoneInt = 5;
            PDMA_CLR_TD_FLAG(PDMA_TDSTS_TDIF4_Msk);
        }
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

    /* Enable ADC module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_ADCCKEN_Msk;

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
/* Function: PDMA_Init                                                                                     */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Configure PDMA channel for ADC continuous scan mode test.                                             */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_Init()
{
    /* Enable PDMA module clock */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Open Channel 2 */
    PDMA->CHCTL |= (1 << PDMA_CH);

    /* Transfer configuration of Channel 2 */
    PDMA->DSCT[PDMA_CH].CTL = \
                              ((ADC_TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is ADC_TEST_COUNT */ \
                              PDMA_WIDTH_32 |  /* Transfer width is 32 bits(one word) */ \
                              PDMA_SAR_FIX |   /* Source increment size is no increment(fixed) */ \
                              PDMA_DAR_INC |   /* Destination increment size is 32 bits(one word) */ \
                              PDMA_REQ_SINGLE | /* Transfer type is burst transfer type */ \
                              PDMA_BURST_1 |   /* Burst size is 1. No effect in single transfer type */ \
                              PDMA_OP_BASIC;   /* Operation mode is basic mode */

    /* Configure source address */
    PDMA->DSCT[PDMA_CH].SA = (uint32_t)&ADC->ADPDMA;

    /* Configure destination address */
    PDMA->DSCT[PDMA_CH].DA = (uint32_t)&g_au32RxPDMADestination;

    /* Configure PDMA channel 2 as memory to memory transfer */
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & ~PDMA_REQSEL0_3_REQSRC0_Pos) | (PDMA_ADC_RX << PDMA_REQSEL0_3_REQSRC2_Pos);

    /* Enable transfer done interrupt */
    PDMA->INTEN |= (1 << PDMA_CH);

    /* Enable PDMA IRQ */
    NVIC_EnableIRQ(PDMA_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: PDMA_ConfigReload                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test.           */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA_ConfigReload()
{
    /* Transfer configuration of Channel 2 */
    PDMA->DSCT[PDMA_CH].CTL |= \
                               ((ADC_TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | /* Transfer count is ADC_TEST_COUNT */ \
                               PDMA_OP_BASIC;   /* Operation mode is basic mode */
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcSingleCycleScanModePDMATest                                                                */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC single cycle scan mode with PDMA test.                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void AdcSingleCycleScanModePDMATest()
{
    uint8_t  u8Option;
    uint32_t u32DataCount;
    uint32_t u32ErrorCount;
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                 ADC single cycle scan mode with PDMA sample code     |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Init PDMA channel to transfer ADC conversion results */
    PDMA_Init();

    while(1)
    {
        printf("\n\nSelect input mode:\n");
        printf("  [1] Single end input (channel 0, 1, 2 and 3)\n");
        printf("  [2] Differential input (input channel pair 0 and 1)\n");
        printf("  Other keys: exit single cycle scan mode with PDMA test\n");
        u8Option = getchar();
        if(u8Option == '1')
        {
            /* Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test. */
            PDMA_ConfigReload();

            /* Set the ADC operation mode as single-cycle, input mode as single-end and enable the ADC converter  */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) | \
                (ADC_ADCR_ADMD_SINGLE_CYCLE | ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADEN_CONVERTER_ENABLE);

            /* Enable PDMA transfer */
            ADC_ENABLE_PDMA(ADC);

            /* Enable analog input channel 0, 1, 2 and 3 */
            ADC->ADCHER = ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0xF));

            /* Clear destination buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
                g_au32RxPDMADestination[u32DataCount] = 0;

            u32DataCount = 0;
            u32ErrorCount = 0;
            g_u32PdmaTDoneInt = 0;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            while(1)
            {
                uint32_t u32Ch;
                if(ADC->ADSR0 & ADC_ADSR0_ADF_Msk)
                {
                    /* Clear the ADC interrupt flag */
                    ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

                    for(u32Ch = 0; u32Ch < 4; u32Ch++)
                    {
                        au32AdcData[u32DataCount++] = ADC_GET_CONVERSION_DATA(ADC, u32Ch);
                        if(u32DataCount >= ADC_TEST_COUNT)
                            break;
                    }
                    if(u32DataCount < ADC_TEST_COUNT)
                        ADC->ADCR |= ADC_ADCR_ADST_Msk; /* Start A/D conversion */
                    else
                        break;
                }
            }

            /* Wait for PDMA transfer done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32PdmaTDoneInt == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PDMA transfer done time-out!\n");
                    return;
                }
            }

            /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
            {
                if(au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF))
                {
                    printf("*** Count %d, conversion result: 0x%X, PDMA result: 0x%X.\n",
                           u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
                    u32ErrorCount++;
                }
            }

            if(u32ErrorCount == 0)
                printf("\n    PASS!\n");
            else
                printf("\n    FAIL!\n");
        }
        else if(u8Option == '2')
        {
            /* Reload transfer count and operation mode of PDMA channel for ADC continuous scan mode test. */
            PDMA_ConfigReload();

            /* Set the ADC operation mode as single-cycle, input mode as differential and enable the ADC converter */
            ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) | \
                (ADC_ADCR_ADMD_SINGLE_CYCLE | ADC_ADCR_DIFFEN_DIFFERENTIAL | ADC_ADCR_ADEN_CONVERTER_ENABLE);

            /* Enable PDMA transfer */
            ADC_ENABLE_PDMA(ADC);

            /* Enable analog input channel 0 and 2 */
            ADC->ADCHER = ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x5));

            /* Clear destination buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
                g_au32RxPDMADestination[u32DataCount] = 0;

            u32DataCount = 0;
            u32ErrorCount = 0;
            g_u32PdmaTDoneInt = 0;

            /* Clear the A/D interrupt flag for safe */
            ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

            /* Start A/D conversion */
            ADC->ADCR |= ADC_ADCR_ADST_Msk;

            while(1)
            {
                uint32_t u32Ch;
                if(ADC->ADSR0 & ADC_ADSR0_ADF_Msk)
                {
                    /* Clear the ADC interrupt flag */
                    ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

                    for(u32Ch = 0; u32Ch < 4; u32Ch += 2)
                    {
                        au32AdcData[u32DataCount++] = ADC_GET_CONVERSION_DATA(ADC, u32Ch);
                        if(u32DataCount >= ADC_TEST_COUNT)
                            break;
                    }
                    if(u32DataCount < ADC_TEST_COUNT)
                        ADC->ADCR |= ADC_ADCR_ADST_Msk; /* Start A/D conversion */
                    else
                        break;
                }
            }

            /* Wait for PDMA transfer done */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32PdmaTDoneInt == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PDMA transfer done time-out!\n");
                    return;
                }
            }

            /* Compare the log of ADC conversion data register with the content of PDMA target buffer */
            for(u32DataCount = 0; u32DataCount < ADC_TEST_COUNT; u32DataCount++)
            {
                if(au32AdcData[u32DataCount] != (g_au32RxPDMADestination[u32DataCount] & 0xFFF))
                {
                    printf("*** Count %d, conversion result: 0x%X, PDMA result: 0x%X.\n",
                           u32DataCount, au32AdcData[u32DataCount], g_au32RxPDMADestination[u32DataCount]);
                    u32ErrorCount++;
                }
            }

            if(u32ErrorCount == 0)
                printf("\n    PASS!\n");
            else
                printf("\n    FAIL!\n");
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

    /* Single cycle scan mode with PDMA test */
    AdcSingleCycleScanModePDMATest();

    /* Reset ADC module */
    SYS->IPRST1 |= (1 << SYS_IPRST1_ADCRST_Pos) ;
    SYS->IPRST1 &= ~(1 << (SYS_IPRST1_ADCRST_Pos)) ;

    /* Disable ADC IP clock */
    CLK->APBCLK0 &= ~CLK_APBCLK0_ADCCKEN_Msk;

    /* Disable PDMA module clock */
    CLK->AHBCLK &= ~CLK_AHBCLK_PDMACKEN_Msk;

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_DisableIRQ(PDMA_IRQn);

    printf("Exit ADC sample code\n");

    while(1);

}
