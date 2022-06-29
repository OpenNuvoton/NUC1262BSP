/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 3 $
 * $Date: 17/05/04 1:13p $
 * @brief    Demonstrate how to trigger ADC by BPWM.
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
void ADC_PWMTrigTest_SingleOpMode(void);


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

    /* Enable BPWM0 and BPWM1 module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_BPWM0CKEN_Msk | CLK_APBCLK0_BPWM1CKEN_Msk;

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

    /* Set multi-function pins for BPWM0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;

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
/* Function: ADC_PWMTrigTest_SingleOpMode                                                                  */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC hardware trigger test.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_PWMTrigTest_SingleOpMode()
{
    uint32_t u32TimeOutCnt;

    printf("\n<<< BPWM trigger test (Single mode) >>>\n");

    /* PWM trigger; ADC single operation mode; single-end input; enable the ADC converter. */
    ADC->ADCR = (ADC->ADCR & (~(ADC_ADCR_TRGS_Msk | ADC_ADCR_TRGEN_Msk |ADC_ADCR_DIFFEN_Msk | ADC_ADCR_ADMD_Msk))) | \
        (ADC_ADCR_TRGS_BPWM | ADC_ADCR_TRGEN_ENABLE | ADC_ADCR_ADMD_SINGLE | ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADEN_CONVERTER_ENABLE);
    /* Enable analog input channel 2 */
    ADC->ADCHER |= ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (1 << 2));
    /* Clear the A/D interrupt flag for safe */
    ADC->ADSR0 = ADC_ADSR0_ADF_Msk;


    /* Center-aligned type */
    BPWM_SET_ALIGNED_TYPE(BPWM0, BPWM_CH_0_MASK, BPWM_CENTER_ALIGNED);
    /* Clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 1);
    /* BPWM counter value */ /* PWM frequency = PWM clock source/(clock prescaler setting + 1)/(CNR+1) */
    BPWM_SET_CNR(BPWM0, 0, 5);
    /* BPWM compare value */
    BPWM_SET_CMR(BPWM0, 0, 1);
    /* Enable BPWM0 to trigger ADC */
    BPWM0->ADCTS0 = BPWM_TRIGGER_ADC_EVEN_PERIOD_POINT | BPWM_ADCTS0_TRGEN0_Msk;
    /* BPWM0 pin output enabled. PWM frequency 1MHz, duty 30%. */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, BPWM_CH_0_MASK, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING);

    BPWM0->POEN = BPWM_POEN_POEN0_Msk;
    /* Enable the BPWM0 counter */
    BPWM0->CNTEN |= BPWM_CNTEN_CNTEN0_Msk;

    /* wait for one cycle */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((BPWM0->INTSTS & BPWM_INTSTS_PIF0_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM period interrupt time-out!\n");
            return;
        }
    }

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((BPWM0->INTSTS & BPWM_INTSTS_ZIF0_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM zero interrupt time-out!\n");
            return;
        }
    }
    BPWM0->INTSTS = BPWM_INTSTS_PIF0_Msk | BPWM_INTSTS_ZIF0_Msk;

    /* Disable the BPWM0 counter */
    BPWM0->CNTEN = 0;

    /* Wait conversion done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!((ADC->ADSR0 & ADC_ADSR0_ADF_Msk) >> ADC_ADSR0_ADF_Pos))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for ADC conversion done time-out!\n");
            return;
        }
    }

    /* Clear the ADC interrupt flag */
    ADC->ADSR0 = ADC_ADSR0_ADF_Msk;

    printf("Channel 2: 0x%X\n", ADC->ADDR[2] & 0xFFF);

    /* Disable ADC */
    ADC->ADCR = 0;
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

    /* ADC hardware trigger test */
    ADC_PWMTrigTest_SingleOpMode();

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
