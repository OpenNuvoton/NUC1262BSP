/****************************************************************************
 * @file     main.c
 * @version  V3.0
 * $Revision: 3 $
 * $Date: 17/05/04 1:14p $
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
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(72000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC/2 and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC_DIV2, CLK_CLKDIV0_UART0(1));

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /* Select ADC module clock source as HIRC/2 and ADC module clock divider as 7 */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADCSEL_HIRC_DIV2, CLK_CLKDIV0_ADC(7));

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

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

	/* Set multi-function pins for BPWM */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
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

    printf("\n<<< PWM trigger test (Single mode) >>>\n");

    /* Power on ADC module */
    ADC_POWER_ON(ADC);

    /* Set the ADC operation mode as single, input mode as single-end and enable the analog input channel 2 */
    ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE, 0x1 << 2);

    /* Configure the hardware trigger condition and enable hardware trigger; PWM trigger delay: (4*10) system clock cycles*/
    ADC_EnableHWTrigger(ADC, ADC_ADCR_TRGS_BPWM, 0);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Center-aligned type */
    BPWM_SET_ALIGNED_TYPE(BPWM0, BPWM_CH_0_MASK, BPWM_CENTER_ALIGNED);
    /* Clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 1);
    /* BPWM counter value */ /* PWM frequency = PWM clock source/(clock prescaler setting + 1)/(CNR+1) */
    BPWM_SET_CNR(BPWM0, 0, 5);
    /* BPWM compare value */
    BPWM_SET_CMR(BPWM0, 0, 1);
    /* Enable BPWM0 to trigger ADC */
    BPWM_EnableADCTrigger(BPWM0, 0, BPWM_TRIGGER_ADC_EVEN_PERIOD_POINT);
    /* BPWM0 pin output enabled */
    BPWM_SET_OUTPUT_LEVEL(BPWM0, BPWM_CH_0_MASK, BPWM_OUTPUT_HIGH, BPWM_OUTPUT_NOTHING, BPWM_OUTPUT_LOW, BPWM_OUTPUT_NOTHING);

    BPWM_EnableOutput(BPWM0, BPWM_CH_0_MASK);

    /* Start BPWM module */
    BPWM_Start(BPWM0, BPWM_CH_0_MASK);

    /* wait for one cycle */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(BPWM_GetPeriodIntFlag(BPWM0, 0) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM period interrupt time-out!\n");
            return;
        }
    }
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(BPWM_GetZeroIntFlag(BPWM0, 0) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM zero interrupt time-out!\n");
            return;
        }
    }
    BPWM_ClearPeriodIntFlag(BPWM0, 0);
    BPWM_ClearZeroIntFlag(BPWM0, 0);

    /* Stop BPWM generation */
    BPWM_ForceStop(BPWM0, BPWM_CH_0_MASK);

    /* Wait conversion done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for ADC conversion done time-out!\n");
            return;
        }
    }

    /* Clear the ADC interrupt flag */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    printf("Channel 2: 0x%X\n", (uint32_t)ADC_GET_CONVERSION_DATA(ADC, 2));

    /* Disable ADC */
    ADC_POWER_DOWN(ADC);

//     while(1);
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

    /* Disable ADC module */
    ADC_Close(ADC);

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("\nExit ADC sample code\n");

    while(1);

}
