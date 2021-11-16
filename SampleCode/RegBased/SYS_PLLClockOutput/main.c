/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Change system clock to different PLL frequency and output system clock from CLKO pin.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "NuMicro.h"


#define SIGNATURE       0x125ab234
#define FLAG_ADDR       0x20000FFC

/*---------------------------------------------------------------------------------------------------------*/
/*  Brown Out Detector IRQ Handler                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Clear BOD Interrupt Flag */
    SYS_CLEAR_BOD_INT_FLAG();

    printf("Brown Out is Detected\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Simple calculation test function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define PI_NUM  256
int32_t f[PI_NUM + 1];
uint32_t piTbl[19] =
{
    3141,
    5926,
    5358,
    9793,
    2384,
    6264,
    3383,
    2795,
    288,
    4197,
    1693,
    9937,
    5105,
    8209,
    7494,
    4592,
    3078,
    1640,
    6284
};

int32_t piResult[19];

int32_t pi(void)
{
    int32_t i, i32Err;
    int32_t a = 10000, b = 0, c = PI_NUM, d = 0, e = 0, g = 0;

    for(; b - c;)
        f[b++] = a / 5;

    i = 0;
    for(; d = 0, g = c * 2; c -= 14,/*printf("%.4d\n",e+d/a),*/ piResult[i++] = e + d / a, e = d % a)
    {
        if(i == 19)
            break;

        for(b = c; d += f[b] * a, f[b] = d % --g, d /= g--, --b; d *= b);
    }
    i32Err = 0;
    for(i = 0; i < 19; i++)
    {
        if(piTbl[i] != piResult[i])
            i32Err = -1;
    }

    return i32Err;
}

void Delay(uint32_t x)
{
    int32_t i;

    for(i = 0; i < x; i++)
    {
        __NOP();
        __NOP();
    }
}

uint32_t g_au32PllSetting[] =
{
    CLK_PLLCTL_PLLSRC_HXT | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF(50) | CLK_PLLCTL_NO_4,  /* PLL = 50MHz */
    CLK_PLLCTL_PLLSRC_HXT | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF(72) | CLK_PLLCTL_NO_4,  /* PLL = 72MHz */
    CLK_PLLCTL_PLLSRC_HXT | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF(96) | CLK_PLLCTL_NO_4,  /* PLL = 96MHz */
    CLK_PLLCTL_PLLSRC_HXT | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF(50) | CLK_PLLCTL_NO_2,  /* PLL = 100MHz */
    CLK_PLLCTL_PLLSRC_HXT | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF(52) | CLK_PLLCTL_NO_2,  /* PLL = 104MHz */
    CLK_PLLCTL_PLLSRC_HXT | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF(60) | CLK_PLLCTL_NO_2,  /* PLL = 120MHz */
    CLK_PLLCTL_PLLSRC_HXT | CLK_PLLCTL_NR(3) | CLK_PLLCTL_NF(72) | CLK_PLLCTL_NO_2   /* PLL = 144MHz */
};

void SYS_PLL_Test(void)
{
    int32_t  i;

    /*---------------------------------------------------------------------------------------------------------*/
    /* PLL clock configuration test                                                                            */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n-------------------------[ Test PLL ]-----------------------------\n");

    for(i = 0; i < sizeof(g_au32PllSetting) / sizeof(g_au32PllSetting[0]) ; i++)
    {
        /* Select HCLK clock source to HXT and HCLK source divider as 1 */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HXT;
        CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

        /* Set PLL to power down mode and PLLSTB bit in STATUS register will be cleared by hardware. */
        CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

        /* Set PLL frequency */
        CLK->PLLCTL = g_au32PllSetting[i];

        /* Wait for PLL clock ready */
        while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

        /* Select HCLK clock source to PLL/2 */
        CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL_DIV2;

        /* Update System Core Clock */
        SystemCoreClockUpdate();

        printf("  Change system clock to %d Hz ...................... ", SystemCoreClock);

        /* Enable CKO clock source */
        CLK->APBCLK0 |= CLK_APBCLK0_CLKOCKEN_Msk;

        /* CKO = clock source / 2^(1 + 1) */
        CLK->CLKOCTL = CLK_CLKOCTL_CLKOEN_Msk | (1);

        /* Select CLKO clock source as HCLK */
        CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_CLKOSEL_Msk)) | CLK_CLKSEL2_CLKOSEL_HCLK;

        /* The delay loop is used to check if the CPU speed is increasing */
        Delay(0x400000);

        if(pi())
        {
            printf("[FAIL]\n");
        }
        else
        {
            printf("[OK]\n");
        }

        /* Disable CLKO clock */
        CLK->APBCLK0 &= (~CLK_APBCLK0_CLKOCKEN_Msk);
    }
}

void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable GPIO Port F module clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPIOFCKEN_Msk;

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) as input mode to use HXT */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC and HXT clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for HIRC and HXT clock ready */
    while( (CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk)) != (CLK_STATUS_HIRCSTB_Msk|CLK_STATUS_HXTSTB_Msk) );

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
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((__HIRC>>1), 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    uint32_t u32data;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|       System Driver Sample Code        |\n");
    printf("+----------------------------------------+\n");

    if(M32(FLAG_ADDR) == SIGNATURE)
    {
        printf("  CPU Reset success!\n");
        M32(FLAG_ADDR) = 0;
        printf("  Press any key to continue ...\n\n");
        getchar();
    }

    /*---------------------------------------------------------------------------------------------------------*/
    /* Misc system function test                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Read Part Device ID */
    printf("Product ID 0x%x\n", SYS->PDID);

    /* Get reset source from last operation */
    u32data = SYS->RSTSTS;
    printf("Reset Source 0x%x\n", u32data);

    /* Clear reset source */
    SYS->RSTSTS = u32data;

    /* Unlock protected registers for Brown-Out Detector settings */
    SYS_UnlockReg();

    /* Check if the write-protected registers are unlocked before BOD setting and CPU Reset */
    if(SYS->REGLCTL != 0)
    {
        printf("Protected Address is Unlocked\n");
    }

    /* Enable Brown-Out Detector and Low Voltage Reset function, and set Brown-Out Detector voltage 2.7V */
    SYS->BODCTL = SYS_BODCTL_BODEN_Msk | SYS_BODCTL_BODVL_2_7V | SYS_BODCTL_LVREN_Msk;

    /* Enable BOD interrupt */
    NVIC_EnableIRQ(BOD_IRQn);

    /* Run PLL Test */
    SYS_PLL_Test();

    /* Write a signature work to SRAM to check if it is reset by software */
    M32(FLAG_ADDR) = SIGNATURE;
    printf("\n\n  >>> Reset CPU <<<\n");

    /* Wait for message send out */
    while(!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power down mode and HW will also clear PLLSTB bit in CLK_STATUS register */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Reset CPU */
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;

}
