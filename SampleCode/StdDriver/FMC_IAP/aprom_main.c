/******************************************************************************
 * @file     APROM_main.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 20/11/27 $
 * @brief    Show how to reboot to LDROM functions from APROM.
 *           This sample code set VECMAP to LDROM and reset to re-boot to LDROM.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


typedef void (FUNC_PTR)(void);

extern uint32_t  loaderImage1Base, loaderImage1Limit;

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

}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}

static int SetIAPBoot(void)
{
    uint32_t  au32Config[2];
    uint32_t u32CBS;

    /* Read current boot mode */
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    if(u32CBS & 1)
    {
        /* Modify User Configuration when it is not in IAP mode */

        FMC_ReadConfig(au32Config, 2);
        if(au32Config[0] & 0x40)
        {
            FMC_EnableConfigUpdate();
            au32Config[0] &= ~0x40;
            FMC_Erase(FMC_CONFIG_BASE);
            FMC_WriteConfig(au32Config, 2);

            // Perform chip reset to make new User Config take effect
            SYS_ResetChip();
        }
    }
    return 0;
}

static int  LoadImage(uint32_t u32ImageBase, uint32_t u32ImageLimit, uint32_t u32FlashAddr, uint32_t u32MaxSize)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = u32MaxSize;

    printf("Program image to flash address 0x%x...", u32FlashAddr);
    pu32Loader = (uint32_t *)u32ImageBase;
    for(i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        FMC_Erase(u32FlashAddr + i);
        for(j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            FMC_Write(u32FlashAddr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\n");

    printf("Verify ...");

    /* Verify loader */
    for(i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for(j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(u32FlashAddr + i + j);

            if(u32Data != pu32Loader[(i + j) / 4])
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", u32FlashAddr + i + j, u32Data, pu32Loader[(i + j) / 4]);
                return -1;
            }

            if(i + j >= u32ImageSize)
                break;
        }
    }
    printf("OK.\n");
    return 0;
}

int main()
{
    uint8_t     u8Item;
    uint32_t    u32Data;
    char *acBootMode[] = {"LDROM+IAP", "LDROM", "APROM+IAP", "APROM"};
    uint32_t u32CBS;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
    UART_Init();

    printf("\n\n");
    printf("+------------------------------------+\n");
    printf("|       FMC IAP Sample Code          |\n");
    printf("|          [APROM code]              |\n");
    printf("+------------------------------------+\n");


    /* Enable FMC ISP function */
    FMC_Open();

    if(SetIAPBoot() < 0)
    {
        printf("Failed to set IAP boot mode!\n");
        goto lexit;
    }

    /* Get boot mode */
    printf("  Boot Mode ............................. ");
    u32CBS = (FMC->ISPSTS & FMC_ISPSTS_CBS_Msk) >> FMC_ISPSTS_CBS_Pos;
    printf("[%s]\n", acBootMode[u32CBS]);

    u32Data = FMC_ReadCID();
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 4));

    do
    {
        printf("\n\n\n");
        printf("+----------------------------------------+\n");
        printf("|               Select                   |\n");
        printf("+----------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM             |\n");
        printf("| [1] Run IAP program (in LDROM)         |\n");
        printf("+----------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();
        printf("%c\n", u8Item);

        switch(u8Item)
        {
        case '0':
            FMC_EnableLDUpdate();
            if(LoadImage((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                         FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
            {
                printf("Load image to LDROM failed!\n");
                goto lexit;
            }
            FMC_DisableLDUpdate();
            break;

        case '1':
            printf("\n\nChange VECMAP and branch to LDROM...\n");
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            UART_WAIT_TX_EMPTY(UART0)  /* To make sure all message has been print out */
                if(--u32TimeOutCnt == 0) break;

            /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
            __set_PRIMASK(1);

            /* Set VECMAP to LDROM for booting from LDROM */
            FMC_SetVectorPageAddr(FMC_LDROM_BASE);

            /* Software reset to boot to LDROM */
            NVIC_SystemReset();

            break;

        default :
            break;
        }
    }
    while(1);


lexit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while(1);
}

