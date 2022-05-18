/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data in UART RS485 mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define IS_USE_RS485NMM     0      //1:Select NMM_Mode , 0:Select AAD_Mode
#define MATCH_ADDRSS1       0xC0
#define MATCH_ADDRSS2       0xA2
#define UNMATCH_ADDRSS1     0xB1
#define UNMATCH_ADDRSS2     0xD3


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void RS485_HANDLE(void);
void RS485_9bitModeSlave(void);
void RS485_FunctionTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    RS485_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE()
{
    volatile uint32_t addr = 0;
    volatile uint32_t regRX = 0xFF;
    volatile uint32_t u32IntSts = UART1->INTSTS;;

    if((u32IntSts & UART_INTSTS_RLSINT_Msk) && (u32IntSts & UART_INTSTS_RDAINT_Msk))           /* RLS INT & RDA INT */ //For RS485 Detect Address
    {
        if(UART1->FIFOSTS & UART_FIFOSTS_ADDRDETF_Msk)   /* RS485 address byte detect flag */
        {
            addr = UART1->DAT;
            UART1->FIFOSTS = UART_FIFOSTS_ADDRDETF_Msk;  /* Clear RS485 address byte detect flag */
            printf("\nAddr=0x%x,Get:", addr);

#if (IS_USE_RS485NMM ==1) //RS485_NMM
            /* if address match, enable RX to receive data, otherwise to disable RX. */
            /* In NMM mode,user can decide multi-address filter. In AAD mode,only one address can set */
            if((addr == MATCH_ADDRSS1) || (addr == MATCH_ADDRSS2))
            {
                UART1->FIFO &= ~UART_FIFO_RXOFF_Msk;   /* Enable RS485 RX */
            }
            else
            {
                UART1->FIFO |= UART_FIFO_RXOFF_Msk;    /* Disable RS485 RX */
                UART1->FIFO |= UART_FIFO_RXRST_Msk;    /* Clear data from RX FIFO */
            }
#endif

        }
    }
    else if((u32IntSts & UART_INTSTS_RDAINT_Msk) || (u32IntSts & UART_INTSTS_RXTOINT_Msk))      /* Rx Ready or Time-out INT */
    {
        /* Handle received data */
        printf("%d,", UART1->DAT);

        /* Forces a write of all user-space buffered data for the given output */
        fflush(stdout);
    }

    else if(u32IntSts & UART_INTSTS_BUFERRINT_Msk)     /* Buffer Error INT */
    {
        printf("\nBuffer Error...\n");
        UART1->FIFOSTS = (UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test (IS_USE_RS485NMM: 0:AAD  1:NMM)                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave()
{
    uint32_t u32TimeOutCnt;

    /* Select UART RS485 function mode */
    UART1->FUNCSEL = UART_FUNCSEL_RS485;

    /* Set Data Format, only need parity enable whenever parity ODD/EVEN */
    UART1->LINE = (UART_WORD_LEN_8 | UART_PARITY_EVEN | UART_STOP_BIT_1);

    /* Set RTS pin active level as high level active */
    UART1->MODEM = (UART1->MODEM & (~UART_MODEM_RTSACTLV_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

#if(IS_USE_RS485NMM == 1)
    printf("+-----------------------------------------------------------+\n");
    printf("|    Normal Multidrop Operation Mode                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("| Only Address %x and %x,data can receive                   |\n", MATCH_ADDRSS1, MATCH_ADDRSS2);
    printf("+-----------------------------------------------------------+\n");

    /* Set Receiver disabled before set RS485-NMM mode */
    UART1->FIFO |= UART_FIFO_RXOFF_Msk;

    /* Set RS485-NMM Mode, RS485 address detection enable */
    UART1->ALTCTL = UART_ALTCTL_RS485NMM_Msk |
                    UART_ALTCTL_ADDRDEN_Msk  |
                    UART_ALTCTL_RS485AUD_Msk;

#else

    printf("Auto Address Match Operation Mode\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("|    Auto Address Match Operation Mode                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|Only Address %x,data can receive                           |\n", MATCH_ADDRSS1);
    printf("+-----------------------------------------------------------+\n");

    /* Set RS485-AAD Mode, address match is 0xC0 and RS485 address detection enable */
    UART1->ALTCTL = UART_ALTCTL_RS485AAD_Msk    |
                    UART_ALTCTL_ADDRDEN_Msk |
                    UART_ALTCTL_RS485AUD_Msk    |
                    ((uint32_t)(MATCH_ADDRSS1) << UART_ALTCTL_ADDRMV_Pos) ;

#endif

    /* Enable RDA\RLS\Time-out Interrupt */
    UART1->INTEN |= UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk;

    /* Enable UART1 IRQ */
    NVIC_EnableIRQ(UART1_IRQn);

    printf("Ready to receive data...(Press any key to stop test)\n");
    GetChar();

    /* Flush FIFO */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(UART_GET_RX_EMPTY(UART1) == 0)
    {
        UART_READ(UART1);
        if(--u32TimeOutCnt == 0) break;
    }

    /* Disable RDA\RLS\Time-out Interrupt */
    UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    printf("\nEnd test\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                         */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    /* Set UART parity as MARK */
    UART1->LINE = (UART_WORD_LEN_8 | UART_PARITY_MARK | UART_STOP_BIT_1);

    /* Send data */
    UART1->DAT = u8data;
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t u32Count;

    /* Set UART parity as SPACE */
    UART1->LINE = (UART_WORD_LEN_8 | UART_PARITY_SPACE | UART_STOP_BIT_1);

    /* Send data */
    for(u32Count = 0; u32Count != u32WriteBytes; u32Count++)
    {
        while(!(UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk));   /* Wait Tx empty */

        UART1->DAT = pu8TxBuf[u32Count]; /* Send UART Data from buffer */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster()
{
    volatile int32_t i32;
    uint8_t g_u8SendDataGroup1[10] = {0};
    uint8_t g_u8SendDataGroup2[10] = {0};
    uint8_t g_u8SendDataGroup3[10] = {0};
    uint8_t g_u8SendDataGroup4[10] = {0};

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|               RS485 9-bit Master Test                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function will send different address with 10 data     |\n");
    printf("| bytes to test RS485 9-bit mode. Please connect TX/RX to   |\n");
    printf("| another board and wait its ready to receive.              |\n");
    printf("| Press any key to start...                                 |\n");
    printf("+-----------------------------------------------------------+\n\n");
    GetChar();

    /* Select UART RS485 function mode */
    UART1->FUNCSEL = UART_FUNCSEL_RS485;

    /* Set RS485-Master as AUD mode */
    /* Enable AUD mode to HW control RTS pin automatically */
    /* It also can use GPIO to control RTS pin for replacing AUD mode */
    UART1->ALTCTL = UART_ALTCTL_RS485AUD_Msk;

    /* Set RTS pin active level as high level active */
    UART1->MODEM = (UART1->MODEM & (~UART_MODEM_RTSACTLV_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

    /* Set TX delay time */
    UART1->TOUT = 0x2000;

    /* Prepare data to transmit */
    for(i32 = 0; i32 < 10; i32++)
    {
        g_u8SendDataGroup1[i32] = i32;
        g_u8SendDataGroup2[i32] = i32 + 10;
        g_u8SendDataGroup3[i32] = i32 + 20;
        g_u8SendDataGroup4[i32] = i32 + 30;
    }

    /* Send different address and data for test */
    printf("Send Address %x and data 0~9\n", MATCH_ADDRSS1);
    RS485_SendAddressByte(MATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup1, 10);

    printf("Send Address %x and data 10~19\n", UNMATCH_ADDRSS1);
    RS485_SendAddressByte(UNMATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup2, 10);

    printf("Send Address %x and data 20~29\n", MATCH_ADDRSS2);
    RS485_SendAddressByte(MATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup3, 10);

    printf("Send Address %x and data 30~39\n", UNMATCH_ADDRSS2);
    RS485_SendAddressByte(UNMATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup4, 10);

    printf("Transfer Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest(void)
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      RS485 Function Test IO Setting                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PA.3)        UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |--UART1_nRTS(PA.0)      UART1_nRTS(PA.0)--|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       RS485 Function Test                                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n\n");
    u32Item = (uint32_t)getchar();

    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.

        Master:
            1.Set AUD mode and HW will control RTS pin. RTSACTLV is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UART_LINE = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UART_LINE = 0x3B)
            5.RTS pin is low in idle state. When master is sending, RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. RTSACTLV is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match address match value.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check if RS485 address byte detect flag is set and read RX FIFO data to clear ADDRESS stored in RX FIFO.

              NMM: The slave will ignore data byte until RXOFF is disabled.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check the ADDRESS is match or not by user in UART_IRQHandler.
              If the ADDRESS is match, clear RXOFF bit to receive data byte.
              If the ADDRESS is not match, set RXOFF bit to avoid data byte stored in FIFO.

        Note: User can measure transmitted data waveform on TXD and RXD pin.
              RTS pin is used for RS485 transceiver to control transmission direction.
              RTS pin is low in idle state. When master is sending data, RTS pin will be pull high.
              The connection to RS485 transceiver is as following figure for reference.
               __________     ___________      ___________      __________
              |          |   |           |    |           |    |          |
              |Master    |   |RS485      |    |RS485      |    |Slave     |
              | UART_TXD |---|Transceiver|<==>|Transceiver|----| UART_RXD |
              | UART_nRTS|---|           |    |           |----| UART_nRTS|
              |__________|   |___________|    |___________|    |__________|
    */

    if(u32Item == '0')
        RS485_9bitModeMaster();
    else
        RS485_9bitModeSlave();

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

    /* Enable UART module clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_UART0CKEN_Msk | CLK_APBCLK0_UART1CKEN_Msk);

    /* Select UART module clock source and UART module clock divider */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HXT;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART0DIV_Msk)) | CLK_CLKDIV0_UART0(1);
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | CLK_CLKSEL1_UART1SEL_HXT;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | CLK_CLKDIV0_UART1(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PA multi-function pins for UART1 RXD(PA.2) and TXD(PA.3) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA2MFP_Msk)) | SYS_GPA_MFPL_PA2MFP_UART1_RXD;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA3MFP_Msk)) | SYS_GPA_MFPL_PA3MFP_UART1_TXD;

    /* Set PA multi-function pin for UART1 RTS(PA.0) */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_UART1_nRTS;

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
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRST1 |=  SYS_IPRST1_UART1RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART1RST_Msk;

    /* Configure UART1 and set UART1 baud rate */
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART1->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
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

    /* Init UART1 for testing */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART RS485 sample function */
    RS485_FunctionTest();

    printf("\nUART Sample Program End\n");

    while(1);

}
