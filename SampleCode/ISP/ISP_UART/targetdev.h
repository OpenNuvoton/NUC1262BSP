/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x32
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"
#include "isp_user.h"


/* rename for uart_transfer.c */
#define UART_T					UART0
#define UART_T_IRQHandler		UART0_IRQHandler
#define UART_T_IRQn				UART0_IRQn

#define CONFIG_SIZE 8 // in bytes
