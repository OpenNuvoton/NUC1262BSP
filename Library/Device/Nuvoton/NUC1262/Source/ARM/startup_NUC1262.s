/**************************************************************************//**
 * @file     startup_NUC1262.s
 * @version  V3.00
 * @brief    NUC1262 Series Startup Source File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/


    .section .bss.STACK, "aw", %nobits
    .align 3

    .global __initial_sp
#ifndef Stack_Size
// <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Stack_Size, 0x00000400
#endif
Stack_Mem:
    .space   Stack_Size
__initial_sp:


    .section .bss.HEAP, "aw", %nobits
    .align  3
    .global Heap_Mem
    .global __heap_base
    .global __heap_limit
#ifndef Heap_Size
// <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Heap_Size, 0x00000000
#endif
__heap_base:
Heap_Mem:
    .space  Heap_Size
__heap_limit:

    .eabi_attribute Tag_ABI_align_preserved, 1
    .thumb

//*** <<< end of configuration section >>>    ***


// ; Vector Table Mapped to Address 0 at Reset
    .section RESET, "ax"
    .global     __Vectors
    .global     __Vectors_End
    .global     __Vectors_Size

__Vectors:
    .word     __initial_sp              //; Top of Stack
    .word     Reset_Handler             //; Reset Handler
    .word     NMI_Handler               //; NMI Handler
    .word     HardFault_Handler         //; Hard Fault Handler
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     SVC_Handler               //; SVCall Handler
    .word     0                         //; Reserved
    .word     0                         //; Reserved
    .word     PendSV_Handler            //; PendSV Handler
    .word     SysTick_Handler           //; SysTick Handler

    //; External Interrupts

    .word     BOD_IRQHandler  
    .word     WDT_IRQHandler  
    .word     EINT024_IRQHandler
    .word     EINT135_IRQHandler
    .word     GPAB_IRQHandler 
    .word     GPCDF_IRQHandler
    .word     BPWM0_IRQHandler 
    .word     BPWM1_IRQHandler 
    .word     TMR0_IRQHandler 
    .word     TMR1_IRQHandler 
    .word     TMR2_IRQHandler 
    .word     TMR3_IRQHandler 
    .word     UART0_IRQHandler
    .word     UART1_IRQHandler
    .word     SPI0_IRQHandler 
    .word     SPI1_IRQHandler 
    .word     BPWM2_IRQHandler 
    .word     BPWM3_IRQHandler 
    .word     I2C0_IRQHandler 
    .word     I2C1_IRQHandler 
    .word     DEFAULT_IRQHandler 
    .word     DEFAULT_IRQHandler
    .word     DEFAULT_IRQHandler 
    .word     USBD_IRQHandler  
    .word     DEFAULT_IRQHandler  
    .word     DEFAULT_IRQHandler 
    .word     PDMA_IRQHandler
    .word     DEFAULT_IRQHandler 
    .word     PWRWU_IRQHandler
    .word     ADC_IRQHandler
    .word     CLKDIRC_IRQHandler  
    .word     DEFAULT_IRQHandler  
    .word     LLSI0_IRQHandler
    .word     LLSI1_IRQHandler
    .word     LLSI2_IRQHandler
    .word     LLSI3_IRQHandler
    .word     LLSI4_IRQHandler
    .word     LLSI5_IRQHandler
    .word     LLSI6_IRQHandler
    .word     LLSI7_IRQHandler
    .word     LLSI8_IRQHandler
    .word     LLSI9_IRQHandler

__Vectors_End:
    .equ    __Vectors_Size, __Vectors_End - __Vectors

    .section .text, "ax"

// ; Reset Handler
    .global Reset_Handler
    .global  SystemInit
    .global  __main
    .type   Reset_Handler, "function"
Reset_Handler:
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__main
        BX      R0


// ; Dummy Exception Handlers (infinite loops which can be modified)
    .weak   NMI_Handler
    .type   NMI_Handler, "function"
NMI_Handler:
        B       .

    .weak   HardFault_Handler
    .type   HardFault_Handler, "function"

HardFault_Handler:
        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0

    .weak   SVC_Handler, "function"
SVC_Handler:
        B       .

    .weak   PendSV_Handler, "function"
PendSV_Handler:
        B       .

    .weak   SysTick_Handler, "function"
SysTick_Handler:
        B       .

    .weak  BOD_IRQHandler,"function"
    .weak  WDT_IRQHandler,"function"
    .weak  EINT024_IRQHandler,"function"
    .weak  EINT135_IRQHandler,"function"
    .weak  GPAB_IRQHandler,"function"
    .weak  GPCDF_IRQHandler,"function"
    .weak  BPWM0_IRQHandler,"function"
    .weak  BPWM1_IRQHandler,"function"
    .weak  TMR0_IRQHandler,"function"
    .weak  TMR1_IRQHandler,"function"
    .weak  TMR2_IRQHandler,"function"
    .weak  TMR3_IRQHandler,"function"
    .weak  UART0_IRQHandler,"function"
    .weak  UART1_IRQHandler,"function"
    .weak  SPI0_IRQHandler,"function"
    .weak  SPI1_IRQHandler,"function"
    .weak  BPWM2_IRQHandler,"function"
    .weak  BPWM3_IRQHandler,"function"
    .weak  I2C0_IRQHandler,"function"
    .weak  I2C1_IRQHandler,"function"
    .weak  USBD_IRQHandler,"function"
    .weak  PDMA_IRQHandler,"function"
    .weak  PWRWU_IRQHandler,"function"
    .weak  ADC_IRQHandler,"function"
    .weak  CLKDIRC_IRQHandler,"function"
    .weak  LLSI0_IRQHandler,"function"
    .weak  LLSI1_IRQHandler,"function"
    .weak  LLSI2_IRQHandler,"function"
    .weak  LLSI3_IRQHandler,"function"
    .weak  LLSI4_IRQHandler,"function"
    .weak  LLSI5_IRQHandler,"function"
    .weak  LLSI6_IRQHandler,"function"
    .weak  LLSI7_IRQHandler,"function"
    .weak  LLSI8_IRQHandler,"function"
    .weak  LLSI9_IRQHandler,"function"
    .weak  DEFAULT_IRQHandler,"function"

Default_Handler:
BOD_IRQHandler:
WDT_IRQHandler:
EINT024_IRQHandler:
EINT135_IRQHandler:
GPAB_IRQHandler:
GPCDF_IRQHandler:
BPWM0_IRQHandler:
BPWM1_IRQHandler:
TMR0_IRQHandler:
TMR1_IRQHandler:
TMR2_IRQHandler:
TMR3_IRQHandler:
UART0_IRQHandler:
UART1_IRQHandler:
SPI0_IRQHandler:
SPI1_IRQHandler:
BPWM2_IRQHandler:
BPWM3_IRQHandler:
I2C0_IRQHandler:
I2C1_IRQHandler:
USBD_IRQHandler:
PDMA_IRQHandler:
PWRWU_IRQHandler:
ADC_IRQHandler:
CLKDIRC_IRQHandler:
LLSI0_IRQHandler:
LLSI1_IRQHandler:
LLSI2_IRQHandler:
LLSI3_IRQHandler:
LLSI4_IRQHandler:
LLSI5_IRQHandler:
LLSI6_IRQHandler:
LLSI7_IRQHandler:
LLSI8_IRQHandler:
LLSI9_IRQHandler:
DEFAULT_IRQHandler:
        B       .

    .align 2


// ;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
    .global     SH_DoCommand
    .global     SH_Return
    .type       SH_DoCommand, "function"
SH_DoCommand:
                BKPT   0xAB                // ; Wait ICE or HardFault
                LDR    R3, =SH_Return
                MOV    R4, lr
                BLX    R3                  // ; Call SH_Return. The return value is in R0
                BX     R4                  // ; Return value = R0

    .align 2

    .global     __PC
    .type       __PC, "function"
__PC:
                MOV     r0, lr
                BLX     lr
