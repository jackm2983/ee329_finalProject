/*USER CODE BEGIN Header*/
/* ============================================================================
EE 329 A7 UART
*=============================================================================
@file: lpuart.c
@brief: Module to initialize and define LPUART functionality
Project : EE329 S'25 Assignment 7
Authors: Joshua Crain & Jack Marshall
Version : 1.0
Date 5/7/2025
compiler STM32CubeIDE v.
target : NUCLEO-L4A7ZG
clocks : 4Mhz MSI to MHB2
@attention: (c) 2025 STMicroelectronics. ALl rights reserved
*
Pin Assignments (ST Nucleo L4A6ZG):
*
Function                   | Pin Name  | STM32-L4A7ZG Pin            |
---------------------------|--------- -|-----------------------------|
LPUART                     |           |                             |
----------------------------|-----------|-----------------------------|
LPUART D4                  | PG7       |    TX                       |
LPUART D5                  | PG8       |    RX                       |
----------------------------|-----------|-----------------------------|
MAIN.C FUNCTIONS
*=========================================================================
Functions for configuring GPIO pins, printing to serial terminal,
printing escape codes, moving the terminal cursor, and IRQ handler.
/
/ 45678-1-2345678-2-2345678-3-2345678-4-2345678-5-2345678-6-2345678-7-234567 */
#include "lpuart.h"
#include "delay.h"
#include "main.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal_conf.h"


/* -----------------------------------------------------------------------------
 * function : LPUART_SetCursor(int row, int col)
 * INs      : row - desired row to place cursor
 * 			  col desired column to place cursor
 * OUTs     : none
 * action   : move terminal cursor to a desired location
 * authors  : Jack Marshall and Joshua Crain
 * version  : 0.1
 * date     : 5/7/2025
 * -------------------------------------------------------------------------- */
void LPUART_SetCursor(int row, int col) {
    char buffer[20];
    sprintf(buffer, "\033[%d;%dH", row, col);
    LPUART_print(buffer);
}

/* -----------------------------------------------------------------------------
 * function : LPUART_init( )
 * INs      : none
 * OUTs     : none
 * action   : Configure pins PG7,8 for serial communication through LPUART
 * authors  : Jack Marshall and Joshua Crain
 * version  : 0.1
 * date     : 5/7/2025
 * -------------------------------------------------------------------------- */
void LPUART_init(void) {
    // 1. Enable power to GPIOG and LPUART1
    PWR->CR2 |= PWR_CR2_IOSV;                  // Enable PG[15:2]
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;       // Enable GPIOG clock
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;   // Enable LPUART1 clock
    RCC->CCIPR |= (0b01 << RCC_CCIPR_LPUART1SEL_Pos); // 01: PCLK selected

    // 2. Configure GPIOG pins 7 (TX) and 8 (RX)
    GPIOG->MODER &= ~(GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk);
    GPIOG->MODER |= (0b10 << GPIO_MODER_MODE7_Pos) | (0b10 << GPIO_MODER_MODE8_Pos);  // AF mode
    GPIOG->OSPEEDR |= (0b11 << GPIO_OSPEEDR_OSPEED7_Pos); // PG7 = very high speed
    GPIOG->AFR[0] |= (8 << (7 * 4)); // PG7 = AF8
    GPIOG->AFR[1] |= (8 << (0 * 4)); // PG8 = AF8


    // 3. LPUART configuration
    LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit word length
    LPUART1->BRR = 8888;                              // Baud rate for 115200 @ 2MHz. 4444
    LPUART1->CR1 |= USART_CR1_RE | USART_CR1_TE;    // Enable receiver and transmitter
    LPUART1->CR1 |= USART_CR1_RXNEIE;               // Enable receive interrupt
    LPUART1->CR1 |= USART_CR1_UE;                   // Enable UART

    // 4. Enable LPUART1 interrupt in NVIC
    NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));
    __enable_irq(); // Global interrupt enable
}

/* -----------------------------------------------------------------------------
 * function : LPUART_ESC_Print(const char* esc_sequence, const char* text)
 * INs      : esc_sequence - character string containing LPUART escape code
 * 			  text - character sequences containing text to be displayed
 * OUTs     : none
 * action   : Sends two character strings, intended to organize one escape code
 * 			  and one text string into a single line.
 * authors  : Jack Marshall and Joshua Crain
 * version  : 0.1
 * date     : 5/7/2025
 * -------------------------------------------------------------------------- */
void LPUART_ESC_Print(const char* esc_sequence, const char* text) {
    LPUART_print(esc_sequence);
    LPUART_print(text);
}

/* -----------------------------------------------------------------------------
 * function : LPUART_print(const char* message)
 * INs      : message - character string to be displayed or escape code
 * OUTs     : none
 * action   : prints character string to terminal
 * authors  : Jack Marshall and Joshua Crain
 * version  : 0.1
 * date     : 5/7/2025
 * -------------------------------------------------------------------------- */
void LPUART_print(const char* message) {
    while (*message) {
        while (!(LPUART1->ISR & USART_ISR_TXE))
            ;  // Wait for TX buffer empty
        LPUART1->TDR = *message++;
    }
}

/* -----------------------------------------------------------------------------
 * function : LPUART1_IRQHandler( )
 * INs      : none
 * OUTs     : none
 * action   : Interupt Request Handler for LPUART serial communication and game
 * 			  controls
 * authors  : Jack Marshall and Joshua Crain
 * version  : 0.1
 * date     : 5/7/2025
 * -------------------------------------------------------------------------- */
void LPUART1_IRQHandler(void) {
    uint8_t charRecv;
    if (LPUART1->ISR & USART_ISR_RXNE) {
        charRecv = LPUART1->RDR;  // Read received character

		while (!(LPUART1->ISR & USART_ISR_TXE));
		LPUART1->TDR = charRecv;  // Echo


    }
}



