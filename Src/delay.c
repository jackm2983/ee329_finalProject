/*USER CODE BEGIN Header*/
/* ============================================================================
EE 329 A7 UART
*=============================================================================
@file: delay.c
@brief: Provides delay functions
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
Initialize and perform us delays.
/
/ 45678-1-2345678-2-2345678-3-2345678-4-2345678-5-2345678-6-2345678-7-234567 */
#include "delay.h"
#include "main.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "stm32l4xx_it.h"
#include "stm32l4xx_hal_conf.h"


/* -----------------------------------------------------------------------------
* function : SysTick_Init( )
* INs      : none
* OUTs     : none
* action   : Enables SysTick
* authors  : Jack Marshall and Joshua Crain
* version  : 0.1
* date     : 5/7/2025
* -------------------------------------------------------------------------- */
void SysTick_Init(void) {
  SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk);
  SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);
}

/* -----------------------------------------------------------------------------
* function : delay_us( )
* INs      : const uint32_t time_us
* OUTs     : none
* action   : Clock division and performs an delay of 1us per given time_us.
* authors  : Jack Marshall and Joshua Crain
* version  : 0.1
* date     : 5/7/2025
* -------------------------------------------------------------------------- */
void delay_us(const uint32_t time_us) {
  SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
  SysTick->VAL = 0;
  SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);
  while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}
