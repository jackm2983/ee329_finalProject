/*USER CODE BEGIN Header */
/* ============================================================================
* EE 329 A9
*=============================================================================
* @file: LEDs.c
* @brief: Enables PB7
* Project : EE329 S'25 A9
* Authors: Ben Tavares & Jack Marshall
* Version : 1.0
* Date 5/19/2025
* compiler STM32CubeIDE v.
* target : NUCLEO-L4A7ZG
* clocks : 4Mhz MSI to MHB2
* @attention: (c) 2025 STMicroelectronics. ALl rights reserved
*
* Pin Assignments (ST Nucleo L4A6ZG):
*
*  Function                   | Pin Name  | STM32-L4A7ZG Pin            |
*  ---------------------------|--------- -|-----------------------------|
*  LED                        |           |                             |
* ----------------------------|-----------|-----------------------------|
*  LED 1                      | PB7       |    CN11       OUT           |
* ----------------------------|-----------|-----------------------------|
* LEDs.C FUNCTIONS
*=========================================================================
*  - Enable PB7.
*/
/* 45678-1-2345678-2-2345678-3-2345678-4-2345678-5-2345678-6-2345678-7-234567 */
/*USER CODE END HEADER */

#include "main.h"


/* -----------------------------------------------------------------------------
* function : LEDs_init( )
* INs      : none
* OUTs     : none
* action   : Enables the blue LED on the STM32 board.
* authors  : Ben Tavares, Jack Marshall
* version  : 0.1
* date     : 250430
* -------------------------------------------------------------------------- */
void LEDs_init(void) {

   // PB7 - blue LED
   GPIOB->BRR = GPIO_PIN_7;
   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
   GPIOB->MODER &= ~(GPIO_MODER_MODE7);
   GPIOB->MODER |=  (GPIO_MODER_MODE7_0);
   GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7);
   GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);
   GPIOB->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED7_Pos);
   GPIOB->BRR = GPIO_PIN_7;
}
   
