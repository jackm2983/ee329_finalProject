#include "motor.h"
#include "main.h"
#include <stdio.h>
#include "lpuart.h"


//L298N Pin	STM32 Pin	Use
//IN1 (Motor A)	PA0	Direction A
//IN2 (Motor A)	PA1	Direction A
//ENA (Motor A)	PA2	PWM (TIM2_CH3)
//IN3 (Motor B)	PB0	Direction B
//IN4 (Motor B)	PB1	Direction B
//ENB (Motor B)	PA3	PWM (TIM2_CH4)


void motor_init() {
	// Enable GPIOA and GPIOB
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

	// Set PA0, PA1 as output (Motor A dir)
	GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
	GPIOA->MODER |=  (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0);
	GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);
	GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
	GPIOA->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) |
	                        (3 << GPIO_OSPEEDR_OSPEED1_Pos));
	GPIOA->BRR = (GPIO_PIN_0 | GPIO_PIN_1);

	// Set PB0, PB1 as output (Motor B dir)
	GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
	GPIOB->MODER |=  (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0);
	GPIOB->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);
	GPIOB->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1);
	GPIOB->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos) |
	                        (3 << GPIO_OSPEEDR_OSPEED1_Pos));
	GPIOB->BRR = (GPIO_PIN_0 | GPIO_PIN_1);




	// Set PA2 and PA3 to alternate function AF1 (TIM2)
	GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
	GPIOA->MODER |=  ((2 << (2 * 2)) | (2 << (3 * 2)));
	GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
	GPIOA->AFR[0] |=  ((1 << (2 * 4)) | (1 << (3 * 4)));  // AF1 = TIM2

	// Enable TIM2
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// Setup base timer (same as before)
	TIM2->PSC = 79;
	TIM2->ARR = 1000 - 1;

	// Motor A PWM = CH3, Motor B PWM = CH4
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;

	// PWM mode
	TIM2->CCMR2 &= ~((7 << TIM_CCMR2_OC3M_Pos) | (7 << TIM_CCMR2_OC4M_Pos));
	TIM2->CCMR2 |=  ((6 << TIM_CCMR2_OC3M_Pos) | (6 << TIM_CCMR2_OC4M_Pos));
	TIM2->CCMR2 |=  TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

	TIM2->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
	TIM2->CR1  |= TIM_CR1_ARPE;
	TIM2->EGR  |= TIM_EGR_UG;
	TIM2->CR1  |= TIM_CR1_CEN;
}

void set_motor_speed(int32_t speed) {
//	char out_buf[32];
//	snprintf(out_buf, sizeof(out_buf), "Output: %ld\r\n", speed);
//	LPUART_ESC_Print("H", "");
//	LPUART_print(out_buf);

    set_motor_A(speed*2);
    set_motor_B(speed*1.5);
}

void set_motor_A(int32_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed >= 0) {
    	GPIOA -> BRR = GPIO_PIN_0;
    	GPIOA -> BSRR = GPIO_PIN_1;
    	//LPUART_print("Made it1");
    } else {
    	GPIOA -> BSRR = GPIO_PIN_0;
    	GPIOA -> BRR = GPIO_PIN_1;
        speed = -speed;
        //LPUART_print("Made it2");
    }
    TIM2->CCR3 = speed; // PWM on PA2 (Motor A)
}

void set_motor_B(int32_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed >= 0) {
    	GPIOB -> BRR = GPIO_PIN_0;
    	GPIOB -> BSRR = GPIO_PIN_1;
    	//LPUART_print("Made it3");
    } else {
    	GPIOB -> BSRR = GPIO_PIN_0;
    	GPIOB -> BRR = GPIO_PIN_1;
    	//LPUART_print("Made it4");
        speed = -speed;
    }
    TIM2->CCR4 = speed; // PWM on PA3 (Motor B)
}
