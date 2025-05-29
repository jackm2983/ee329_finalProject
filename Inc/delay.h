
#ifndef INC_DELAY_H_
#define INC_DELAY_H_

#include <stdint.h>

void SysTick_Init(void);
void delay_us(const uint32_t time_us);

#endif
