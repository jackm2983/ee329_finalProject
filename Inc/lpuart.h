/*
 * lpuart.h
 *
 *  Created on: May 19, 2025
 *      Author: J_mas
 */

#ifndef INC_LPUART_H_
#define INC_LPUART_H_


void LPUART_init(void);
void LPUART1_IRQHandler(void);
void LPUART_print(const char* message);
void LPUART_ESC_Print(const char* esc_sequence, const char* text);
void LPUART_SetCursor(int row, int col);


#endif /* INC_LPUART_H_ */
