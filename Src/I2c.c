
/*
* I2c.c
*
*  Created on: May 15, 2025
*      Author: scarh
*/
#include "stm32l4xx.h"  // CMSIS device header for STM32L4 series
#include "I2c.h"

#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F

int16_t Read_Accel_X(void) {
  uint8_t hi, lo;
  int16_t raw;

  // --- Write register address (ACCEL_XOUT_H) ---
  I2C1->CR2 &= ~I2C_CR2_RD_WRN;                   // WRITE mode
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |=  (1 << I2C_CR2_NBYTES_Pos);
  I2C1->CR2 &= ~I2C_CR2_SADD;
  I2C1->CR2 |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos + 1));
  I2C1->CR2 |=  I2C_CR2_START;
  while (!(I2C1->ISR & I2C_ISR_TXIS));
  I2C1->TXDR = ACCEL_XOUT_H;
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |=  I2C_ICR_STOPCF;

  // --- Read 2 bytes from XOUT_H/XOUT_L ---
  I2C1->CR2 |=  I2C_CR2_RD_WRN;                   // READ mode
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |=  (2 << I2C_CR2_NBYTES_Pos);
  I2C1->CR2 &= ~I2C_CR2_SADD;
  I2C1->CR2 |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos + 1));
  I2C1->CR2 |=  (I2C_CR2_START | I2C_CR2_AUTOEND);
  while (!(I2C1->ISR & I2C_ISR_RXNE));
  hi = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_RXNE));
  lo = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |=  I2C_ICR_STOPCF;

  raw = (int16_t)((hi << 8) | lo);
  return raw;
}


int16_t Read_Accel_Y(void) {
  uint8_t hi, lo;
  int16_t raw;
  I2C1->CR2 &= ~I2C_CR2_RD_WRN;                   // WRITE mode
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |=  (1 << I2C_CR2_NBYTES_Pos);
  I2C1->CR2 &= ~I2C_CR2_SADD;
  I2C1->CR2 |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos + 1));
  I2C1->CR2 |=  I2C_CR2_START;
  while (!(I2C1->ISR & I2C_ISR_TXIS));
  I2C1->TXDR = ACCEL_YOUT_H;
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |=  I2C_ICR_STOPCF;

  I2C1->CR2 |=  I2C_CR2_RD_WRN;                   // READ mode
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |=  (2 << I2C_CR2_NBYTES_Pos);
  I2C1->CR2 &= ~I2C_CR2_SADD;
  I2C1->CR2 |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos + 1));
  I2C1->CR2 |=  (I2C_CR2_START | I2C_CR2_AUTOEND);
  while (!(I2C1->ISR & I2C_ISR_RXNE));
  hi = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_RXNE));
  lo = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |=  I2C_ICR_STOPCF;

  raw = (int16_t)((hi << 8) | lo);
  return raw;
}

int16_t Read_Accel_Z(void) {
  uint8_t hi, lo;
  int16_t raw;
  I2C1->CR2 &= ~I2C_CR2_RD_WRN;                   // WRITE mode
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |=  (1 << I2C_CR2_NBYTES_Pos);
  I2C1->CR2 &= ~I2C_CR2_SADD;
  I2C1->CR2 |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos + 1));
  I2C1->CR2 |=  I2C_CR2_START;
  while (!(I2C1->ISR & I2C_ISR_TXIS));
  I2C1->TXDR = ACCEL_ZOUT_H;
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |=  I2C_ICR_STOPCF;

  I2C1->CR2 |=  I2C_CR2_RD_WRN;                   // READ mode
  I2C1->CR2 &= ~I2C_CR2_NBYTES;
  I2C1->CR2 |=  (2 << I2C_CR2_NBYTES_Pos);
  I2C1->CR2 &= ~I2C_CR2_SADD;
  I2C1->CR2 |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos + 1));
  I2C1->CR2 |=  (I2C_CR2_START | I2C_CR2_AUTOEND);
  while (!(I2C1->ISR & I2C_ISR_RXNE));
  hi = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_RXNE));
  lo = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_STOPF));
  I2C1->ICR |=  I2C_ICR_STOPCF;

  raw = (int16_t)((hi << 8) | lo);
  return raw;
}


void I2c_Config(void) {
  // Enable GPIOB clock
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  // Set PB6 and PB9 to Alternate Function mode
  GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE9);
  GPIOB->MODER |=  (GPIO_MODER_MODE6_1 | GPIO_MODER_MODE9_1); // AF mode (10)
  // Set output type to open-drain
  GPIOB->OTYPER |= (GPIO_OTYPER_OT6 | GPIO_OTYPER_OT9);
  // Disable internal pull-up/pull-down (external PU )
  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD9);
  // Set high speed
  GPIOB->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED6_Pos) |
              (3 << GPIO_OSPEEDR_OSPEED9_Pos);
  // Set alternate function 4 (AF4) for PB6 and PB9
  GPIOB->AFR[0] &= ~(0xF << GPIO_AFRL_AFSEL6_Pos);
  GPIOB->AFR[0] |=  (0x4 << GPIO_AFRL_AFSEL6_Pos);
  GPIOB->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL9_Pos);
  GPIOB->AFR[1] |=  (0x4 << GPIO_AFRH_AFSEL9_Pos);
  // Configure I2C
  RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;  // enable I2C bus clock
  I2C1->CR1   &= ~( I2C_CR1_PE );      // put I2C into reset (release SDA, SCL)
  I2C1->CR1   &= ~( I2C_CR1_ANFOFF );    // filters: enable analog
  I2C1->CR1   &= ~( I2C_CR1_DNF );       // filters: disable digital
  I2C1->TIMINGR = 0x00100D14;            // 4 MHz SYSCLK timing from CubeMX
  I2C1->CR2   |=  ( I2C_CR2_AUTOEND );   // auto send STOP after transmission
  I2C1->CR2   &= ~( I2C_CR2_ADD10 );     // 7-bit address mode
  I2C1->CR1   |=  ( I2C_CR1_PE );        // enable I2C
}
int16_t  Read_Gyro_Y(void){
  uint8_t hi, lo;
  int16_t raw;
  I2C1->CR2   &= ~I2C_CR2_RD_WRN;                     // WRITE mode
  I2C1->CR2   &= ~I2C_CR2_NBYTES;                     // clear NBYTES
  I2C1->CR2   |=  (1 << I2C_CR2_NBYTES_Pos);         // NBYTES = 1
  I2C1->CR2   &= ~I2C_CR2_SADD;                       // clear slave addr
  I2C1->CR2   |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos+1)); // set MPU address
  I2C1->CR2   |=  I2C_CR2_START;                      // START condition
  while (!(I2C1->ISR & I2C_ISR_TXIS));                // wait TXDR ready
  I2C1->TXDR = GYRO_YOUT_H;                              // send register index
  while (!(I2C1->ISR & I2C_ISR_STOPF));               // wait for STOP
  I2C1->ICR |=  I2C_ICR_STOPCF;                       // clear STOP flag
  I2C1->CR2   |=  I2C_CR2_RD_WRN;                     // READ mode
  I2C1->CR2   &= ~I2C_CR2_NBYTES;                     // clear NBYTES
  I2C1->CR2   |=  (2 << I2C_CR2_NBYTES_Pos);         // NBYTES = 2
  I2C1->CR2   &= ~I2C_CR2_SADD;                       // clear slave addr
  I2C1->CR2   |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos+1)); // set MPU address
  I2C1->CR2   |=  (I2C_CR2_START | I2C_CR2_AUTOEND);   // START + AUTOEND
  while (!(I2C1->ISR & I2C_ISR_RXNE));                // wait for first byte
  hi = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_RXNE));                // wait for second
  lo = I2C1->RXDR;
  while (!(I2C1->ISR & I2C_ISR_STOPF));               // wait for STOP
  I2C1->ICR |=  I2C_ICR_STOPCF;                       // clear STOP flag
  /* combine MSB/LSB into signed 16-bit */
  raw = (int16_t)((hi << 8) | lo);
  return raw;
}

void MPU6050_Wake(void) {
   /* write 0x00 → PWR_MGMT_1 (0x6B) */
   while (I2C1->ISR & I2C_ISR_BUSY) {}
   I2C1->CR2   &= ~I2C_CR2_RD_WRN;                     // WRITE
   I2C1->CR2   &= ~I2C_CR2_NBYTES;
   I2C1->CR2   |=  (2 << I2C_CR2_NBYTES_Pos);         // reg + data
   I2C1->CR2   &= ~I2C_CR2_SADD;
   I2C1->CR2   |=  (MPU6050_ADDR << (I2C_CR2_SADD_Pos+1));
   I2C1->CR2   |=  (I2C_CR2_START | I2C_CR2_AUTOEND);
   while (!(I2C1->ISR & I2C_ISR_TXIS));
   I2C1->TXDR  = MPU6050_PWR_MGMT1;
   while (!(I2C1->ISR & I2C_ISR_TXIS));
   I2C1->TXDR  = 0x00;
   while (!(I2C1->ISR & I2C_ISR_STOPF));
   I2C1->ICR =  I2C_ICR_STOPCF;
}
