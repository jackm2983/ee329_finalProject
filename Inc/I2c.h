/*
* I2c.h
*
*  Created on: May 15, 2025
*      Author: scarh
*/
#ifndef INC_I2C_H_
#define INC_I2C_H_
#include "main.h"
#define MPU6050_ADDR (0x68)
#define GYRO_YOUT_H (0x45)
#define GYRO_YOUT_L (0x46)
#define MPU6050_PWR_MGMT1  (0x6B)
#define WAKE_TIMEOUT        100000U


int16_t  Read_Gyro_Y(void);
void I2c_Config(void);
void MPU6050_Wake(void);
int16_t Read_Accel_X(void);
int16_t Read_Accel_Z(void);
int16_t Read_Accel_Y(void);
//int16_t Read_Reg16(uint8_t reg_h);
//int MPU6050_Wake(void);
#endif /* INC_I2C_H_ */
