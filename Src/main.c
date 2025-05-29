
#include "main.h"
#include "I2c.h"
#include "delay.h"
#include "lpuart.h"
#include "LEDs.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "motor.h"

#define ALPHA 95 // speed of returning to zero
#define ONE_MINUS_ALPHA (100 - ALPHA)
#define DT_MS 10
#define DT_DIVIDER (1000 / DT_MS)
#define DELTA_THRESHOLD 10

int32_t angle100 = 0;      // tilt angle in centidegrees
int32_t gyro_bias = 0;
int tick = 0;

void SystemClock_Config(void);

#define KP 15   // proportional gain
#define KD 1    // derivative gain
#define KI 0    // optional, start with 0

int32_t prev_error = 0;
int32_t integral = 0;

int main(void)
{
    char buf[32];
    HAL_Init();
    SystemClock_Config();
    I2c_Config();
    LPUART_init();
    MPU6050_Wake();
    motor_init();

    // --- Bias Calibration ---
    int32_t bias_sum = 0;
    for (int i = 0; i < 100; i++) {
        bias_sum += Read_Gyro_Y();
        delay_us(2000);
    }
    gyro_bias = bias_sum / 100;

    // --- Main Loop ---
    LPUART_ESC_Print("2J", "");
    LPUART_ESC_Print("H", "");

    while (1) {
        // Gyro: angular rate in °/s × 100
        int16_t gyro_raw = Read_Gyro_Y() - gyro_bias;
        int32_t rate100 = (int32_t)gyro_raw * 100 / 131;
        int32_t gyro_delta = rate100 / DT_DIVIDER;  // Δangle over 20ms

        // Accelerometer: compute tilt angle from ay and az
        int16_t ax = Read_Accel_X();
        int16_t ay = Read_Accel_Y();
        int16_t az = Read_Accel_Z();
        int32_t accel_angle100 = compute_accel_angle100(ax, ay, az);  // centidegrees

        // Complementary filter
        angle100 = (ALPHA * (angle100 + gyro_delta) + ONE_MINUS_ALPHA * accel_angle100) / 100;

//        // tilt limit
//        if (abs(angle100) > 4000) {  // 40°
//            set_motor_speed(0);      // shut down motors
//            continue;                // skip rest of loop
//        }

        // --- PID Control ---
        int32_t error = 0 - angle100; // Target is 0
        integral += error;
        int32_t derivative = error - prev_error;
        prev_error = error;

        if (integral > 5000) integral = 5000;
        if (integral < -5000) integral = -5000;

        // PID output
        int32_t output = KP * error + KD * derivative + KI * integral;

        if (output > 50000) {
            output = 1000;
        } else if (output < -50000) {
            output = -1000;
        } else {
            output = (output * 1000) / 50000;
        }

        set_motor_speed(output);

         //print to lpuart
        tick++;
        if (tick >= 20) {  // every 10 * 20ms = 200ms
            tick = 0;
            int32_t whole = angle100 / 100;
            int32_t frac  = abs(angle100 % 100);
            snprintf(buf, sizeof(buf), "Angle: %4ld.%02ld deg\r\n", whole, frac);
            LPUART_ESC_Print("2J", "");
            LPUART_ESC_Print("H", "");
            LPUART_print(buf);


            char out_buf[32];
			snprintf(out_buf, sizeof(out_buf), "Output: %ld\r\n", output);
			LPUART_ESC_Print("H", "");
			LPUART_print(out_buf);
        }

        delay_us(DT_MS * 1000);
    }

}



int32_t compute_accel_angle100(int16_t ax, int16_t ay, int16_t az)
{
    // Normalize to g (assuming ±2g full scale)
    float axf = (float)ax / 16384.0f;
    float ayf = (float)ay / 16384.0f;
    float azf = (float)az / 16384.0f;

    // PITCH: forward-back tilt, since X changes most
    float angle_rad = atan2f(axf, sqrtf(ayf * ayf + azf * azf));
    return (int32_t)(angle_rad * 180.0f / M_PI * 100.0f);  // centidegrees
}





















void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
