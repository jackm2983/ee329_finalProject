
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




        // === PID CONTROL BLOCK ===
		static int32_t prev_cmd = 0;
		int32_t error = -angle100;
		int32_t error_deg = error / 100;  // Convert to degrees (was /10, too sensitive)
		int32_t rate_deg = rate100 / 100;

		// === Balanced Gains ===
		#define KP_BASE 8            // Higher base P gain for response
		#define KP_MAX 18            // Reasonable max P gain
		#define KD_BASE 2            // Base D gain
		#define KD_MAX 12            // Reasonable max D gain
		#define KI 0                 // Start with no integral term
		#define ANGLE_RAMP_START 50  // Start ramping after 0.5°
		#define ANGLE_RAMP_END 300   // Full gain past 3°
		#define RATE_LIMIT 500       // Reasonable rate limit
		#define MAX_OUT 60           // Higher max output
		#define MAX_ACCEL 15         // Faster acceleration limit
		#define DEADBAND 30          // Don't respond to tiny angles

		// === Deadband - ignore very small angles ===
		int32_t abs_angle = abs(error);
		if (abs_angle < DEADBAND) {
			error = 0;
			error_deg = 0;
		}

		// === Progressive P gain (starts low, ramps up) ===
		int32_t kp;
		if (abs_angle < ANGLE_RAMP_START) {
			kp = KP_BASE;
		} else if (abs_angle < ANGLE_RAMP_END) {
			int32_t ramp_progress = abs_angle - ANGLE_RAMP_START;
			int32_t ramp_range = ANGLE_RAMP_END - ANGLE_RAMP_START;
			kp = KP_BASE + ((KP_MAX - KP_BASE) * ramp_progress) / ramp_range;
		} else {
			kp = KP_MAX;
		}

		// === Progressive D gain (lower when upright) ===
		int32_t kd;
		if (abs_angle < 100) {  // Within 1 degree
			kd = KD_BASE;
		} else {
			kd = KD_BASE + ((KD_MAX - KD_BASE) * abs_angle) / 500; // Gradual ramp
		}

		// Clamp rate for sanity
		if (rate100 > RATE_LIMIT * 100) rate100 = RATE_LIMIT * 100;
		else if (rate100 < -RATE_LIMIT * 100) rate100 = -RATE_LIMIT * 100;

		// === Terms ===
		int32_t p_term = (kp * error_deg) / 5;   // Less scaling for more response
		int32_t d_term = -(kd * rate_deg) / 8;   // Less scaling for more damping

		// Simple integral term (optional, start with 0)
		integral += error_deg;
		if (integral > 1000) integral = 1000;
		else if (integral < -1000) integral = -1000;
		int32_t i_term = (KI * integral) / 100;

		// === Combine ===
		int32_t cmd = p_term + d_term + i_term;

		// Clamp output to much lower values
		if (cmd > MAX_OUT) cmd = MAX_OUT;
		else if (cmd < -MAX_OUT) cmd = -MAX_OUT;

		// Much gentler slew rate limit
		if (cmd > prev_cmd + MAX_ACCEL) cmd = prev_cmd + MAX_ACCEL;
		else if (cmd < prev_cmd - MAX_ACCEL) cmd = prev_cmd - MAX_ACCEL;

		prev_cmd = cmd;

		// Keep motor scaling reasonable
		set_motor_speed(cmd * 0.8);  // Less conservative scaling


















         //print to lpuart
        tick++;
        if (tick >= 20) {  // every 200ms
            tick = 0;

            LPUART_ESC_Print("2J", "");  // clear screen
            LPUART_ESC_Print("H", "");   // cursor home

            // Print all relevant state info
            snprintf(buf, sizeof(buf),
                "angle100: %ld\r\nrate100: %ld\r\naccel_angle100: %ld\r\ngyro_delta: %ld\r\n"
                "P: %ld  D: %ld\r\n",
                angle100, rate100, accel_angle100, gyro_delta,
                p_term, d_term
            );
            LPUART_print(buf);

            // Optional balance window detection
            if (abs(angle100) < 100 && abs(rate100) < 100) {
                LPUART_print(">> IN BALANCE WINDOW <<\r\n");
            }

        }




        delay_us(DT_MS * 100);
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
