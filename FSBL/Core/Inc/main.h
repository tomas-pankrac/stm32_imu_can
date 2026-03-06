/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined ( __ICCARM__ )
#  define CMSE_NS_CALL  __cmse_nonsecure_call
#  define CMSE_NS_ENTRY __cmse_nonsecure_entry
#else
#  define CMSE_NS_CALL  __attribute((cmse_nonsecure_call))
#  define CMSE_NS_ENTRY __attribute((cmse_nonsecure_entry))
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Fusion.h"
#include "can_functions.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* Function pointer declaration in non-secure*/
#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif

/* typedef for non-secure callback functions */
typedef funcptr funcptr_NS;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define LSM6DSO_FUNC_CFG_ACCESS              0x1U
#define LSM6DSO_WHO_AM_I                     0x0FU
#define LSM6DSO_CTRL1_XL                     0x10U
#define LSM6DSO_CTRL2_G                      0x11U
#define LSM6DSO_CTRL3_C                      0x12U
#define LSM6DSOX_CTRL10_C                    0x19U
#define LSM6DSO_STATUS_REG                   0x1EU
#define LSM6DSO_OUT_TEMP_L                   0x20U
#define LSM6DSO_OUT_TEMP_H                   0x21U
#define LSM6DSO_OUTX_L_G                     0x22U
#define LSM6DSO_OUTX_H_G                     0x23U
#define LSM6DSO_OUTY_L_G                     0x24U
#define LSM6DSO_OUTY_H_G                     0x25U
#define LSM6DSO_OUTZ_L_G                     0x26U
#define LSM6DSO_OUTZ_H_G                     0x27U
#define LSM6DSO_OUTX_L_A                     0x28U
#define LSM6DSO_OUTX_H_A                     0x29U
#define LSM6DSO_OUTY_L_A                     0x2AU
#define LSM6DSO_OUTY_H_A                     0x2BU
#define LSM6DSO_OUTZ_L_A                     0x2CU
#define LSM6DSO_OUTZ_H_A                     0x2DU
#define LSM6DSO_TIMESTAMP0                   0x40U

#define LIS2MDL_OFFSET_X_REG_L          0x45U
#define LIS2MDL_OFFSET_X_REG_H          0x46U
#define LIS2MDL_OFFSET_Y_REG_L          0x47U
#define LIS2MDL_OFFSET_Y_REG_H          0x48U
#define LIS2MDL_OFFSET_Z_REG_L          0x49U
#define LIS2MDL_OFFSET_Z_REG_H          0x4AU
#define LIS2MDL_WHO_AM_I                0x4FU
#define LIS2MDL_CFG_REG_A               0x60U
#define LIS2MDL_CFG_REG_B               0x61U
#define LIS2MDL_CFG_REG_C               0x62U
#define LIS2MDL_INT_THS_L_REG           0x65U
#define LIS2MDL_INT_THS_H_REG           0x66U
#define LIS2MDL_STATUS_REG              0x67U
#define LIS2MDL_OUTX_L_REG              0x68U
#define LIS2MDL_OUTX_H_REG              0x69U
#define LIS2MDL_OUTY_L_REG              0x6AU
#define LIS2MDL_OUTY_H_REG              0x6BU
#define LIS2MDL_OUTZ_L_REG              0x6CU
#define LIS2MDL_OUTZ_H_REG              0x6DU
#define LIS2MDL_TEMP_OUT_L_REG          0x6EU
#define LIS2MDL_TEMP_OUT_H_REG          0x6FU

typedef struct __attribute__((packed)) {
//  int16_t accel_x;
//  int16_t accel_y;
//  int16_t accel_z;
//
//  int16_t gyro_x;
//  int16_t gyro_y;
//  int16_t gyro_z;
//
//  int16_t mag_x;
//  int16_t mag_y;
//  int16_t mag_z;

  float_t euler_x;
  float_t euler_y;
  float_t euler_z;

  float_t leg_angles[12];

  float_t endpoints[4][3];
  float_t endpoints_velocity[4][3];

  uint32_t can_data_low[12];
  uint32_t can_data_high[12];

//  uint32_t can_data0;
//  uint32_t can_data1;

  int16_t mag_temp;

  uint32_t timestamp;

} TX_Data_t;





void platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void read_imu_fd(void *handle, uint8_t reg, uint8_t *buft, uint8_t *bufr, uint16_t len);
void read_mag_fd(void *handle, uint8_t reg, uint8_t *buft, uint8_t *bufr, uint16_t len);
void readout_imu(void* spi_handle, uint8_t* TX_Buffer, uint8_t* RX_Buffer, int16_t* accelerometer_data, int16_t* gyroscope_data, int16_t* magnetometer_data, uint32_t* previousTimestamp, uint32_t* timestamp, float_t* deltaTime, int16_t* mag_temp);
void initialize_imu_mag(void *handle, uint8_t *TX_Buffer, uint8_t *RX_Buffer);
void timer_wait(void);
void timer_wait2(uint16_t clks);
void initialize_ahrs(FusionBias* bias, FusionAhrs* ahrs);
void update_ahrs(FusionBias* bias, FusionAhrs* ahrs, int16_t accelerometer_data[], int16_t gyroscope_data[], int16_t magnetometer_data[], float_t euler_orientation[], float_t deltaTime);
void send_telemetry(void *handle, TX_Data_t* data, uint32_t data_size, uint8_t* TX_Buffer_1, uint8_t* RX_Buffer_1);
void receive_input_can_command(void *handle, uint8_t* TX_Buffer_1, uint8_t* RX_Buffer_1, CAN_Packet* can_packet);
void parse_input_can_command(uint8_t RX_Buffer_1[], CAN_Packet* can_packet);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
uint8_t poll_for_imu_new_data(void* spi_handle, uint8_t* TX_Buffer, uint8_t* RX_Buffer);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
