/*
 * MPU6500.h
 *
 *  Created on: Oct 2, 2024
 *      Author: Li Shi
 */

#ifndef IMU_DRIVERS_MPU6500_H_
#define IMU_DRIVERS_MPU6500_H_


#include "stm32h7xx_hal.h"  // Assuming you're using STM32H7

// MPU6500 register addresses
#define MPU6500_ACCEL_XOUT_H  0x3B
#define MPU6500_GYRO_XOUT_H   0x43
#define MPU6500_WHO_AM_I      0x75

// Function prototypes
void MPU6500_ReadIMU(SPI_HandleTypeDef* hspi, int16_t* accelData, int16_t* gyroData);


#endif /* IMU_DRIVERS_MPU6500_H_ */
