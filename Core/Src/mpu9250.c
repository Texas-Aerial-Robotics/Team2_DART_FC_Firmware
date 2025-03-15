/*
 * mpu9250.c
 *
 *  Created on: Nov 19, 2024
 *      Author: abhir
 */

#include <math.h>
#include "mpu9250.h"
#include "main.h"
#include "MahonyAHRS.h"

#define RAD_TO_DEG (180.0 / M_PI)

extern SPI_HandleTypeDef hspi1;

float quat[4];

// Declare global variables for the IMU data
IMU_RawData_t imu_raw_data;         // Instance of raw IMU data
IMU_ProcessedData_t imu_processed_data; // Instance of processed IMU data
IMU_Angles_t imu_angles;            // Instance of IMU angles
Mag_CalibData_t mag_calibration_data;

void mpu9250_write_reg(uint8_t reg, uint8_t data)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
	uint8_t temp_data = 0x80|reg;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi1, &temp_data , 1, 100);
	if(ret != HAL_OK)
		Error_Handler();
	ret = HAL_SPI_Receive(&hspi1, data, len, 100);
	if(ret != HAL_OK)
		Error_Handler();
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void mpu9250_setup()
{
	mpu9250_write_reg(26, 0x05);		//enable digital low pass filter
	mpu9250_write_reg(28, 0x10);		//set accelerometer full scale to +-8g
	mpu9250_write_reg(27, 0x08);		//set gyroscope full scale full scale to +-500deg
	mpu9250_calibrateGyro(1500);
	quat[0] = 1.0f;
	quat[1] = 0.0f;
	quat[2] = 0.0f;
	quat[3] = 0.0f;
}

void mpu9250_calibrateGyro(uint16_t numCalPoints)
{
    // Init
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        mpu9250_getRawData();
        x += imu_raw_data.gyro_x;
        y += imu_raw_data.gyro_y;
        z += imu_raw_data.gyro_z;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    imu_processed_data.gyro_offX = (float)x / (float)numCalPoints;
    imu_processed_data.gyro_offY = (float)y / (float)numCalPoints;
    imu_processed_data.gyro_offZ = (float)z / (float)numCalPoints;
}



void mpu9250_getRawData()
{
	  uint8_t imu_data[6];

	  mpu9250_read_reg(59, imu_data, sizeof(imu_data));
	  imu_raw_data.accel_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.accel_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.accel_z = ((int16_t)imu_data[4]<<8) | imu_data[5];

	  mpu9250_read_reg(67, imu_data, sizeof(imu_data));
	  imu_raw_data.gyro_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.gyro_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.gyro_z = ((int16_t)imu_data[4]<<8) | imu_data[5];
}

void mpu9250_getProcessedAngle()
{
	  mpu9250_getRawData();

	  imu_processed_data.accel_x = ((float)imu_raw_data.accel_x/4096.0) * 9.81;
	  imu_processed_data.accel_y = ((float)imu_raw_data.accel_y/4096.0) * 9.81;
	  imu_processed_data.accel_z = ((float)imu_raw_data.accel_z/4096.0) * 9.81;
//	  imu_processed_data.accel_z -= 4;	//offset AccZ to be around 0

	  imu_processed_data.gyro_x = ((float)imu_raw_data.gyro_x - imu_processed_data.gyro_offX)/65.5 * M_PI/180.0f;
	  imu_processed_data.gyro_y = ((float)imu_raw_data.gyro_y - imu_processed_data.gyro_offY)/65.5 * M_PI/180.0f;;
	  imu_processed_data.gyro_z = ((float)imu_raw_data.gyro_z - imu_processed_data.gyro_offZ)/65.5 * M_PI/180.0f;;

	  MahonyAHRSupdateIMU(quat, imu_processed_data.gyro_x, imu_processed_data.gyro_y, imu_processed_data.gyro_z, imu_processed_data.accel_x, imu_processed_data.accel_y ,imu_processed_data.accel_z);

	    /* Quternion to Euler */
	  float radPitch = asinf(-2.0f * (quat[1] * quat[3] - quat[0] * quat[2]));
	  float radRoll = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]), 2.0f * (quat[0] * quat[0] + quat[3] * quat[3]) - 1.0f);
	  float radYaw =  atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]), 2.0f * (quat[0] * quat[0] + quat[1] * quat[1]) - 1.0f);
	    /* Radian to Degree*/
	  imu_angles.pitch = radPitch * RAD_TO_DEG;
	  imu_angles.roll = radRoll * RAD_TO_DEG;
	  imu_angles.yaw = radYaw * RAD_TO_DEG;

}

