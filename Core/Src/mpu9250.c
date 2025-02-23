/*
 * mpu9250.c
 *
 *  Created on: Nov 19, 2024
 *      Author: abhir
 */

#include <math.h>
#include "mpu9250.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

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
//
//	// magnetometer setup
//	mpu9250_write_reg(0x6A, 0x20);
//	mpu9250_write_reg(0x24, 0x0D);
//	mpu9250_write_reg(0x25, 0x8C);
//	mpu9250_write_reg(0x26, 0x03);
}

void mpu9250_init_ak8963()
{
    uint8_t calibData[3]; // buffer for factory calibration data

    mpu9250_write_reg(0x27, 0x00);  // disable I2C_SLV0_CTRL temporarily

    mpu9250_write_reg(0x25, 0x0C);
    // I2C_SLV0_REG (0x26): Point to AK8963_CNTL (0x0A)
    mpu9250_write_reg(0x26, 0x0A);
    // I2C_SLV0_DO (0x63): Data to write: 0x00 (power down)
    mpu9250_write_reg(0x63, 0x00);
    // I2C_SLV0_CTRL (0x27): Enable 1-byte write (0x80 | 1)
    mpu9250_write_reg(0x27, 0x81);
    HAL_Delay(10);

    mpu9250_write_reg(0x63, 0x0F);
    mpu9250_write_reg(0x27, 0x81);
    HAL_Delay(10);

    mpu9250_write_reg(0x25, 0x0C | 0x80);
    // Set I2C_SLV0_REG to AK8963_ASAX (starting register for calibration data)
    mpu9250_write_reg(0x26, 0x10);
    // Enable reading 3 bytes (0x80 | 3)
    mpu9250_write_reg(0x27, 0x83);
    HAL_Delay(10);

    mpu9250_read_reg(0x49, calibData, 3);

    mag_calibration_data.calibData1 = (((float)calibData[0] - 128.0f) / 256.0f) + 1.0f;
    mag_calibration_data.calibData2 = (((float)calibData[1] - 128.0f) / 256.0f) + 1.0f;
    mag_calibration_data.calibData3 = (((float)calibData[2] - 128.0f) / 256.0f) + 1.0f;

    mpu9250_write_reg(0x25, 0x0C);
    mpu9250_write_reg(0x26, 0x0A);
    mpu9250_write_reg(0x63, 0x00);  // Power down command
    mpu9250_write_reg(0x27, 0x81);
    HAL_Delay(10);

    uint8_t ctrlValue = (1 << 4) | 0;
    mpu9250_write_reg(0x63, ctrlValue);
    mpu9250_write_reg(0x27, 0x81);
    HAL_Delay(10);

    // ---- Step 6. Restore Automatic Continuous Reading ----
    // Reconfigure the I2C slave to read 7 bytes from the magnetometer starting at register 0x03 (HXL)
    mpu9250_write_reg(0x25, 0x0C | 0x80); // Set to read mode
    mpu9250_write_reg(0x26, 0x03);                   // Start at HXL register
    mpu9250_write_reg(0x27, 0x87);                   // Enable reading 7 bytes (0x80 | 7)
}



void mpu9250_getRawAngle()
{
	  uint8_t imu_data[6];

	  mpu9250_read_reg(59, imu_data, sizeof(imu_data));
	  imu_raw_data.accel_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.accel_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.accel_z = ((int16_t)imu_data[4]<<8) | imu_data[5];

	  imu_processed_data.accel_x = (float)imu_raw_data.accel_x/4096.0;
	  imu_processed_data.accel_y = (float)imu_raw_data.accel_y/4096.0;
	  imu_processed_data.accel_z = (float)imu_raw_data.accel_z/4096.0;
//	  imu_processed_data.accel_z -= 4;	//offset AccZ to be around 0

	  mpu9250_read_reg(67, imu_data, sizeof(imu_data));
	  imu_raw_data.gyro_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.gyro_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.gyro_z = ((int16_t)imu_data[4]<<8) | imu_data[5];

	  imu_processed_data.gyro_x = (float)imu_raw_data.gyro_x/65.5;
	  imu_processed_data.gyro_y = (float)imu_raw_data.gyro_y/65.5;
	  imu_processed_data.gyro_z = (float)imu_raw_data.gyro_z/65.5;
//	  imu_processed_data.gyro_x -= 4;	//offset GyroX to be around 0
//	  imu_processed_data.gyro_y += 20;	//offset GyroY to be around 0
//	  imu_processed_data.gyro_z += 5;	//offset GyroZ to be around 0

//	  mpu9250_read_reg(0x49, imu_data, sizeof(imu_data));
//	  imu_raw_data.mag_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
//	  imu_raw_data.mag_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
//	  imu_raw_data.mag_z = ((int16_t)imu_data[4]<<8) | imu_data[5];

	  imu_angles.roll=atan(imu_processed_data.accel_y/sqrt((imu_processed_data.accel_x*imu_processed_data.accel_x)+(imu_processed_data.accel_z*imu_processed_data.accel_z)))*1/(3.142/180);
	  imu_angles.pitch=-atan(imu_processed_data.accel_x/sqrt((imu_processed_data.accel_y*imu_processed_data.accel_y)+(imu_processed_data.accel_z*imu_processed_data.accel_z)))*1/(3.142/180);
}


