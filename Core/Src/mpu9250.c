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

// Declare global variables for the IMU data
IMU_RawData_t imu_raw_data;         // Instance of raw IMU data
IMU_ProcessedData_t imu_processed_data; // Instance of processed IMU data
IMU_Angles_t imu_angles;            // Instance of IMU angles
Mag_CalibData_t mag_calibration_data;

float quat[4];

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
	mpu9250_write_reg(MPU9250_CONFIG, 0x05);		//enable digital low pass filter
	mpu9250_write_reg(MPU9250_ACCEL_CONFIG, 0x10);		//set accelerometer full scale to +-8g
	mpu9250_write_reg(MPU9250_GYRO_CONFIG, 0x08);		//set gyroscope full scale full scale to +-500deg


//
//	// magnetometer setup
	mpu9250_write_reg(MPU9250_USER_CTRL, 0x20);
	mpu9250_write_reg(MPU9250_I2C_MST_CTRL, 0x0D);
	mpu9250_write_reg(MPU9250_I2C_SLV0_ADDR, 0x8C);
	mpu9250_write_reg(MPU9250_I2C_SLV0_REG, 0x03);

	mpu9250_init_ak8963();
	mpu9250_calibrateIMU(1500);
	quat[0] = 1.0f;
	quat[1] = 0.0f;
	quat[2] = 0.0f;
	quat[3] = 0.0f;
}

void mpu9250_init_ak8963()
{
    uint8_t calibData[3]; // buffer for factory calibration data

    mpu9250_write_reg(MPU9250_I2C_SLV0_CTRL, 0x00);  // disable I2C_SLV0_CTRL temporarily

    mpu9250_write_reg(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    // I2C_SLV0_REG (0x26): Point to AK8963_CNTL (0x0A)
    mpu9250_write_reg(MPU9250_I2C_SLV0_REG, AK8963_CNTL1);
    // I2C_SLV0_DO (0x63): Data to write: 0x00 (power down)
    mpu9250_write_reg(MPU9250_I2C_SLV0_DO, AK8963_POWER_DOWN);
    // I2C_SLV0_CTRL (0x27): Enable 1-byte write (0x80 | 1)
    mpu9250_write_reg(MPU9250_I2C_SLV0_CTRL, 0x81);
    HAL_Delay(10);

    mpu9250_write_reg(MPU9250_I2C_SLV0_DO, AK8963_FUSE_ROM_ACCESS);
    mpu9250_write_reg(MPU9250_I2C_SLV0_CTRL, 0x81);
    HAL_Delay(10);

    mpu9250_write_reg(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR | 0x80);
    // Set I2C_SLV0_REG to AK8963_ASAX (starting register for calibration data)
    mpu9250_write_reg(MPU9250_I2C_SLV0_REG, AK8963_ASAX);
    // Enable reading 3 bytes (0x80 | 3)
    mpu9250_write_reg(MPU9250_I2C_SLV0_CTRL, 0x83);
    HAL_Delay(10);

    mpu9250_read_reg(MPU9250_EXT_SENS_DATA_00, calibData, 3);

    mag_calibration_data.calibData1 = (((float)calibData[0] - 128.0f) / 256.0f) + 1.0f;
    mag_calibration_data.calibData2 = (((float)calibData[1] - 128.0f) / 256.0f) + 1.0f;
    mag_calibration_data.calibData3 = (((float)calibData[2] - 128.0f) / 256.0f) + 1.0f;

    mpu9250_write_reg(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_write_reg(MPU9250_I2C_SLV0_REG, AK8963_CNTL1);
    mpu9250_write_reg(MPU9250_I2C_SLV0_DO, AK8963_POWER_DOWN);  // Power down command
    mpu9250_write_reg(MPU9250_I2C_SLV0_CTRL, 0x81);
    HAL_Delay(10);

    // MADE ONE CHANGE HERE --> ADDED CONTINUOUS 100 HZ SAMPLING MODE
    uint8_t ctrlValue = (1 << 4) | AK8963_CONTINUOUS_100HZ;
    mpu9250_write_reg(MPU9250_I2C_SLV0_DO, ctrlValue);
    mpu9250_write_reg(MPU9250_I2C_SLV0_CTRL, 0x81);
    HAL_Delay(10);

    // ---- Step 6. Restore Automatic Continuous Reading ----
    // Reconfigure the I2C slave to read 7 bytes from the magnetometer starting at register 0x03 (HXL)
    mpu9250_write_reg(MPU9250_I2C_SLV0_ADDR, AK8963_I2C_ADDR  | 0x80); // Set to read mode
    mpu9250_write_reg(MPU9250_I2C_SLV0_REG, AK8963_HXL);                   // Start at HXL register
    mpu9250_write_reg(MPU9250_I2C_SLV0_CTRL, 0x87);                   // Enable reading 7 bytes (0x80 | 7)
}

void mpu9250_calibrateIMU(uint16_t numCalPoints)
{
    // Init
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

	int32_t max_x = INT32_MAX,min_x = INT32_MIN;
	int32_t max_y = INT32_MAX, min_y = INT32_MIN;
	int32_t max_z = INT32_MAX, min_z = INT32_MIN;

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

        // Used for magnetometer hard-iron/soft-iron scaling
        if (mag_x > max_x) max_x = imu_raw_data.mag_x;
		if (mag_x < min_x) min_x = imu_raw_data.mag_x;
		if (mag_y > max_y) max_y = imu_raw_data.mag_y;
		if (mag_y < min_y) min_y = imu_raw_data.mag_y;
		if (mag_z > max_z) max_z = imu_raw_data.mag_z;
		if (mag_z < min_z) min_z = imu_raw_data.mag_z;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    imu_processed_data.gyro_offX = (float)x / (float)numCalPoints;
    imu_processed_data.gyro_offY = (float)y / (float)numCalPoints;
    imu_processed_data.gyro_offZ = (float)z / (float)numCalPoints;

    imu_processed_data.mag_offX = (float)(max_x + min_x) / 2.0f;
    imu_processed_data.mag_offY = (float)(max_y + min_y) / 2.0f;
    imu_processed_data.mag_offZ = (float)(max_z + min_z) / 2.0f;

    if(APPLY_SOFT_IRON_SCALING) {
    	float half_x = (float)(max_x - min_x) / 2.0f;
		float half_y = (float)(max_y - min_y) / 2.0f;
		float half_z = (float)(max_z - min_z) / 2.0f;

		float r_max = fmaxf(fmaxf(half_x, half_y), half_z);

		calib.scale_x = r_max / half_x;
		calib.scale_y = r_max / half_y;
		calib.scale_z = r_max / half_z;
    } else {
    	imu_processed_data.mag_scaleX = 1.0f;
		imu_processed_data.mag_scaleY = 1.0f;
		imu_processed_data.mag_scaleZ = 1.0f;
    }
}


void mpu9250_getRawData()
{
	  uint8_t imu_data[6];

	  // addr # 59
	  mpu9250_read_reg(MPU9250_ACCEL_XOUT_H, imu_data, sizeof(imu_data));
	  imu_raw_data.accel_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.accel_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.accel_z = ((int16_t)imu_data[4]<<8) | imu_data[5];

	  // addr # 67
	  mpu9250_read_reg(MPU9250_GYRO_XOUT_H, imu_data, sizeof(imu_data));
	  imu_raw_data.gyro_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.gyro_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.gyro_z = ((int16_t)imu_data[4]<<8) | imu_data[5];

      mpu9250_read_reg(MPU9250_EXT_SENS_DATA_00, imu_data, sizeof(imu_data));
	  imu_raw_data.mag_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.mag_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.mag_z = ((int16_t)imu_data[4]<<8) | imu_data[5];
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

	  imu_processed_data.mag_x =  (((float)imu_raw_data.mag_x * mag_calibration_data.calibData1) - imu_processed_data.mag_offX) * imu_processed_data.mag_scaleX;
	  imu_processed_data.mag_y =  (((float)imu_raw_data.mag_y * mag_calibration_data.calibData2) - imu_processed_data.mag_offY) * imu_processed_data.mag_scaleY;
	  imu_processed_data.mag_z =  (((float)imu_raw_data.mag_z * mag_calibration_data.calibData3) - imu_processed_data.mag_offZ) * imu_processed_data.mag_scaleZ;

	  MahonyAHRSupdate(quat, imu_processed_data.gyro_x, imu_processed_data.gyro_y, imu_processed_data.gyro_z, imu_processed_data.accel_x,
			  imu_processed_data.accel_y ,imu_processed_data.accel_z, imu_processed_data.mag_x, imu_processed_data.mag_y, imu_processed_data.mag_z);

	    /* Quternion to Euler */
	  float radPitch = asinf(-2.0f * (quat[1] * quat[3] - quat[0] * quat[2]));
	  float radRoll = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]), 2.0f * (quat[0] * quat[0] + quat[3] * quat[3]) - 1.0f);
	  float radYaw =  atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]), 2.0f * (quat[0] * quat[0] + quat[1] * quat[1]) - 1.0f);

	  	/* Radian to Degree*/
	  imu_angles.pitch = radPitch * RAD_TO_DEG;
	  imu_angles.roll = radRoll * RAD_TO_DEG;
	  imu_angles.yaw = radYaw * RAD_TO_DEG;
}

