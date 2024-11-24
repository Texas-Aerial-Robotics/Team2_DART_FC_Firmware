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

/*
 * Q_angle: Process Noise for Angle
 * Q_bias: Process Noise for Bias
 * R_measure: Measurement Noise
 */
Kalman_t KalmanPitch = {
		.Q_angle = 0.01f,		//smaller value: slower updates & reliance on gyro, higher value: faster updates & reliance on accelerometer
		.Q_bias = 0.003f,		//increase value if bias changes frequently
		.R_measure = 0.03f		//smaller value: faster response & amplify noise, larger value: slower response & smoothened output
};
Kalman_t KalmanRoll = {
		.Q_angle = 0.01f,
		.Q_bias = 0.003f,
		.R_measure = 0.03f
};


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
		Error_Handler;
	ret = HAL_SPI_Receive(&hspi1, data, len, 100);
	if(ret != HAL_OK)
		Error_Handler;
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void mpu9250_setup()
{
	mpu9250_write_reg(26, 0x05);		//enable digital low pass filter
	mpu9250_write_reg(28, 0x10);		//set accelerometer full scale to +-8g
	mpu9250_write_reg(27, 0x08);		//set gyroscope full scale full scale to +-500deg

	//maybe ensure all IMU struct values are set to 0
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
	  imu_processed_data.accel_z -= 4;	//offset AccZ to be around 0

	  mpu9250_read_reg(67, imu_data, sizeof(imu_data));
	  imu_raw_data.gyro_x = ((int16_t)imu_data[0]<<8) | imu_data[1];
	  imu_raw_data.gyro_y = ((int16_t)imu_data[2]<<8) | imu_data[3];
	  imu_raw_data.gyro_z = ((int16_t)imu_data[4]<<8) | imu_data[5];

	  imu_processed_data.gyro_x = (float)imu_raw_data.gyro_x/65.5;
	  imu_processed_data.gyro_y = (float)imu_raw_data.gyro_y/65.5;
	  imu_processed_data.gyro_z = (float)imu_raw_data.gyro_z/65.5;
	  imu_processed_data.gyro_x -= 4;	//offset GyroX to be around 0
	  imu_processed_data.gyro_y += 20;	//offset GyroY to be around 0
	  imu_processed_data.gyro_z += 5;	//offset GyroZ to be around 0

	  imu_angles.roll=atan(imu_processed_data.accel_y/sqrt((imu_processed_data.accel_x*imu_processed_data.accel_x)+(imu_processed_data.accel_z*imu_processed_data.accel_z)))*1/(3.142/180);
	  imu_angles.pitch=-atan(imu_processed_data.accel_x/sqrt((imu_processed_data.accel_y*imu_processed_data.accel_y)+(imu_processed_data.accel_z*imu_processed_data.accel_z)))*1/(3.142/180);
}

double kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
	//Step 1: State Prediction
	double rate = newRate - Kalman->bias;	//newRate is the newest gyro measurement
	Kalman->angle += dt * rate;

	//Step 2: Covariance Prediction
	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[1][0] - Kalman->P[0][1] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	//Step 3: Innovation (calculate angle difference)
	double y = newAngle - Kalman->angle;

	//Step 4: Innovation covariance	(estimate error)
	double S = Kalman->P[0][0] + Kalman->R_measure;

	//Step 5: Kalman Gain
	double K[2];	//2x1 vector
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	//Step 6: Update Angle
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	//Step 7: Update Covariance
	double P00_temp = Kalman->P[0][0];
	double P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;

	return Kalman->angle;
}
