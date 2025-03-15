/*
 * mpu9250.h
 *
 *  Created on: Nov 19, 2024
 *      Author: abhir
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

#include <stdint.h>

// Struct for raw IMU data
typedef struct {
    volatile int16_t accel_x;
    volatile int16_t accel_y;
    volatile int16_t accel_z;
    volatile int16_t gyro_x;
    volatile int16_t gyro_y;
    volatile int16_t gyro_z;
    volatile int16_t mag_x;
    volatile int16_t mag_y;
    volatile int16_t mag_z;
} IMU_RawData_t;

// Struct for processed IMU data
typedef struct {
	float gyro_offX;
	float gyro_offY;
	float gyro_offZ;
    volatile float accel_x;
    volatile float accel_y;
    volatile float accel_z;
    volatile float gyro_x;
    volatile float gyro_y;
    volatile float gyro_z;
    volatile float mag_x;
    volatile float mag_y;
    volatile float mag_z;
} IMU_ProcessedData_t;

typedef struct {
	float calibData1;
	float calibData2;
	float calibData3;
} Mag_CalibData_t;

// Struct for orientation angles
typedef struct {
    volatile float roll;
    volatile float pitch;
    volatile float yaw;
} IMU_Angles_t;


void mpu9250_setup();
void mpu9250_write_reg(uint8_t reg, uint8_t data);
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len);

void mpu9250_calibrateGyro(uint16_t numCalPoints);

//update raw measurements from IMU
void mpu9250_getRawData();
void mpu9250_getProcessedAngle();


#endif /* INC_MPU9250_H_ */
