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

// Struct for Kalman filter outputs
typedef struct {
    double angle;          // Estimated angle
    double bias;           // Bias of the rate sensor
    double rate;           // Measured rate from the gyroscope

    double P[2][2];        // Error covariance matrix
    double Q_angle;        // Process noise variance for the angle
    double Q_bias;         // Process noise variance for the bias
    double R_measure;      // Measurement noise variance
} Kalman_t;

// Struct for orientation angles
typedef struct {
    volatile float roll;
    volatile float pitch;
} IMU_Angles_t;


void mpu9250_setup();
void mpu9250_write_reg(uint8_t reg, uint8_t data);
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len);

//update raw measurements from IMU
void mpu9250_getRawAngle();



#endif /* INC_MPU9250_H_ */
