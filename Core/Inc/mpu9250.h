/*
 * mpu9250.h
 *
 *  Created on: Nov 19, 2024
 *      Author: abhir
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

// MPU-9250 register map
#define MPU9250_CONFIG 0x1A
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_GYRO_CONFIG 0x1B

#define MPU9250_USER_CTRL 0x6A
#define MPU9250_I2C_MST_CTRL 0x24
#define MPU9250_I2C_SLV0_ADDR   0x25
#define MPU9250_I2C_SLV0_REG    0x26

#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_EXT_SENS_DATA_00 0x49

#define MPU9250_I2C_SLV0_CTRL   0x27
#define MPU9250_I2C_SLV0_DO     0x63

// AK8963 (magnetometer) register map
#define AK8963_I2C_ADDR         0x0C   // 7-bit address
#define AK8963_WHO_AM_I         0x00
#define AK8963_INFO             0x01
#define AK8963_ST1              0x02
#define AK8963_HXL              0x03
#define AK8963_HXH              0x04
#define AK8963_HYL              0x05
#define AK8963_HYH              0x06
#define AK8963_HZL              0x07
#define AK8963_HZH              0x08
#define AK8963_ST2              0x09
#define AK8963_CNTL1            0x0A
#define AK8963_CNTL2            0x0B
#define AK8963_ASAX             0x10   // X-axis sensitivity adjustment
#define AK8963_ASAY             0x11
#define AK8963_ASAZ             0x12

// AK8963 control values
#define AK8963_POWER_DOWN       0x00
#define AK8963_FUSE_ROM_ACCESS  0x0F
#define AK8963_CNTL2_RESET      0x01
#define AK8963_CONTINUOUS_100HZ (0x06) // Continuous measurement mode 2 (100 Hz), 16-bit output

#define APPLY_SOFT_IRON_SCALING 1

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
    float mag_offX;
    float mag_offY;
    float mag_offZ;
    float mag_scaleX;
    float mag_scaleY;
    float mag_scaleZ;
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
