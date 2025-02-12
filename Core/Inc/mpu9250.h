#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <math.h>

// IMU data structures
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_RawData_t;

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_ProcessedData_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} IMU_Angles_t;

// EKF structure
typedef struct {
    float q[4];                 // Quaternion [w, x, y, z]
    float P[6][6];              // Covariance matrix
    float Q[6][6];              // Process noise covariance
    float R[3][3];              // Measurement noise covariance
    float dt;                   // Time step
} EKF_t;

// Extern declarations for global instances
extern IMU_RawData_t imu_raw_data;
extern IMU_ProcessedData_t imu_processed_data;
extern IMU_Angles_t imu_angles;
extern EKF_t ekf;

// Function prototypes
void mpu9250_write_reg(uint8_t reg, uint8_t data);
void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
void mpu9250_init(void);
void mpu9250_get_data(void);
void mpu9250_update_filter(float dt);
void mpu9250_calibrate(unsigned int samples);
void quaternion_to_euler(float q[4], float* roll, float* pitch, float* yaw);

#endif // MPU9250_H
