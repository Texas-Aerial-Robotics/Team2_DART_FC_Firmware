/*
 * mpu9250.c
 * Extended Kalman Filter Implementation for IMU Orientation Estimation
 */

#include <math.h>
#include "mpu9250.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

// IMU Data Structures
IMU_RawData_t imu_raw_data;
IMU_ProcessedData_t imu_processed_data;
IMU_Angles_t imu_angles;

// EKF Structure and Parameters
EKF_t ekf;

// Sensor calibration offsets (should be determined during initialization)
float gyro_bias[3] = {0};
float accel_bias[3] = {0};

// Private function prototypes
static void ekf_predict(float dt);
static void ekf_update(void);
void quaternion_to_euler(float q[4], float* roll, float* pitch, float* yaw);

void mpu9250_write_reg(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
    HAL_SPI_Transmit(&hspi1, &data, 1, 100);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void mpu9250_read_reg(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t temp_data = 0x80 | reg;
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &temp_data, 1, 100);
    HAL_SPI_Receive(&hspi1, data, len, 100);
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void mpu9250_init(void) {
    // Initialize EKF
    ekf.dt = 0.01f;  // Will be updated dynamically

    // Initial state (quaternion)
    ekf.q[0] = 1.0f;
    ekf.q[1] = 0.0f;
    ekf.q[2] = 0.0f;
    ekf.q[3] = 0.0f;

    // Initial covariance matrix
    for(int i = 0; i < 6; i++)
        ekf.P[i][i] = 1.0f;

    // Process noise covariance
    ekf.Q[0][0] = 0.0001f;  // Gyro noise
    ekf.Q[1][1] = 0.0001f;
    ekf.Q[2][2] = 0.0001f;
    ekf.Q[3][3] = 0.00001f; // Gyro bias noise
    ekf.Q[4][4] = 0.00001f;
    ekf.Q[5][5] = 0.00001f;

    // Measurement noise covariance
    ekf.R[0][0] = 0.1f;    // Accelerometer noise
    ekf.R[1][1] = 0.1f;
    ekf.R[2][2] = 0.1f;

    // IMU configuration
    mpu9250_write_reg(26, 0x05);  // DLPF_CFG = 5 (41Hz)
    mpu9250_write_reg(28, 0x10);  // ACCEL_FS_SEL = ±8g
    mpu9250_write_reg(27, 0x08);  // GYRO_FS_SEL = ±500°/s
}

void mpu9250_get_data(void) {
    uint8_t imu_data[14];

    // Read accelerometer and gyroscope data
    mpu9250_read_reg(59, imu_data, 14);

    // Process accelerometer data
    imu_raw_data.accel_x = (int16_t)((imu_data[0] << 8) | imu_data[1]);
    imu_raw_data.accel_y = (int16_t)((imu_data[2] << 8) | imu_data[3]);
    imu_raw_data.accel_z = (int16_t)((imu_data[4] << 8) | imu_data[5]);

    // Process gyroscope data
    imu_raw_data.gyro_x = (int16_t)((imu_data[8] << 8)  | imu_data[9]);
    imu_raw_data.gyro_y = (int16_t)((imu_data[10] << 8) | imu_data[11]);
    imu_raw_data.gyro_z = (int16_t)((imu_data[12] << 8) | imu_data[13]);

    // Convert to physical units
    imu_processed_data.accel_x = (imu_raw_data.accel_x / 4096.0f) - accel_bias[0];
    imu_processed_data.accel_y = (imu_raw_data.accel_y / 4096.0f) - accel_bias[1];
    imu_processed_data.accel_z = (imu_raw_data.accel_z / 4096.0f) - accel_bias[2];

    imu_processed_data.gyro_x = (imu_raw_data.gyro_x / 65.5f) * (M_PI / 180.0f) - gyro_bias[0];
    imu_processed_data.gyro_y = (imu_raw_data.gyro_y / 65.5f) * (M_PI / 180.0f) - gyro_bias[1];
    imu_processed_data.gyro_z = (imu_raw_data.gyro_z / 65.5f) * (M_PI / 180.0f) - gyro_bias[2];
}

void mpu9250_update_filter(float dt) {
    ekf.dt = dt;
    ekf_predict(dt);
    ekf_update();
    quaternion_to_euler(ekf.q, &imu_angles.roll, &imu_angles.pitch, &imu_angles.yaw);
}

static void ekf_predict(float dt) {
    // State transition matrix (F)
    float F[6][6] = {0};

    float wx = imu_processed_data.gyro_x;
    float wy = imu_processed_data.gyro_y;
    float wz = imu_processed_data.gyro_z;

    // Quaternion derivative
    float dq[4] = {
        0.5f * (-wx*ekf.q[1] - wy*ekf.q[2] - wz*ekf.q[3]),
        0.5f * ( wx*ekf.q[0] + wz*ekf.q[2] - wy*ekf.q[3]),
        0.5f * ( wy*ekf.q[0] - wz*ekf.q[1] + wx*ekf.q[3]),
        0.5f * ( wz*ekf.q[0] + wy*ekf.q[1] - wx*ekf.q[2])
    };

    // Update quaternion state
    ekf.q[0] += dq[0] * dt;
    ekf.q[1] += dq[1] * dt;
    ekf.q[2] += dq[2] * dt;
    ekf.q[3] += dq[3] * dt;

    // Normalize quaternion
    float norm = sqrtf(ekf.q[0]*ekf.q[0] + ekf.q[1]*ekf.q[1] +
                      ekf.q[2]*ekf.q[2] + ekf.q[3]*ekf.q[3]);
    for(int i = 0; i < 4; i++)
        ekf.q[i] /= norm;

    // Update covariance matrix: P = F*P*F^T + Q
    // (Implementation of matrix operations omitted for brevity)
}

static void ekf_update(void) {
    // Measurement model: predicted accelerations
    float g = 9.81f;
    float ax_pred = 2.0f*(ekf.q[1]*ekf.q[3] - ekf.q[0]*ekf.q[2]) * g;
    float ay_pred = 2.0f*(ekf.q[0]*ekf.q[1] + ekf.q[2]*ekf.q[3]) * g;
    float az_pred = (ekf.q[0]*ekf.q[0] - ekf.q[1]*ekf.q[1]
                    - ekf.q[2]*ekf.q[2] + ekf.q[3]*ekf.q[3]) * g;

    // Measurement residual
    float y[3] = {
        imu_processed_data.accel_x - ax_pred,
        imu_processed_data.accel_y - ay_pred,
        imu_processed_data.accel_z - az_pred
    };

    // Kalman gain and covariance update
    // (Implementation of matrix operations omitted for brevity)


    // After calculating residual y[3]:
    // 1. Compute Kalman gain K = P * H^T * (H * P * H^T + R)^-1
    // 2. Update state estimate
    // 3. Update covariance matrix P = (I - K*H)*P

//    // Placeholder implementation:
//    (void)ax_pred;  // Temporary silence warnings
//    (void)ay_pred;
//    (void)az_pred;

    // The code below deepseek generated later

    // 3. Simplified Kalman gain (example values)
	float K[4][3] = {{0.1, 0, 0},  // Gain for quaternion[0]
					 {0, 0.1, 0},  // Gain for quaternion[1]
					 {0, 0, 0.1},  // Gain for quaternion[2]
					 {0, 0, 0}};   // Gain for quaternion[3]

	// 4. Update quaternion state
	ekf.q[0] += K[0][0] * y[0] + K[0][1] * y[1] + K[0][2] * y[2];
	ekf.q[1] += K[1][0] * y[0] + K[1][1] * y[1] + K[1][2] * y[2];
	ekf.q[2] += K[2][0] * y[0] + K[2][1] * y[1] + K[2][2] * y[2];
	ekf.q[3] += K[3][0] * y[0] + K[3][1] * y[1] + K[3][2] * y[2];

	// 5. Renormalize quaternion
	float norm = sqrtf(ekf.q[0]*ekf.q[0] + ekf.q[1]*ekf.q[1] +
					  ekf.q[2]*ekf.q[2] + ekf.q[3]*ekf.q[3]);
	for (int i = 0; i < 4; i++) ekf.q[i] /= norm;

}

void quaternion_to_euler(float q[4], float* roll, float* pitch, float* yaw) {
    // Convert quaternion to Euler angles (ZYX convention)
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    *roll = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    *pitch = asinf(2*(q0*q2 - q3*q1));
    *yaw = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

    // Convert to degrees
    *roll *= (180.0f / M_PI);
    *pitch *= (180.0f / M_PI);
    *yaw *= (180.0f / M_PI);
}

void mpu9250_calibrate(unsigned int samples) {
    // Simple calibration routine to determine sensor biases
    float temp_gyro[3] = {0};
    float temp_accel[3] = {0};

    for(unsigned int i = 0; i < samples; i++) {
        mpu9250_get_data();
        temp_gyro[0] += imu_processed_data.gyro_x;
        temp_gyro[1] += imu_processed_data.gyro_y;
        temp_gyro[2] += imu_processed_data.gyro_z;

        temp_accel[0] += imu_processed_data.accel_x;
        temp_accel[1] += imu_processed_data.accel_y;
        temp_accel[2] += imu_processed_data.accel_z - 9.81f;

        HAL_Delay(10);
    }

    gyro_bias[0] = temp_gyro[0] / samples;
    gyro_bias[1] = temp_gyro[1] / samples;
    gyro_bias[2] = temp_gyro[2] / samples;

    accel_bias[0] = temp_accel[0] / samples;  // X-axis bias
    accel_bias[1] = temp_accel[1] / samples;  // Y-axis bias
    accel_bias[2] = (temp_accel[2] / samples) - 9.81f;  // Z-axis (remove gravity)
}
