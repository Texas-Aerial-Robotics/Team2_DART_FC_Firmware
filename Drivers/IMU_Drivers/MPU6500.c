#include "MPU6500.h"

void MPU6500_ReadIMU(SPI_HandleTypeDef* hspi, int16_t* accelData, int16_t* gyroData)
{
    uint8_t txBuffer[14];
    uint8_t rxBuffer[14];

    // Set the first byte to the register address for accelerometer and gyro data, with the read bit (0x80).
    txBuffer[0] = MPU6500_ACCEL_XOUT_H | 0x80;  // Setting the read bit

    // Pull CS low to select the IMU
    HAL_GPIO_WritePin(GPIOx, GPIO_PIN_CS, GPIO_PIN_RESET);

    // Send the register address and read the 14 bytes of data (6 bytes for accelerometer, 6 for gyroscope, 2 for temperature)
    HAL_SPI_TransmitReceive(hspi, txBuffer, rxBuffer, 14, HAL_MAX_DELAY);

    // Pull CS high to deselect the IMU
    HAL_GPIO_WritePin(GPIOx, GPIO_PIN_CS, GPIO_PIN_SET);

    // Combine high and low bytes for accelerometer (16-bit values)
    accelData[0] = (int16_t)((rxBuffer[1] << 8) | rxBuffer[2]);  // Accel X
    accelData[1] = (int16_t)((rxBuffer[3] << 8) | rxBuffer[4]);  // Accel Y
    accelData[2] = (int16_t)((rxBuffer[5] << 8) | rxBuffer[6]);  // Accel Z

    // Combine high and low bytes for gyroscope (16-bit values)
    gyroData[0] = (int16_t)((rxBuffer[9] << 8) | rxBuffer[10]);  // Gyro X
    gyroData[1] = (int16_t)((rxBuffer[11] << 8) | rxBuffer[12]); // Gyro Y
    gyroData[2] = (int16_t)((rxBuffer[13] << 8) | rxBuffer[14]); // Gyro Z
}
