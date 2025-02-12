/*
 * bmp388.c
 *
 *  Created on: Jan 19, 2025
 *      Author:
 */

#include "bmp388.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;
calib_data_t bmp388_calib;
quantized_calib_data_t quantized_bmp388_calib;
BMP388_RawData_t bmp388_rawData;
BMP388_ProcessedData_t bmp388_processedData;


// quantized values directly from the datasheet
void bmp388_quantize_calibration()
{
  quantized_bmp388_calib.par_t1 = (float)bmp388_calib.par_t1 * (1 << 8);     // T1 scaling
  quantized_bmp388_calib.par_t2 = (float)bmp388_calib.par_t2 / (1 << 30);    // T2 scaling
  quantized_bmp388_calib.par_t3 = (float)bmp388_calib.par_t3 / (1ULL << 48); // T3 scaling

  quantized_bmp388_calib.par_p1 = ((float)bmp388_calib.par_p1 - (1 << 14)) / (1 << 20); // P1 scaling
  quantized_bmp388_calib.par_p2 = ((float)bmp388_calib.par_p2 - (1 << 14)) / (1 << 29); // P2 scaling
  quantized_bmp388_calib.par_p3 = (float)bmp388_calib.par_p3 / (1ULL << 32);               // P3 scaling
  quantized_bmp388_calib.par_p4 = (float)bmp388_calib.par_p4 / (1ULL << 37);            // P4 scaling
  quantized_bmp388_calib.par_p5 = (float)bmp388_calib.par_p5 * (1 << 3);                // P5 scaling
  quantized_bmp388_calib.par_p6 = (float)bmp388_calib.par_p6 / (1 << 6);                // P6 scaling
  quantized_bmp388_calib.par_p7 = (float)bmp388_calib.par_p7 / (1 << 8);                // P7 scaling
  quantized_bmp388_calib.par_p8 = (float)bmp388_calib.par_p8 / (1 << 15);               // P8 scaling
  quantized_bmp388_calib.par_p9 = (float)bmp388_calib.par_p9 / (1ULL << 48);            // P9 scaling
  quantized_bmp388_calib.par_p10 = (float)bmp388_calib.par_p10 / (1ULL << 48);          // P10 scaling
  quantized_bmp388_calib.par_p11 = (float)bmp388_calib.par_p11 / 3.6893488147419103e19;          // P11 scaling 2^65
}

void bmp388_write_reg(uint8_t reg, uint8_t data)
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &reg, 1, 100);
  HAL_SPI_Transmit(&hspi2, &data, 1, 100);
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

void bmp388_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t temp_data = 0x80 | reg;
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi2, &temp_data, 1, 100);
  if (ret != HAL_OK)
    Error_Handler();
  ret = HAL_SPI_Receive(&hspi2, data, len, 100);
  if (ret != HAL_OK)
    Error_Handler();
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}
void bmp388_setup()
{
  uint8_t temp_data[22];
  bmp388_read_reg(0x31, temp_data, 21);
  uint8_t calib_data[21];
  for (int idx = 0; idx < 21; idx++)
      {
      	  calib_data[idx] = temp_data[idx + 1];
      }
  // register of calibration data, starts from 0x31 to 0x45 all details in datasheet
  bmp388_calib.par_t1 = (uint16_t)calib_data[0] | ((uint16_t)calib_data[1] << 8);
  bmp388_calib.par_t2 = (uint16_t)(calib_data[2] | ((uint16_t)calib_data[3] << 8));
  bmp388_calib.par_t3 = (int8_t)calib_data[4];
  bmp388_calib.par_p1 = (int16_t)(calib_data[5] | ((uint16_t)calib_data[6] << 8));
  bmp388_calib.par_p2 = (int16_t)(calib_data[7] | ((uint16_t)calib_data[8] << 8));
  bmp388_calib.par_p3 = (int8_t)calib_data[9];
  bmp388_calib.par_p4 = (int8_t)calib_data[10];
  bmp388_calib.par_p5 = (uint16_t)calib_data[11] | ((uint16_t)calib_data[12] << 8);
  bmp388_calib.par_p6 = (uint16_t)calib_data[13] | ((uint16_t)calib_data[14] << 8);
  bmp388_calib.par_p7 = (int8_t)calib_data[15];
  bmp388_calib.par_p8 = (int8_t)calib_data[16];
  bmp388_calib.par_p9 = (int16_t)(calib_data[17] | ((uint16_t)calib_data[18] << 8));
  bmp388_calib.par_p10 = (int8_t)calib_data[19];
  bmp388_calib.par_p11 = (int8_t)calib_data[20];

  bmp388_quantize_calibration(); // get the quantized calibration values for easier math later
  bmp388_write_reg(0x1B, 0x30);  // normal mode, temp pressure on by default with it
  bmp388_write_reg(0x1C, 0x03);  // bits 5-3 are temperature oversampling, and 2-0 are pressure oversampling
  bmp388_write_reg(0x1D, 0x02);  // ODR 50Hz, 20ms
  bmp388_write_reg(0x1F, 0x02);  // coefficient for IIR filter, ideally low value for the dart
}

void bmp388_read_raw_data()
{
  uint8_t data[6];
  uint8_t temp_data[7];
  bmp388_read_reg(0x04, temp_data, 6); // 0x04-0x06 pressure, 0x07-0x09 temperature
  for (int idx = 0; idx < 6; idx++)
      {
      	  data[idx] = temp_data[idx + 1];
      }

  // annoying bit shift because both are 20 bit values
  bmp388_rawData.pressure = (((int32_t)data[2] << 16) | ((int32_t)data[1] << 8) | ((int32_t)data[0])); // ask jason
  bmp388_rawData.temperature = (((int32_t)data[5] << 16) | ((int32_t)data[4] << 8) | ((int32_t)data[3]));

}

// the math performed in both functions are directly from the datasheet
static float bmp388_compensated_temperature(uint32_t raw_temp)
{
  float partial_data1 = (float)(raw_temp)-quantized_bmp388_calib.par_t1;
  float partial_data2 = partial_data1 * quantized_bmp388_calib.par_t2;
  quantized_bmp388_calib.t_lin = partial_data2 + (partial_data1 * partial_data1) * quantized_bmp388_calib.par_t3;
  return quantized_bmp388_calib.t_lin;
}

static float bmp388_compensated_pressure(uint32_t raw_pressure)
{
  float pressure;
  float partial_data1, partial_data2, partial_data3, partial_data4;
  float partial_out1, partial_out2;

  partial_data1 = quantized_bmp388_calib.par_p6 * quantized_bmp388_calib.t_lin;
  partial_data2 = quantized_bmp388_calib.par_p7 * quantized_bmp388_calib.t_lin * quantized_bmp388_calib.t_lin;
  partial_data3 = quantized_bmp388_calib.par_p8 * quantized_bmp388_calib.t_lin * quantized_bmp388_calib.t_lin * quantized_bmp388_calib.t_lin;
  partial_out1 = quantized_bmp388_calib.par_p5 + partial_data1 + partial_data2 + partial_data3;

  partial_data1 = quantized_bmp388_calib.par_p2 * quantized_bmp388_calib.t_lin;
  partial_data2 = quantized_bmp388_calib.par_p3 * quantized_bmp388_calib.t_lin * quantized_bmp388_calib.t_lin;
  partial_data3 = quantized_bmp388_calib.par_p4 * quantized_bmp388_calib.t_lin * quantized_bmp388_calib.t_lin * quantized_bmp388_calib.t_lin;
  partial_out2 = (float)raw_pressure * (quantized_bmp388_calib.par_p1 + partial_data1 + partial_data2 + partial_data3);

  partial_data1 = (float)raw_pressure * (float)raw_pressure;
  partial_data2 = quantized_bmp388_calib.par_p9 + quantized_bmp388_calib.par_p10 * quantized_bmp388_calib.t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = ((float)raw_pressure * (float)raw_pressure * (float)raw_pressure) * quantized_bmp388_calib.par_p11;

  pressure = partial_out1 + partial_out2 + partial_data3 + partial_data4;

  return pressure;
}

void bmp388_getData()
{
  bmp388_read_raw_data();
  bmp388_processedData.pressure = bmp388_compensated_pressure(bmp388_rawData.pressure);
  bmp388_processedData.temperature = bmp388_compensated_temperature(bmp388_rawData.temperature);
}
