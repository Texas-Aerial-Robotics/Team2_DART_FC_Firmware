/*
 * bmp388.h
 *
 *  Created on: Jan 19, 2025
 *      Author: abhir
 */

#ifndef INC_BMP388_H_
#define INC_BMP388_H_

#include <stdint.h>


/* struct containing important calibration data */
typedef struct {
  uint16_t par_t1;
  uint16_t par_t2;
  int8_t   par_t3;
  int16_t  par_p1;
  int16_t  par_p2;
  int8_t   par_p3;
  int8_t   par_p4;
  uint16_t par_p5;
  uint16_t par_p6;
  int8_t   par_p7;
  int8_t   par_p8;
  int16_t  par_p9;
  int8_t   par_p10;
  int8_t   par_p11;
} calib_data_t;

typedef struct {
    double par_t1;
    double par_t2;
    double par_t3;
    double par_p1;
    double par_p2;
    double par_p3;
    double par_p4;
    double par_p5;
    double par_p6;
    double par_p7;
    double par_p8;
    double par_p9;
    double par_p10;
    double par_p11;
    double t_lin; // intermediate value for computations later
} quantized_calib_data_t;

typedef struct {
	volatile uint32_t temperature; // later used for compensated pressure
	volatile uint32_t pressure;
} BMP388_RawData_t;

typedef struct {
	volatile double temperature; // later used for compensated pressure
	volatile double pressure;
} BMP388_ProcessedData_t;


void bmp388_setup();
void bmp388_write_reg(uint8_t reg, uint8_t data);
void bmp388_read_reg(uint8_t reg, uint8_t *data, uint8_t len);
void bmp388_quantize_calibration();
void bmp388_read_raw_data();
void bmp388_getData();

#endif /* INC_BMP388_H_ */
