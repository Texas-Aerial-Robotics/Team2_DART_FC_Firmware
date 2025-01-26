/*
 * bmp388.h
 *
 *  Created on: Jan 19, 2025
 *      Author: abhir
 */

#ifndef INC_BMP388_H_
#define INC_BMP388_H_

void bmp388_setup();
void bmp388_write_reg(uint8_t reg, uint8_t data);
void bmp388_read_reg(uint8_t reg, uint8_t *data, uint8_t len);

#endif /* INC_BMP388_H_ */
