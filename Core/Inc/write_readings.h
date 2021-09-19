/*
 * write_readings.h
 *
 *  Created on: Sep 19, 2021
 *      Author: Jeff
 */

#ifndef INC_WRITE_READINGS_H_
#define INC_WRITE_READINGS_H_
#include <stdio.h>

int WriteTemperature(int32_t temperature, char* wr_buf, int len);
int WriteHumidity(int32_t humidity, char* wr_buf, int len);
int WritePressure(int32_t pressure, char* wr_buf, int len);

#endif /* INC_WRITE_READINGS_H_ */
