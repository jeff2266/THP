/*
 * write_readings.c
 *
 *  Created on: Sep 19, 2021
 *      Author: Jeff
 */

#include "write_readings.h"

int WriteTemperature(int32_t temperature, char* wr_buf, int len) {
	// Temperature is Q30.2 [degC]
	if (wr_buf == NULL || len < 0) return -1;
	if (len == 0) return 0;

	int minChars = 4; // "x.xx"
	if (temperature < 0) minChars += 1; // '-'
	int ctInt = (temperature / 100);
	int tInt = ctInt;
	while (0 < (ctInt /= 10)) minChars++;
	if (len < minChars) return -1;

	int iBuf = 0;
	int tMant = (temperature % 100);
	if (temperature < 0) wr_buf[iBuf++] = '-';
	return sprintf(&(wr_buf[iBuf]), "%i.%i", tInt, tMant);
}

int WriteHumidity(int32_t humidity, char* wr_buf, int len) {
	// Humidity / 1024 -> adj_humidity is Q32.0 [%RH]
	if (wr_buf == NULL || len < 0) return -1;
	if (len == 0) return 0;

	int minChars = 1; // 'x'
	int ctInt = (humidity >> 10);
	int tInt = ctInt;
	while (0 < (ctInt /= 10)) minChars++;
	if (len < minChars) return -1;
	return sprintf(wr_buf, "%i", tInt);
}

int WritePressure(int32_t pressure, char* wr_buf, int len) {
	// Pressure / 256 -> adj_pressure is Q30.2 [hPa]
	if (wr_buf == NULL || len < 0) return -1;
	if (len == 0) return 0;

	int minChars = 4; // "x.xx"
	int ctInt = (pressure >> 8);
	int tInt = ctInt;

	ctInt /= 100;
	while (0 < (ctInt /= 10)) minChars++;
	if (len < minChars) return -1;

	int tMant = (tInt % 100);
	tInt = (tInt / 100);
	return sprintf(wr_buf, "%i.%i", tInt, tMant);
}
