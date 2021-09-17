/*
 * bme280_app.h
 *
 *  Created on: Sep 16, 2021
 *      Author: Jeff
 */

#ifndef INC_BME280_APP_H_
#define INC_BME280_APP_H_

#include "bme280.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "stm32l5xx_hal.h"

int8_t bme280_app_init(void);
void user_delay_us(uint32_t period, void *intf_ptr);
BME280_INTF_RET_TYPE user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BME280_INTF_RET_TYPE user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

#endif /* INC_BME280_APP_H_ */
