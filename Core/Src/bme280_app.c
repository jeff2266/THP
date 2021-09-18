/*
 * bme280_app.c
 *
 *  Created on: Sep 16, 2021
 *      Author: Jeff
 */

#include "bme280_app.h"

SPI_HandleTypeDef hspi3;

/* Sensor_0 interface over SPI with native chip select line */
uint8_t dev_addr = 0;
struct bme280_dev bme_device = {
	.intf_ptr = &dev_addr,
	.intf = BME280_SPI_INTF,
	.read = user_spi_read,
	.write = user_spi_write,
	.delay_us = user_delay_us,
};
struct bme280_data curr_bme_data;

int8_t bme280_app_init(void) {
	if (bme280_init(&bme_device) == BME280_OK) {
		bme_device.settings.osr_h = BME280_OVERSAMPLING_1X;
		bme_device.settings.osr_p = BME280_OVERSAMPLING_16X;
		bme_device.settings.osr_t = BME280_OVERSAMPLING_2X;
		bme_device.settings.filter = BME280_FILTER_COEFF_16;
		return bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &bme_device);
	}
	return BME280_E_COMM_FAIL;
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    /*
     * Return control or wait,
     * for a period amount of microseconds
     */
	osDelay(pdMS_TO_TICKS(period / 1000));
}

BME280_INTF_RET_TYPE user_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	HAL_StatusTypeDef rslt = BME280_OK;
    /*
     * The parameter intf_ptr can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */

    uint8_t ctrl_byte = 0b10000000 | reg_addr;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi3, &ctrl_byte, 1, 100) != HAL_OK) rslt = BME280_E_COMM_FAIL;
    if (HAL_SPI_Receive(&hspi3, reg_data, len, 100) != HAL_OK) rslt = BME280_E_COMM_FAIL;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

    return rslt;
}

BME280_INTF_RET_TYPE user_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    int8_t rslt = BME280_E_COMM_FAIL; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter intf_ptr can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */

    uint8_t *send_buf = pvPortMalloc((1 + len) * sizeof(uint8_t));
    if (send_buf == NULL) return rslt;

    send_buf[0] = reg_addr;

    for (uint8_t i = 0; i < len; i++)
    {
    	send_buf[i + 1] = reg_data[i];
    }
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(&hspi3, send_buf, 1 + len, 100 * len) == HAL_OK) rslt = BME280_OK;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

    vPortFree(send_buf);
    return rslt;
}
