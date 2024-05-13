/*
 * bmp390_stm32.c
 *
 *  Created on: Mar 27, 2024
 *      Author: SURFACE
 */
#include "bmp390_stm32.h"
static uint16_t settings_sel;
static struct bmp3_dev bmp390dev;
static struct bmp3_data data = { 0 };
static struct bmp3_settings settings = { 0 };
static struct bmp3_status status = { { 0 } };
static I2C_HandleTypeDef *bmp390_phi2c;

BMP3_INTF_RET_TYPE BMP390_Read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr){
	return HAL_I2C_Mem_Read(
			intf_ptr,
			(BMP3_ADDR_I2C_SEC << 1),
			reg_addr,
			I2C_MEMADD_SIZE_8BIT,
			read_data,
			len,
			BMP390_I2C_TIMEOUT);
}
BMP3_INTF_RET_TYPE BMP390_Write(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr){
	return HAL_I2C_Mem_Write(
			intf_ptr,
			(BMP3_ADDR_I2C_SEC << 1),
			reg_addr,
			I2C_MEMADD_SIZE_8BIT,
			read_data,
			len,
			BMP390_I2C_TIMEOUT
	);
}
void BMP390_DelayUs(uint32_t period, void *intf_ptr){
	/* bmp3.c only requires ms delay, so i'll just use millisecond delay function */
	uint32_t delay = period / 1000;
	HAL_Delay(delay);
}
void BMP390_Init(void){
	HAL_Delay(100);
	bmp390dev.chip_id = BMP390_CHIP_ID;
	bmp390dev.intf_ptr = bmp390_phi2c;
	bmp390dev.intf = BMP3_I2C_INTF;
	bmp390dev.read = BMP390_Read;
	bmp390dev.write = BMP390_Write;
	bmp390dev.delay_us = BMP390_DelayUs;

	// Force soft reset
	uint8_t buf = 0xB6;
	HAL_I2C_Mem_Write(bmp390dev.intf_ptr, BMP3_ADDR_I2C_SEC << 1, 0x7E, I2C_MEMADD_SIZE_8BIT, &buf, 1, 100);
	bmp3_init(&bmp390dev);

	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
	settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	settings.odr_filter.odr = BMP3_ODR_50_HZ;

	settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;
	bmp3_set_sensor_settings(settings_sel, &settings, &bmp390dev);

	settings.op_mode = BMP3_MODE_NORMAL;
	bmp3_set_op_mode(&settings, &bmp390dev);
}
HAL_StatusTypeDef BMP390_GetValue( int64_t *pTemperature, uint64_t *pPressure, uint32_t Timeout ){

	uint32_t 	tickstart = HAL_GetTick();

	while( bmp3_get_status(&status, &bmp390dev) != BMP3_OK ){
		if ( ( HAL_GetTick() - tickstart ) > Timeout )
			return HAL_TIMEOUT;
	}

	/* BMP3_OK; sensor ready */
	if ( bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &bmp390dev) ){
		return HAL_ERROR;
	}
	*pTemperature 	= data.temperature;
	*pPressure		= data.pressure;

	return HAL_OK;

}
void BMP390_AssignI2C(I2C_HandleTypeDef *phi2c){
	bmp390_phi2c = phi2c;
}
