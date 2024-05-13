/*
 * bmp390.c
 *
 *  Created on: Feb 23, 2024
 *      Author: SURFACE
 */

// include
#include <module/bmp390.h>

// variables
I2C_HandleTypeDef *bmp390_phi2c;
static BMP390_CalibrationTypeDef Bmp390calib;

// functions implement
void BMP390_AssignI2C(I2C_HandleTypeDef *phi2c){
	bmp390_phi2c = phi2c;
}
HAL_StatusTypeDef BMP390_ReadSensorData(float *PressurePtr, float *TemperaturePtr){
	HAL_StatusTypeDef result;
	uint32_t buf;

	result = BMP390_ReadRawTemperature(&buf, 10);
	if ( result != HAL_OK ) return result;
	*TemperaturePtr = BMP390_CompensateTemperature(buf);

	result = BMP390_ReadRawTemperature(&buf, 10);
	if ( result != HAL_OK ) return result;
	*PressurePtr = BMP390_CompensatePressure(buf);

	return HAL_OK;
}
float BMP390_CompensateTemperature( uint32_t RawTemp ){
	float pd1, pd2;	// partial data
	pd1 = ( float ) ( RawTemp - Bmp390calib.par_t1 );
	pd2 = ( float ) ( pd1 * Bmp390calib.par_t2);
	Bmp390calib.t_lin = pd2 + ( pd1 * pd1 ) * Bmp390calib.par_t3;
	return Bmp390calib.t_lin;
}
float BMP390_CompensatePressure( uint32_t RawPres ){
	float pd1, pd2, pd3;
	float po1, po2, po3;	// partial out

	// independent from pressure
	pd1 = Bmp390calib.par_p6 *  Bmp390calib.t_lin;
	pd2 = Bmp390calib.par_p7 * (Bmp390calib.t_lin * Bmp390calib.t_lin);
	pd3 = Bmp390calib.par_p8 * (Bmp390calib.t_lin * Bmp390calib.t_lin * Bmp390calib.t_lin);
	po1 = Bmp390calib.par_p5 +  pd1 + pd2 + pd3;

	// linear to pressure
	pd1 = Bmp390calib.par_p2 *  Bmp390calib.t_lin;
	pd2 = Bmp390calib.par_p3 * (Bmp390calib.t_lin * Bmp390calib.t_lin);
	pd3 = Bmp390calib.par_p4 * (Bmp390calib.t_lin * Bmp390calib.t_lin * Bmp390calib.t_lin);
	po2 = ( float ) RawPres * (Bmp390calib.par_p1 + pd1 + pd2 + pd3);

	// power 2
	pd1 = (float) RawPres     * (float) RawPres;
	pd2 = Bmp390calib.par_p9 +  Bmp390calib.par_p10 * Bmp390calib.t_lin;
	po3 = pd1 * pd2;

	// power 3
	pd3 = ((float) RawPres * (float) RawPres * (float) RawPres) * Bmp390calib.par_p11;

	return po1 + po2 + po3 + pd3;
}
HAL_StatusTypeDef BMP390_ReadCalibration(void){
	// declare
	uint16_t	nvm_par_t1 = 0, nvm_par_t2 = 0;
	int8_t		nvm_par_t3 = 0;
	int16_t		nvm_par_p1 = 0, nvm_par_p2 = 0;
	int8_t		nvm_par_p3 = 0, nvm_par_p4 = 0;
	uint16_t	nvm_par_p5 = 0, nvm_par_p6 = 0;
	int8_t		nvm_par_p7 = 0, nvm_par_p8 = 0;
	int16_t		nvm_par_p9 = 0;
	int8_t		nvm_par_p10 = 0, nvm_par_p11 = 0;

	uint8_t buf[21];
	HAL_StatusTypeDef result;

	// read
	result = BMP390_Read(BMP390_REG_NVMPAR, buf, sizeof(buf), 10);
	if ( result != HAL_OK) return result;
	nvm_par_t1 	= ((uint16_t) buf[1] ) 	<< 8 | (uint16_t)buf[0];
	nvm_par_t2 	= ((uint16_t) buf[3]) 	<< 8 | (uint16_t)buf[2];
	nvm_par_t3 	=  (int8_t)   buf[4];
	nvm_par_p1 	= ((int16_t)  buf[6])	<< 8 |  (int16_t)buf[5];
	nvm_par_p2 	= ((int16_t)  buf[8])	<< 8 |  (int16_t)buf[7];
	nvm_par_p3 	=  (int8_t)   buf[9];
	nvm_par_p4 	=  (int8_t)   buf[10];
	nvm_par_p5 	= ((uint16_t) buf[12]) 	<< 8 | (uint16_t)buf[11];
	nvm_par_p6 	= ((uint16_t) buf[14]) 	<< 8 | (uint16_t)buf[13];
	nvm_par_p7 	=  (int8_t)   buf[15];
	nvm_par_p8 	=  (int8_t)	  buf[16];
	nvm_par_p9 	= ((int16_t)  buf[18]) 	<< 8 |  (int16_t)buf[17];
	nvm_par_p10 =  (int8_t)   buf[19];
	nvm_par_p11 =  (int8_t)   buf[20];

	// calculate actual value
	Bmp390calib.par_t1 	= ( float ) 	nvm_par_t1 				/ 0.00390625f;
	Bmp390calib.par_t2 	= ( float ) 	nvm_par_t2 				/ 1073741824.f;
	Bmp390calib.par_t3 	= ( float ) 	nvm_par_t3 				/ 281474976710656.f;
	Bmp390calib.par_p1 	= ( float ) (	nvm_par_p1 - 16384.f) 	/ 1048576.f;
	Bmp390calib.par_p2 	= ( float ) (	nvm_par_p2 - 16384.f) 	/ 536870912.f;
	Bmp390calib.par_p3 	= ( float ) 	nvm_par_p3				/ 4294967296.f;
	Bmp390calib.par_p4 	= ( float ) 	nvm_par_p4				/ 137438953472.f;
	Bmp390calib.par_p5 	= ( float ) 	nvm_par_p5				/ 0.125f;
	Bmp390calib.par_p6 	= ( float ) 	nvm_par_p6				/ 64.f;
	Bmp390calib.par_p7 	= ( float ) 	nvm_par_p7				/ 256.f;
	Bmp390calib.par_p8 	= ( float ) 	nvm_par_p8				/ 32768.f;
	Bmp390calib.par_p9		= ( float ) 	nvm_par_p9				/ 281474976710656.f;
	Bmp390calib.par_p10	= ( float ) 	nvm_par_p10				/ 281474976710656.f;
	Bmp390calib.par_p11	= ( float ) 	nvm_par_p11				/ 36893488147419103232.f;

	return HAL_OK;
}
HAL_StatusTypeDef BMP390_ReadRawPressure( uint32_t *Buf, uint32_t Timeout){
	uint8_t buffer[3];
	HAL_StatusTypeDef result = BMP390_Read(BMP390_REG_PRES, buffer, sizeof(buffer), Timeout);
	if ( result != HAL_OK ) return result;
	*Buf = ( buffer[2] << 16 ) | ( buffer[1] << 8 ) | buffer[0];

	return HAL_OK;
}
HAL_StatusTypeDef BMP390_ReadRawTemperature( uint32_t *Buf, uint32_t Timeout){
	uint8_t buffer[3];
	HAL_StatusTypeDef result = BMP390_Read(BMP390_REG_TEMP, buffer, sizeof(buffer), Timeout);
	if ( result != HAL_OK ) return result;
	*Buf = ( buffer[2] << 16 ) | ( buffer[1] << 8 ) | buffer[0];

	return HAL_OK;
}
HAL_StatusTypeDef BMP390_ReadSensorTime( uint32_t *Buf, uint32_t Timeout){
	uint8_t buffer[3];
	HAL_StatusTypeDef result = BMP390_Read(BMP390_REG_TIME, buffer, sizeof(buffer), Timeout);
	if ( result != HAL_OK ) return result;
	*Buf = ( buffer[2] << 16 ) | ( buffer[1] << 8 ) | buffer[0];

	return HAL_OK;
}
HAL_StatusTypeDef BMP390_SoftReset( uint32_t Timeout ){
	uint8_t Buf = BMP390_CMD_SFTRST;
	return BMP390_Write( BMP390_REG_CMD, &Buf, 1, Timeout );
}
HAL_StatusTypeDef BMP390_Write(uint8_t MemAddress, uint8_t *Buf, uint16_t Size, uint32_t Timeout){
	return HAL_I2C_Mem_Write(bmp390_phi2c, BMP390_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, Buf, Size, Timeout);
}
HAL_StatusTypeDef BMP390_Read(uint8_t MemAddress, uint8_t *Buf, uint16_t Size, uint32_t Timeout){
	return HAL_I2C_Mem_Read(bmp390_phi2c, BMP390_ADDRESS, MemAddress, I2C_MEMADD_SIZE_8BIT, Buf, Size, Timeout);
}
