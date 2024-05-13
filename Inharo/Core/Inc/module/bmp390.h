/*
 * bmp390.h
 *
 *  Created on: Feb 23, 2024
 *      Author: MyeongSik Jeong
 */

#ifndef BMP390_H_
#define BMP390_H_

// include
# include "main.h"

// type
typedef struct _BMP390_CalibrationTypeDef{
	float par_t1;
	float par_t2;
	float par_t3;

	float par_p1;
	float par_p2;
	float par_p3;
	float par_p4;
	float par_p5;
	float par_p6;
	float par_p7;
	float par_p8;
	float par_p9;
	float par_p10;
	float par_p11;

	float t_lin;
}BMP390_CalibrationTypeDef;

// function prototypes
void BMP390_AssignI2C(I2C_HandleTypeDef *phi2c);
HAL_StatusTypeDef BMP390_ReadSensorData();
float BMP390_CompensatePressure( uint32_t RawTemp );
float BMP390_CompensateTemperature( uint32_t RawPres );
HAL_StatusTypeDef BMP390_ReadCalibration(void);
HAL_StatusTypeDef BMP390_ReadRawPressure( uint32_t *Buf, uint32_t Timeout);
HAL_StatusTypeDef BMP390_ReadRawTemperature( uint32_t *Buf, uint32_t Timeout);
HAL_StatusTypeDef BMP390_ReadSensorTime( uint32_t *Buf, uint32_t Timeout);
HAL_StatusTypeDef BMP390_SoftReset( uint32_t Timeout );
HAL_StatusTypeDef BMP390_Write(uint8_t MemAddress, uint8_t *Buf, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef BMP390_Read(uint8_t MemAddress, uint8_t *Buf, uint16_t Size, uint32_t Timeout);

// define
# define BMP390_ADDRESS	( 0x77 << 1 )

# define BMP390_PWR_PRS_NORMAL	0B00110010
# define BMP390_PWR_TMP_NORMAL	0B00110001
# define BMP390_PWR_ALL_NORMAL	0B00110011

# define BMP390_PWR_PRS_FORCED	0B00100001
# define BMP390_PWR_TMP_FORCED	0B00100010
# define BMP390_PWR_ALL_FORCED	0B00100011

# define BMP390_PWR_DISABLED	0B00000000

# define BMP390_OSR_PX01	0B00000000
# define BMP390_OSR_PX02	0B00001000
# define BMP390_OSR_PX04	0B00010000
# define BMP390_OSR_PX08	0B00011000
# define BMP390_OSR_PX16	0B00100000
# define BMP390_OSR_PX32	0B00101000

# define BMP390_OSR_PX01TX2	0B00000001
# define BMP390_OSR_PX02TX2	0B00001001
# define BMP390_OSR_PX04TX2	0B00010001
# define BMP390_OSR_PX08TX2	0B00011001
# define BMP390_OSR_PX16TX2	0B00100001
# define BMP390_OSR_PX32TX2	0B00101001

# define BMP390_ODR_200		0X00
# define BMP390_ODR_100		0X01
# define BMP390_ODR_50		0X02
# define BMP390_ODR_25		0X03
# define BMP390_ODR_12P5	0X04
# define BMP390_ODR_6P25	0X05
# define BMP390_ODR_3P1		0X06
# define BMP390_ODR_1P5		0X07
# define BMP390_ODR_0P78	0X08
# define BMP390_ODR_0P39	0X09
# define BMP390_ODR_0P2		0X0A
# define BMP390_ODR_0P1		0X0B
# define BMP390_ODR_0P05	0X0C
# define BMP390_ODR_0P02	0X0D
# define BMP390_ODR_0P01	0X0E
# define BMP390_ODR_0P006	0X0F
# define BMP390_ODR_0P003	0X10
# define BMP390_ODR_0P0015	0X11

# define BMP390_CFG_IIR0	0B00000000
# define BMP390_CFG_IIR1	0B00000010
# define BMP390_CFG_IIR3	0B00000100
# define BMP390_CFG_IIR7	0B00000110
# define BMP390_CFG_IIR15	0B00001000
# define BMP390_CFG_IIR31	0B00001010
# define BMP390_CFG_IIR63	0B00001100
# define BMP390_CFG_IIR127	0B00001110

# define BMP390_CMD_NOP		0X00
# define BMP390_CMD_FLUSH	0XB0
# define BMP390_CMD_SFTRST	0XB6

# define BMP390_REG_DATA	0x04
# define BMP390_REG_PRES	0x04
# define BMP390_REG_TEMP	0x07
# define BMP390_REG_TIME	0x0C
# define BMP390_REG_PWR		0X1B
# define BMP390_REG_OSR		0X1C
# define BMP390_REG_ODR		0X1D
# define BMP390_REG_CFG		0X1F
# define BMP390_REG_NVMPAR 	0x31
# define BMP390_REG_CMD		0x7E

#endif /* INC_BMP390_H_ */
