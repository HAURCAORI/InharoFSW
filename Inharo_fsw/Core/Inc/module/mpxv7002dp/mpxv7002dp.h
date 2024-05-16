/*
 * mpxv7002dp.h
 *
 *  Created on: May 13, 2024
 *      Author: SURFACE
 */

#ifndef INC_MODULE_MPXV7002DP_MPXV7002DP_H_
#define INC_MODULE_MPXV7002DP_MPXV7002DP_H_

#include <math.h>
#include <stdint.h>

#define DP_HEAT_CAPACITY_RATIO ( 1.4 )
#define DP_GAS_COEFFICIENT_AIR ( 287.052874	)

//#define DP_POWER_COEFFICIENT (( DP_HEAT_CAPACITY_RATIO - 1 ) / ( DP_HEAT_CAPACITY_RATIO ))
#define DP_POWER_COEFFICIENT ( 0.285714285714286 )
//#define DP_PRODUCT_COEFFICIENT ( 2 / ( DP_GAS_COEFFICIENT_AIR - 1 ) )
#define DP_PRODUCT_COEFFICIENT ( 5.0 )
//#define DP_INCOMP_PRODUCT_COEFFICIENT ( 2 / 1.225 )
#define DP_INCOMP_PRODUCT_COEFFICIENT ( 1.632653061224490 )

#define DP_ADC_SCALE ( 1.5 )
#define DP_ADC_PRODUCT_COEFFICIENT ( 0.001232876712329 )

uint16_t DP_calculateAirSpeedComp(uint16_t adc_value, double static_pressure, double static_temperature);
uint16_t DP_calculateAirSpeedIncomp(uint16_t adc_value, double static_pressure, double static_temperature);
void DP_calcCalibrationFromADC(uint16_t adc_value);
void DP_setCalibrationFromDouble(double calibration_voltage);
double DP_getCalibration(void);

#endif /* INC_MODULE_MPXV7002DP_MPXV7002DP_H_ */
