/*
 * mpxv7002dp.c
 *
 *  Created on: May 13, 2024
 *      Author: SURFACE
 */

#include "module/mpxv7002dp/mpxv7002dp.h"

static double DP_tare_voltage = 2.5;

uint16_t DP_calculateAirSpeedComp(uint16_t adc_value, double static_pressure, double static_temperature){
	// get ADC voltage
	double adc_voltage;
	double differential_pressure;
	double stagnaion_pressure;
	double pressure_ratio;
	double velocity;
#ifdef DP_ADC_SCALE
//	adc_voltage = adc_value / 4015 * 3.3 * DP_ADC_SCALE;
	adc_voltage = adc_value * DP_ADC_PRODUCT_COEFFICIENT;
#else
//	adc_voltage = adc_value / 4015 * 3.3;
	adc_voltage = adc_value * 8.219178082191780e-4;
#endif
	// calculate pressure
	differential_pressure = (adc_voltage - DP_tare_voltage > 0) ?\
			(adc_voltage - DP_tare_voltage) * 1000 : 0;
	stagnaion_pressure = static_pressure + differential_pressure;
	pressure_ratio = stagnaion_pressure / static_pressure;

	// calculate velocity
	velocity = sqrt( ( pow(pressure_ratio, DP_POWER_COEFFICIENT) - 1 ) * DP_PRODUCT_COEFFICIENT / ( DP_HEAT_CAPACITY_RATIO * DP_GAS_COEFFICIENT_AIR * static_temperature ) );

	return velocity * 100;
}
uint16_t DP_calculateAirSpeedIncomp(uint16_t adc_value, double static_pressure, double static_temperature){
	double adc_voltage;
	double differential_pressure;
	double velocity;
#ifdef DP_ADC_SCALE
//	adc_voltage = adc_value / 4015 * 3.3 * DP_ADC_SCALE;
	adc_voltage = adc_value * DP_ADC_PRODUCT_COEFFICIENT;
#else
//	adc_voltage = adc_value / 4015 * 3.3;
	adc_voltage = adc_value * 8.219178082191780e-4;
#endif
	// calculate pressure
	differential_pressure = (adc_voltage - DP_tare_voltage > 0) ?\
			(adc_voltage - DP_tare_voltage) * 1000 : 0;
	velocity = sqrt(DP_INCOMP_PRODUCT_COEFFICIENT * differential_pressure);
	return velocity*100;
}

void DP_calcCalibrationFromADC(uint16_t adc_value){
	double adc_voltage;
	// set input adc_value for zero differential pressure voltage
	#ifdef DP_ADC_SCALE
	//	adc_voltage = adc_value / 4015 * 3.3 * DP_ADC_SCALE;
	adc_voltage = adc_value * DP_ADC_PRODUCT_COEFFICIENT;
	#else
	//	adc_voltage = adc_value / 4015 * 3.3;
	adc_voltage = adc_value * 8.219178082191780e-4;
	#endif
	DP_tare_voltage = adc_voltage;
}
void DP_setCalibrationFromDouble(double calibration_voltage){
	DP_tare_voltage = calibration_voltage;
}
double DP_getCalibration(void){
	// return calibration data for backup
	return DP_tare_voltage;
}
