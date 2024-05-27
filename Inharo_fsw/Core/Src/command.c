/*
 * command.c
 *
 *  Created on: 2024. 5. 21.
 *      Author: sunny
 */

#include <string.h>
#include <stdlib.h>
#include "command.h"
#include "type.h"

int Calibrate(void);
void Buzzer_On();
void Buzzer_Off();

void CMD_excuteCX_ON(void) {
	cmd_echo = ECHO_CX_ON;
	isCommunication = IH_CX_ON;

	//Backup
	uint32_t bkpdata;
	bkpdata = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP0R);
	bkpdata |= (1U << 1);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP0R, bkpdata);
}

void CMD_excuteCX_OFF(void) {
	cmd_echo = ECHO_CX_OFF;
	isCommunication = IH_CX_OFF;

	//Backup
	uint32_t bkpdata;
	bkpdata = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP0R);
	bkpdata &= ~(1U << 1);
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP0R, bkpdata);
}

void CMD_excuteST_TIME(const uint8_t *argument) {
	cmd_echo = ECHO_ST_UTC;
	RTC_TimeTypeDef time = { 0 };
	uint8_t btemp[2];
	uint8_t vtemp;
	// hours
	memcpy(btemp, argument, 2);
	vtemp = atoi((char*) btemp);
	if (vtemp > 24) return;
	time.Hours = vtemp;

	// minutes
	memcpy(btemp, argument + 3, 2);
	vtemp = atoi((char*) btemp);
	if (vtemp > 60) return;
	time.Minutes = vtemp;

	// seconds
	memcpy(btemp, argument + 6, 2);
	vtemp = atoi((char*) btemp);
	if (vtemp > 60) return;
	time.Seconds = vtemp;

	HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
}

void CMD_excuteST_GPS(void) {
	cmd_echo = ECHO_ST_GPS;
	isTimeSetGPS = TRUE;
}


void CMD_excuteSIM_ENABLE(void) {
	cmd_echo = ECHO_SIM_ENABLE;
	isSimulationEnable = TRUE;
}

void CMD_excuteSIM_ACTIVATE(void) {
	if (isSimulationEnable == TRUE) {
		cmd_echo = ECHO_SIM_ACTIVATE;
		isSimulationMode = TRUE;
		vehicle_state = (vehicle_state & STATE_MASK ) | STATE_MASK_SIMULATION;
	}
}

void CMD_excuteSIM_DISABLE(void) {
	cmd_echo = ECHO_SIM_DISABLE;
	isSimulationEnable = FALSE;
	isSimulationMode = FALSE;
	vehicle_state = (vehicle_state & STATE_MASK );
	CMD_excuteINIT();
}


void CMD_excuteCAL(void) {
	cmd_echo = ECHO_CAL;
	Calibrate();
}

void CMD_excuteBCN_ON(void) {
	cmd_echo = ECHO_BCN_ON;
	Buzzer_On();
}

void CMD_excuteBCN_OFF(void) {
	cmd_echo = ECHO_BCN_OFF;
	Buzzer_Off();
}


void CMD_excuteDEP_PC(void) {
	cmd_echo = ECHO_DEP_PC;
	//ToDO: implementation

}

void CMD_excuteREL_HS(void) {
	cmd_echo = ECHO_REL_HS;
	//ToDO: implementation
}

void CMD_excuteINIT(void) {
	cmd_echo = ECHO_INIT;
	packetCount = 0;
	if(isSimulationMode == TRUE) {
		vehicle_state = S_LAUNCH_WAIT;
	} else {
		vehicle_state = F_LAUNCH_WAIT;
	}
	isHSDeployed = FALSE;
	isPCDeployed = FALSE;

	osEventFlagsSet(CommandEventHandle, ACT_CAMERA);
}

void CMD_excuteRESET(void) {
	cmd_echo = ECHO_RESET;
	//Backup();
	NVIC_SystemReset();
}
