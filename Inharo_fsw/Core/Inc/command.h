#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include "main.h"

extern uint8_t isCommunication;
extern uint8_t isTimeSetGPS;
extern uint8_t isSimulationMode;
extern uint8_t isSimulationEnable;
extern enum CommandEcho cmd_echo;

extern uint32_t packetCount;
extern enum VehicleStateTypeDef vehicle_state;
extern uint8_t isHSDeployed;
extern uint8_t isPCDeployed;

typedef void *osEventFlagsId_t;
extern osEventFlagsId_t CommandEventHandle;

void CMD_excuteCX_ON(void);
void CMD_excuteCX_OFF(void);
void CMD_excuteST_TIME(const uint8_t *argument);
void CMD_excuteST_GPS(void);
void CMD_excuteSIM_ENABLE(void);
void CMD_excuteSIM_ACTIVATE(void);
void CMD_excuteSIM_DISABLE(void);
void CMD_excuteCAL(void);
void CMD_excuteBCN_ON(void);
void CMD_excuteBCN_OFF(void);
void CMD_excuteDEP_PC(void);
void CMD_excuteREL_HS(void);
void CMD_excuteINIT(void);
void CMD_excuteRESET(void);

#endif
