
#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "main.h"
#include <stdio.h>

int _write(int file, char *ptr, int len);

void logi(const char* message, ...);
void logd(const char* message, ...);
void loge(const char* message, ...);

#endif /* INC_LOGGER_H_ */
