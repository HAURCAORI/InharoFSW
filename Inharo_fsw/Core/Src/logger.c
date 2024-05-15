#include "logger.h"
#include "stdarg.h"

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

int _write(int file, char *ptr, int len)
{
	CDC_Transmit_FS((uint8_t*) ptr, len);
	for(int i = 0; i < len; i++)
	{
		ITM_SendChar(*ptr++);

	}
	return len;
}

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
uint32_t g_SubSeconds;

void UpdateTime() {
	g_SubSeconds = ((255-(uint32_t)(hrtc.Instance->SSR))*1000L)/(255+1); // ms
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

void log_format(const char* tag, const char* message, va_list args) {
	UpdateTime();

	printf("%02d:%02d:%02d,%03lu[%s] ", sTime.Hours, sTime.Minutes, sTime.Seconds, g_SubSeconds, tag);
	vprintf(message, args);
	printf("\r\n");
}

void logi(const char* message, ...) {
	va_list args;
	va_start(args, message);
	log_format("INFO", message, args);
	va_end(args);
}

void logd(const char* message, ...) {
	va_list args;
	va_start(args, message);
	log_format("DEBUG", message, args);
	va_end(args);
}

void loge(const char* message, ...) {
	va_list args;
	va_start(args, message);
	log_format("ERROR", message, args);
	va_end(args);
}
