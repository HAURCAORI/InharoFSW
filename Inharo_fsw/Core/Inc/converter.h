#ifndef INC_CONVERTER_H_
#define INC_CONVERTER_H_

static uint8_t HexCharToByte(uint8_t upper, uint8_t lower) {
	uint8_t value;
	if (lower >= '0' && lower <= '9')
		value = (lower - '0');
	else if (lower >= 'A' && lower <= 'F')
		value = (10 + (lower - 'A'));
	else if (lower >= 'a' && lower <= 'f')
		value = (10 + (lower - 'a'));
	else
		return 0;

	if (upper >= '0' && upper <= '9')
		value += (upper - '0') * 0x10;
	else if (upper >= 'A' && upper <= 'F')
		value += (10 + (upper - 'A')) * 0x10;
	else if (upper >= 'a' && upper <= 'f')
		value += (10 + (upper - 'a')) * 0x10;
	else
		return 0;

	return value;
}

#endif /* INC_CONVERTER_H_ */
