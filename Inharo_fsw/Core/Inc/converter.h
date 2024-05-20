#ifndef INC_CONVERTER_H_
#define INC_CONVERTER_H_

#include <stdlib.h>

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

static uint8_t WeakCharCompare(const uint8_t *p1, const char *p2)
{
  const unsigned char *s1 = (const unsigned char *) p1;

  const unsigned char *s2 = (const unsigned char *) p2;
  unsigned char c1, c2;

 do {
      c1 = (unsigned char) *s1++;
      c2 = (unsigned char) *s2++;
      if (c1 == '\0' || c2 == '\0')
        return TRUE;
 } while (c1 == c2);

 return FALSE;
}


#endif /* INC_CONVERTER_H_ */
