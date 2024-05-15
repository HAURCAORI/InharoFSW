/*
 * gps.c
 *
 *  Created on: Mar 27, 2024
 *      Author: SURFACE
 */

#include "module/gps/gps.h"
#include "converter.h"
#include <stdlib.h>

int GPS_NMEA_parseGGA(uint8_t *GPS_NMEA_Message, GPS_DataTypeDef* pGPS_Data){
	// the message starts with comma(','), which means the message cut off the "$GPGGA" header
	uint8_t head = 0;
	uint8_t field = 0;
	uint8_t word[20] = {0,};
	uint8_t word_n = 0;
	uint8_t byte;

	int time_now;
	int hours, minutes, seconds;
	int latitude, latitude_deg, latitude_point;
	int longitude, longitude_deg, longitude_point;
	int altitude;
	char* end;
	int satnum;

	while(1){
		// for a word
		word_n = 0;
		while(1){
			// for a byte
			byte = GPS_NMEA_Message[head+word_n];
			if (byte == ','){
				// a word is done
				word[word_n] = '\0';
				head += word_n + 1;
				word_n = 0;
				break;
			}
			word[word_n] = byte;
			word_n++;
		}
		// check field number and do some processing
		switch(field){
		case 1:
			// time
			time_now = atoi((char*) word);
			seconds = time_now % 100;
			time_now /= 100;
			minutes = time_now % 100;
			time_now /= 100;
			hours = time_now % 100;
			// make BCD time
			//hours 	= 	(int) ( hours / 10 << 4 ) | ( hours % 10);
			//minutes	= 	(int) ( minutes / 10 << 4 ) | ( minutes % 10);
			//seconds	= 	(int) ( seconds / 10 << 4 ) | ( seconds % 10);
			break;
		case 2:
			// latitude
			// 0.0001 accuracy
			latitude = strtod((char*) word,&end)*10000;
			latitude_deg = latitude / 1000000;
			latitude_deg = latitude_deg * 10000;
			latitude_point = ( latitude - latitude_deg * 100 ) / 60;

			// based on significant figures
			//latitude = atof(word) * 100000;
			//latitude_deg = latitude / 10000000;
			//latitude_deg = latitude_deg * 10000000;
			//latitude_point = ( latitude - latitude_deg) * 100 / 60;

			latitude = latitude_deg + latitude_point;
			break;
		case 4:
			// longitude
			// 0.0001 accuracy
			longitude = strtod((char*) word,&end) * 10000;
			longitude_deg = longitude / 1000000;
			longitude_deg = longitude_deg * 10000;
			longitude_point = ( longitude - longitude_deg * 100 ) / 60;

			// based on significant figures
			//longitude = atof(word) * 100000;
			//longitude_deg = longitude / 10000000;
			//longitude_deg = longitude_deg * 10000000;
			//longitude_point = ( longitude - longitude_deg) * 100 / 60;

			longitude = longitude_deg + longitude_point;
			break;
		case 7:
			// number of satellites
			satnum = atoi((char*) word);
			break;
		case 9:
			// MSL altitude
			altitude = strtod((char*) word,&end)*10;
			break;
		default:
			break;
		}
		field++;	// next field
		if (field >= 10){
			// now all wanted fields are done
			break;
		}
	}
	// completed parsing
	pGPS_Data->hours = hours;
	pGPS_Data->minutes = minutes;
	pGPS_Data->seconds = seconds;
	pGPS_Data->altitude = altitude / 10.0f;
	pGPS_Data->latitude = latitude / 10000.0f;
	pGPS_Data->longitude = longitude / 10000.0f;
	pGPS_Data->satellites = satnum;

	return 0;
}
