/*
 * gps.c
 *
 *  Created on: Mar 27, 2024
 *      Author: SURFACE
 */

#include "module/gps/gps.h"

int GPS_parseGGA(uint8_t *GPS_NMEA_Message, GPS_DataTypeDef* pGPS_Data){
	// the message starts with comma(','), which means the message cut off the "$GPGGA" header
	uint8_t head = 0;
	uint8_t field = 0;
	uint8_t word[20] = {0,};
	uint8_t word_n = 0;
	uint8_t byte;

	int time_now;
	int hour, minute, second;
	int latitude, latitude_deg, latitude_point;
	int longitude, longitude_deg, longitude_point;
	int altitude;
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
			time_now = atoi(word);
			second = time_now % 100;
			time_now /= 100;
			minute = time_now % 100;
			time_now /= 100;
			hour = time_now % 100;
			// make BCD time
			hour 	= 	(int) ( hour / 10 << 4 ) | ( hour % 10);
			minute	= 	(int) ( minute / 10 << 4 ) | ( minute% 10);
			second	= 	(int) ( second / 10 << 4 ) | ( second % 10);
			break;
		case 2:
			// latitude
			// 0.0001 accuracy
			latitude = atof(word) * 10000;
			latitude_deg = latitude / 1000000;
			latitude_deg = latitude_deg * 10000;
			latitude_point = ( latitude - latitude_deg * 100 ) / 60;

			// based on significant figures
//			latitude = atof(word) * 100000;
//			latitude_deg = latitude / 10000000;
//			latitude_deg = latitude_deg * 10000000;
//			latitude_point = ( latitude - latitude_deg) * 100 / 60;

			latitude = latitude_deg + latitude_point;
			break;
		case 4:
			// longitude
			// 0.0001 accuracy
			longitude = atof(word) * 10000;
			longitude_deg = longitude / 1000000;
			longitude_deg = longitude_deg * 10000;
			longitude_point = ( longitude - longitude_deg * 100 ) / 60;

			// based on significant figures
//			longitude = atof(word) * 100000;
//			longitude_deg = longitude / 10000000;
//			longitude_deg = longitude_deg * 10000000;
//			longitude_point = ( longitude - longitude_deg) * 100 / 60;

			longitude = longitude_deg + longitude_point;
			break;
		case 7:
			// number of satellites
			satnum = atoi(word);
			break;
		case 9:
			// MSL altitude
			altitude = atof(word)*10;
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
	pGPS_Data->BCDhour = hour;
	pGPS_Data->BCDminute = minute;
	pGPS_Data->BCDsecond = second;
	pGPS_Data->altitude = altitude;
	pGPS_Data->latitude = latitude;
	pGPS_Data->longitude = longitude;
	pGPS_Data->satellites = satnum;

	return 0;
}
