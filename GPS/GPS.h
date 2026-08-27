#ifndef __GPS_H
#define __GPS_H
#include "stabilizer_types.h"
#include "usart.h"
#include "cmsis_os.h"
#include "main.h"
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

extern    uint8_t GPSbuffer[2048];
extern   osMessageQueueId_t GPSDataHandle ;
extern    GPRMC_Data  GPS_data_Que;



float transs(uint8_t s[], int i);
int seek_i(int i, uint8_t s[]);
void state_read(uint8_t s[]);
void getGPSRawData(GPRMC_Data *gps);


#endif