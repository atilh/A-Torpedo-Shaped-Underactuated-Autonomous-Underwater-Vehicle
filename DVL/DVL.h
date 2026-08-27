#ifndef __DVL_H
#define __DVL_H
#include "stabilizer_types.h"
#include "usart.h"
#include "cmsis_os.h"
#include "main.h"
#include <ctype.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

extern    uint8_t DVL[2048];
extern   osMessageQueueId_t DVL_wrp_posHandle ;
extern   osMessageQueueId_t DVL_wrz_velocityHandle ;
extern   osMessageQueueId_t DVL_eurlHandle;
extern   osMessageQueueId_t DVL_attitudeHandle;
extern osMessageQueueId_t DVL_Speed_QueHandle;
extern osMessageQueueId_t DVL_Position_QueHandle;
extern   osSemaphoreId_t        DVLRxcpltHandle;
extern osSemaphoreId_t DVL_Handle;

extern   DVL_wrp_data wrp;
extern   DVL_wrz_data wrz;
extern uint8_t posreset[] ;
extern uint8_t gyroreset[] ;

void process_dma_buffer(uint8_t* buf, size_t len);
void DvlReadTask(void *argument);
void getDVLvelocityData(DVLdata* get);
void getDVLattitudeData(float* get);
void getDVLposData(DVLdata* get);
void getDVLeurlData(Axis3f* get);
void Reset_Dr(void);

#endif