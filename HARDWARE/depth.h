#ifndef __DEPTH_H
#define __DEPTH_H
#include "stabilizer_types.h"
#include "usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdio.h"
#include "string.h"


extern osMessageQueueId_t depthHandle;
extern uint8_t depthbuffer[128];



void getDepthData(float* f);








#endif