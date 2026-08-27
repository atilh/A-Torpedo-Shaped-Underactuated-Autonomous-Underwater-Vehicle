#ifndef __Uart2link_H
#define __Uart2link_H
#include "stabilizer_types.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "main.h"
#include "stdio.h"
#include <stdbool.h>
#include "string.h"
#include "usbd_cdc_if.h"
#include "atpk.h"


extern uint8_t* Uart2linkrx ;
extern uint32_t* Uart2rslen;
extern osMessageQueueId_t Uart2RxQueHandle;

//extern osSemaphoreId_t usblinkRxcpltHandle;

bool Uart2linkSendPacket(const atkp_t *p);





#endif