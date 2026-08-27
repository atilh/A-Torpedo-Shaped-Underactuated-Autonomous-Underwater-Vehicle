#ifndef __USB_H
#define __USB_H
#include "stabilizer_types.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "main.h"
#include "stdio.h"
#include <stdbool.h>
#include "string.h"
#include "usbd_cdc_if.h"
#include "atpk.h"


extern uint8_t* usblinkrx ;
extern uint32_t* usbrslen;
extern osMessageQueueId_t usbRxQueHandle;

extern osSemaphoreId_t usblinkRxcpltHandle;

bool usblinkSendPacket(const atkp_t *p);





#endif