#ifndef __IMU_H
#define __IMU_H
#include "stabilizer_types.h"
#include "usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdio.h"
#include "string.h"

extern osMessageQueueId_t myQueue01Handle;
extern osMessageQueueId_t IMU_accHandle;
extern osMessageQueueId_t IMU_freeaccHandle;
extern osMessageQueueId_t IMU_gyroHandle;
extern uint8_t IMU[256];
extern XsensData_t    mti_data;

void getEurlRawData(Axis3f* Eurl);
void getAccRawData(IMUdata *Acc);
void getFreeaccRawData(Axis3f *FreeAcc);
void getGyroRawData(IMUdata *Gyro);

void parse_mti630_data(uint8_t* imubuffer, uint16_t Size,XsensData_t *mti_data) ;


#endif