#ifndef __STABILIIZER_H
#define __STABILIIZER_H
#include "stabilizer_types.h"
#include "usart.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "moter.h"
#include "DVL.h"
#include "imu.h"
#include "math.h"


#define PIDSTOP 0
#define PIDSTART 1
void  getconRawData(control_t *pp);


#endif