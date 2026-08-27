#ifndef __moter_H
#define __moter_H
#include "tim.h"
#include "stabilizer.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#define PWM_Heading_Min 1000
#define PWM_Heading_Max 2000

#define PID_Output_Min -1
#define PID_Output_Max 1

#define PWM_Thrust_Min 1100
#define PWM_Thrust_Max 1900


#define PWM_Depth_Min 1200
#define PWM_Depth_Max 1800

extern uint8_t pwmbuffer[50];

int map(float x, float in_min, float in_max, float out_min, float out_max);

//PB0 PB1 连接方向舵
void Heading_control(control_t *output);

//PD12 连接T200
void thrust_control(control_t *output);

//PA6 PA7 连接升降舵
void Depth_control(control_t *output);

void Control_PWM_Set(control_t *output);

void JoystickTask(void *pvParameters);
void rudder_control(control_t *output);



#endif /* __ATTITUDE_PID_H */