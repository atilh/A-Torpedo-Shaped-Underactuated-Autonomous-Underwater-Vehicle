#include "moter.h"


uint8_t pwmbuffer[50]={0};

int map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}






//PB0 PB1 ���ӷ����
void Heading_control(control_t *output){

__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, map(output->headingOut, 0,255, PWM_Heading_Min, PWM_Heading_Max));

__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 3000-map(output->headingOut,  0,255, PWM_Heading_Min, PWM_Heading_Max));

}

//PD12 ����T200
void thrust_control(control_t *output){
	
__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, map(output->thrust,  -255,255, PWM_Thrust_Min,PWM_Thrust_Max));

}

//PA6 PA7 ����������
void Depth_control(control_t *output){

__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,3000-map(output->depthOut, 0,255, PWM_Depth_Min, PWM_Depth_Max));

__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4,map(output->depthOut, 0,255, PWM_Depth_Min, PWM_Depth_Max));

}






void rudder_control(control_t *output){
   int pwm1 ,pwm2 ;
   int pwm3 ,pwm4 ;
   pwm3 = map(output->depthOut, 0,255, PWM_Depth_Min, PWM_Depth_Max);
   pwm4 = 3000-map(output->depthOut, 0,255, PWM_Depth_Min, PWM_Depth_Max);
   pwm1 = map(output->headingOut, 0,255, PWM_Depth_Min, PWM_Depth_Max);
   pwm2 = 3000-map(output->headingOut, 0,255, PWM_Depth_Min, PWM_Depth_Max);
   vTaskSuspendAll();
   sprintf((char*)pwmbuffer,"PWM1:%d,PWM2:%d,PWM3:%d,PWM4:%d\r\n", pwm1, pwm2,pwm3, pwm4);
   xTaskResumeAll();
   HAL_UART_Transmit_DMA(&huart3,(uint8_t *)pwmbuffer,sizeof(pwmbuffer));
   }
   






void Control_PWM_Set(control_t *output)
{
//   Heading_control(output);
   thrust_control(output);//推力输出-255 255;
//   Depth_control(output);
  rudder_control(output);
}


