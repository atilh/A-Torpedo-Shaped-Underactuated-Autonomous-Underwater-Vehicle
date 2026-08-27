#include "stabilizer.h"
#include "pid.h"
#include "atpk.h"
control_t 	control;
PidObject PID_Position;
PidObject PID_Heading;

pidInit_t   pidheading;

sensorData_t getsensorData;
setpoint_t        set;
unsigned char Mode=0;

float normalize_heading_error(float desired, float imu) {
    // è®¡ç®—åŽŸå§‹èˆªå‘å·®
    float delta = desired - imu;

    // å½’ä¸€åŒ–åˆ° [-180, 180)
    delta = fmodf(delta + 540.0f, 360.0f) - 180.0f;

    // å¤„ç†æµ®ç‚¹è¿ç®—çš„è¾¹ç•Œæƒ…å†µï¼ˆä¾‹å¦‚ 179.9999999 â†’ -180.0ï¼‰
    if (delta >= 180.0f) {
        delta -= 360.0f;
    } else if (delta < -180.0f) {
        delta += 360.0f;
    }

    return delta;
}



void state1Control(control_t *control, sensorData_t *sensors, setpoint_t *setpoint);


void stabilizerTask(void* param)
{
	uint32_t tick = 0;
	uint32_t lastWakeTime = xTaskGetTickCount();
	
	Axis3f  DVL_v;
    Axis3f  DVL_p;
    Axis3f  DVL_e;
    float     DVL_a;
    Axis3f   IMU_e;
    Axis3f   IMU_a;
    Axis3f   IMU_fa;
    Axis3f     IMU_g;
	
	GPRMC_Data GPS_Data;
	
	
	
	pidheading.kp = 5;
	pidheading.ki = 0;
	pidheading.kd = 0;
	
	
	
	pidInit(&PID_Heading, 0, pidheading, 0.02);
	
	set.attitude.yaw = 0;
	
	
    control.depthOut=255;
    control.headingOut=0;
  
  
    rudder_control(&control);
    osDelay(20);
    int i =0;
	for(i =0;i<50;i++){
	
	control.depthOut-=2.5;
	control.headingOut+=2.5;
	rudder_control(&control);	
	osDelay(20);
	
	}
		
    control.depthOut=127;
    control.headingOut=127;
	control.thrust=0;
	thrust_control(&control);
	osDelay(2000);
//	while(!sensorsAreCalibrated())
//	{
//		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
//	}
	
	while(1) 
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);		

//		//»ñÈ¡6ÖáºÍÆøÑ¹Êý¾Ý£¨500Hz£©
//		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
//		{
//			sensorsAcquire(&sensorData, tick);				/*»ñÈ¡6ÖáºÍÆøÑ¹Êý¾Ý*/
//		}

//		//ËÄÔªÊýºÍÅ·À­½Ç¼ÆËã£¨250Hz£©
//		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
//		{
//			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
//		}

//		//Î»ÖÃÔ¤¹À¼ÆËã£¨250Hz£©
//		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
//		{
//			positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
//		}
//			
//		//Ä¿±ê×ËÌ¬ºÍ·ÉÐÐÄ£Ê½Éè¶¨£¨100Hz£©	
//		if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated()==true)
//		{
//			commanderGetSetpoint(&setpoint, &state);	/*Ä¿±êÊý¾ÝºÍ·ÉÐÐÄ£Ê½Éè¶¨*/	
//		}
//		
//		if (RATE_DO_EXECUTE(RATE_250_HZ, tick))
//		{
//			fastAdjustPosZ();	/*¿ìËÙµ÷Õû¸ß¶È*/
//		}		
//		
//		/*¶ÁÈ¡¹âÁ÷Êý¾Ý(100Hz)*/
//		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
//		{
//			getOpFlowData(&state, 0.01f);	
//		}
//		

	
		
//		/*PID¿ØÖÆ*/	
//		
//		  stateControl(&control, &sensorData, &state, &setpoint, tick);
//				
//		
    
//	       getDVLvelocityData(&DVL_v);
//           getDVLattitudeData(&DVL_a);
//           getDVLposData(&DVL_p);
//           getDVLeurlData(&DVL_e);
//	       getEurlRawData(&IMU_e);
//           getAccRawData(&IMU_a);
//           getFreeaccRawData(&IMU_fa);
//           getGyroRawData(&IMU_g);
//		   getGyroRawData(&GPS_Data);
           
		   
//		   getsensorData.Hearding = IMU_e.z;
           
		   
//     if(MODE_ch == 1){

//        if (RATE_DO_EXECUTE(RATE_50_HZ, tick))
//		{
//           state1Control(&control, &getsensorData, &set);
//		}
//	}

		
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{

			//if(Mode==PIDSTART)
			//{
//				if(control.thrust>=100)
//				{
//					control.thrust=100;
//				}
				//control.headingOut=50;//Ð¡ÓÚ127×ó×ªÏò£¬´óÓÚ127ÓÒ×ª
				//control.depthOut=50;//Ð¡ÓÚ127ÏÂÇ±£¬´óÓÚ127ÉÏÉý
				Control_PWM_Set(&control);
//			   control.thrust+=1;
			//}
				//__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, map(200,  -255,255, PWM_Thrust_Min,PWM_Thrust_Max));
////				thrust_control(&control);
			
		}	
//        if(tick%2==0)
//		{
//			//control.thrust=200;
//			Control_PWM_Set(&control);
//		}			
//		
    
		tick++;
	}
}


float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float calculate_heading_error(double target, double actual) {
    float error = target - actual;
    
    // å°†è¯¯å·®é™åˆ¶åˆ° [-180, 180) èŒƒå›´å†…
    error = fmod(error, 360.0);       // å…ˆå–æ¨¡åˆ° (-360, 360)
    error = fmod(error + 360.0, 360.0); // ç¡®ä¿éžè´Ÿ [0, 360)
    
    // è½¬æ¢ä¸º [-180, 180)
    if (error > 180.0) {
        error -= 360.0;
    }
    
    // å¤„ç†180åº¦çš„è¾¹ç•Œæƒ…å†µ
    if (error == 180.0) {
        error = -180.0;
    }
    
    return error;
}



void state1Control(control_t *control, sensorData_t *sensors, setpoint_t *setpoint)
{

	   float erro=0.0;

	   erro =  normalize_heading_error(setpoint->attitude.yaw,sensors->Hearding);

	   control->headingOut= mapf(pidUpdate(&PID_Heading, erro),-500,500,0,255);
	
}

void  getconRawData(control_t *pp){

   *pp = control;

}

