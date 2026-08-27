#include "atpk.h"
#include "cmsis_os.h"
#include "main.h"
#include "usart.h"
#include "debug.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "usblink.h"
#include "imu.h"
#include "depth.h"
#include "DVL.h"
#include "GPS.h"
#include "moter.h"
#include "stabilizer.h"
#include "Uart2link.h"
#include "EKF.h"
#define  BYTE0(dwTemp)       ( *( (uint8_t *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (uint8_t *)(&dwTemp) + 3) )


#define  PERIOD_STATUS		30
#define  PERIOD_SENSOR 		10
#define  PERIOD_RCDATA 		40
#define  PERIOD_POWER 		100
#define  PERIOD_MOTOR		  40
#define  PERIOD_SENSOR2 	40
#define  PERIOD_SPEED   	50
#define  PERIOD_USERDATA   	20

extern control_t 	control;
extern unsigned char Mode;
float jcz;
extern Axis3f Init_Elur;
//extern osMessageQId ATKP_RXHandle;

typedef struct  
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t thrust;
}joystickFlyui16_t;

static void atkpSendPacket(atkp_t *p)//？？？
{
   
	usblinkSendPacket(p);
	//Uart2linkSendPacket(p);
	    
}


static void sendBACK(atkp_t *p)
{
	atkpSendPacket(p);
}

void sendSenser(float a_x,float a_y,float a_z)
{
    atkp_t p1;
	uint8_t _cnt=0;
  uint8_t FU  =0;
	
	int16_t _temp;
	
	p1.msgID = UP_RCDATA;//欧拉角

	_temp = (int)a_y*100;
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);

	_temp = (int)a_x*100;
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	_temp = (int)100*a_z;	
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	
	p1.data[_cnt++]=4;
	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}


void sendacc_gyro(IMUdata acc,IMUdata gory)
{
    atkp_t p1;
	uint8_t _cnt=0;
  uint8_t FU  =0;
	
	int16_t _temp;
	unsigned int _temp2;
	p1.msgID =0x01;

	_temp = (int16_t)(acc.IMU.y*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);

	_temp = (int16_t)(acc.IMU.x*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	_temp = (int16_t)(acc.IMU.z*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	
	
	_temp = (int16_t)(gory.IMU.y*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);

	_temp = (int16_t)(gory.IMU.x*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	_temp = (int16_t)(gory.IMU.z*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	
	//若使用上位机需要注释的部分
	_temp2=acc.timestamp;
	p1.data[_cnt++]=BYTE0(_temp2);
	p1.data[_cnt++]=BYTE1(_temp2);
    p1.data[_cnt++]=BYTE2(_temp2);
	p1.data[_cnt++]=BYTE3(_temp2);
//	p1.data[_cnt++]=0;
	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}

















void sendDepth(float temp,float bar)
{
  atkp_t p1;
	uint8_t _cnt=0;
  uint8_t FU  =0;
	int32_t _temp_bar;
	int16_t _temp;
	
	p1.msgID = 0x05;//深度
  
  _temp_bar =(int)(bar*100);
  p1.data[_cnt++]=BYTE0(_temp_bar);
	p1.data[_cnt++]=BYTE1(_temp_bar);
  p1.data[_cnt++]=BYTE2(_temp_bar);
	p1.data[_cnt++]=BYTE3(_temp_bar);
  
  
  _temp = (int)0;
  p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
  p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	p1.data[_cnt++]=0;

	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}


//上位机专用
//void sendPOS(float x,float y,float z)
//{
//     atkp_t p1;
//	uint8_t _cnt=0;
//    uint8_t FU  =0;
//	
//	int32_t _temp;
//	unsigned int _temp2;
//	p1.msgID = UP_POS;//0x08

//	_temp = (int32_t)(x*100);
////	_temp =-46;
//	p1.data[_cnt++]=BYTE0(_temp);
//	p1.data[_cnt++]=BYTE1(_temp);
//    p1.data[_cnt++]=BYTE2(_temp);
//	p1.data[_cnt++]=BYTE3(_temp);
//	
//	
//	_temp = (int32_t)(y*100);
////	_temp =-46;
//	p1.data[_cnt++]=BYTE0(_temp);
//	p1.data[_cnt++]=BYTE1(_temp);
//    p1.data[_cnt++]=BYTE2(_temp);
//	p1.data[_cnt++]=BYTE3(_temp);
//	

//	
//	p1.dataLen = _cnt;
//	atkpSendPacket(&p1);
//}

void sendPOS(float x,float y,float z,unsigned int timestamp)
{
     atkp_t p1;
	uint8_t _cnt=0;
    uint8_t FU  =0;
	
	int32_t _temp;
	unsigned int _temp2;
	p1.msgID = UP_POS;//0x08

	_temp = (int32_t)(x*100);
//	_temp =-46;
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	
	_temp = (int32_t)(y*100);
//	_temp =-46;
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	//若使用上位机需要注释的部分
	_temp2=timestamp;
	p1.data[_cnt++]=BYTE0(_temp2);
	p1.data[_cnt++]=BYTE1(_temp2);
    p1.data[_cnt++]=BYTE2(_temp2);
	p1.data[_cnt++]=BYTE3(_temp2);
	
	_temp = (int32_t)(z*100);
//	_temp =-46;
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}




void senduserGPS(GPRMC_Data GPS_data)
{
    atkp_t p1;
	uint8_t _cnt=0;
    uint8_t FU  =0;
	
	int32_t _temp;
	int16_t _temp1;
	unsigned int _temp2;
	p1.msgID = 0xF1;

	
//	_temp = (int32_t)(1200345628);
//	p1.data[_cnt++]=BYTE0(_temp);
//	p1.data[_cnt++]=BYTE1(_temp);
//    p1.data[_cnt++]=BYTE2(_temp);
//	p1.data[_cnt++]=BYTE3(_temp);
//	_temp = (int32_t)(1200345628);
//	p1.data[_cnt++]=BYTE0(_temp);
//	p1.data[_cnt++]=BYTE1(_temp);
//    p1.data[_cnt++]=BYTE2(_temp);
//	p1.data[_cnt++]=BYTE3(_temp);
//	_temp1 = (int16_t)(213.25*100);	
//	p1.data[_cnt++]=BYTE0(_temp1);
//	p1.data[_cnt++]=BYTE1(_temp1);
//	
//	p1.data[_cnt++]=1;
//	p1.data[_cnt++]=1;
//	p1.data[_cnt++]=2;
//	p1.data[_cnt++]=3;
	_temp = (int32_t)((GPS_data.latitude_deg+ (((double)GPS_data.latitude_min)/60.0))*10000000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	_temp = (int32_t)((GPS_data.longitude_deg+ (((double)GPS_data.longitude_min)/60.0))*10000000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
//	_temp1 = (int16_t)(GPS_data.heading_angle*100);	
//	p1.data[_cnt++]=BYTE0(_temp1);
//	p1.data[_cnt++]=BYTE1(_temp1);
//	
//	p1.data[_cnt++]=GPS_data.GPS_bit.state_bit;
//	p1.data[_cnt++]=GPS_data.GPS_bit.latitude_bit;
//	p1.data[_cnt++]=GPS_data.GPS_bit.longitude_bit;
//	p1.data[_cnt++]=GPS_data.GPS_bit.RTK_bit;
	
	//若使用上位机需要注释的部分
//	_temp2=getRunTimeCounterValue();
//	p1.data[_cnt++]=BYTE0(_temp2);
//	p1.data[_cnt++]=BYTE1(_temp2);
//    p1.data[_cnt++]=BYTE2(_temp2);
//	p1.data[_cnt++]=BYTE3(_temp2);
	
	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}

void senduserDVLwrz(DVLdata wrz_v)
{
    atkp_t p1;
	uint8_t _cnt=0;
    uint8_t FU  =0;
	
	int32_t _temp;
	unsigned int _temp2;
	p1.msgID = 0xF2;

	_temp = (int32_t)(wrz_v.DVL.x*1000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	_temp = (int32_t)(wrz_v.DVL.y*1000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	_temp = (int32_t)(wrz_v.DVL.z*1000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);

	_temp2=wrz_v.timestamp;
	p1.data[_cnt++]=BYTE0(_temp2);
	p1.data[_cnt++]=BYTE1(_temp2);
    p1.data[_cnt++]=BYTE2(_temp2);
	p1.data[_cnt++]=BYTE3(_temp2);	
	
	_temp = (int32_t)(wrz_v.time_of_transmission);//大的
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	_temp = (int32_t)(wrz_v.time_of_validity);//小的
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);

	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}





void senduserPWM(control_t PWM)
{
    atkp_t p1;
	uint8_t _cnt=0;
    uint8_t FU  =0;
	

	uint16_t _temp;
	unsigned int _temp2;
	p1.msgID = 0x20;

	_temp = (uint16_t)map(PWM.headingOut, 0,255, PWM_Heading_Min, PWM_Heading_Max);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
   
	_temp = (uint16_t)map(PWM.depthOut, 0,255, PWM_Heading_Min, PWM_Heading_Max);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
   
	_temp = (uint16_t)map(PWM.thrust, -255,255, 1100, 1900);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	
	_temp2=getRunTimeCounterValue();
	p1.data[_cnt++]=BYTE0(_temp2);
	p1.data[_cnt++]=BYTE1(_temp2);
    p1.data[_cnt++]=BYTE2(_temp2);
	p1.data[_cnt++]=BYTE3(_temp2);
//    _temp = (uint16_t)map(127, 0,255, PWM_Heading_Min, PWM_Heading_Max);
//	p1.data[_cnt++]=BYTE0(_temp);
//	p1.data[_cnt++]=BYTE1(_temp);
	

	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}

void send_InitElur(Axis3f* Elur_Init)
{
	int16_t _temp;
	int32_t _temp1;
	atkp_t p1;
	p1.msgID =0x87;
	uint8_t _cnt=0;
	_temp = (int16_t)(Elur_Init->y*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);

	_temp = (int16_t)(Elur_Init->x*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
	_temp1 = (int32_t)(Elur_Init->z*100);
	p1.data[_cnt++]=BYTE0(_temp1);
	p1.data[_cnt++]=BYTE1(_temp1);
	p1.data[_cnt++]=BYTE2(_temp1);
	p1.data[_cnt++]=BYTE3(_temp1);
	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
	
}
void senduserDVLwrp(float x,float y,float z)
{
    atkp_t p1;
	uint8_t _cnt=0;

	
	int32_t _temp;

	
	p1.msgID = 0x32;

	_temp = (int32_t)(x*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	_temp = (int32_t)(y*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	_temp = (int32_t)(z*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);

	

	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
}
void sendFusion(IMUdata *PP,IMUdata *SS)
{
	 atkp_t p1;
	uint8_t _cnt=0;
	
	int32_t _temp;
	unsigned int _temp2;
	p1.msgID = 0x55;//0x08

	_temp = (int32_t)(PP->IMU.x*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	_temp = (int32_t)(PP->IMU.y*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	_temp = (int32_t)(PP->IMU.z*100);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	_temp = (int32_t)(SS->IMU.x*1000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	_temp = (int32_t)(SS->IMU.y*1000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
	
	_temp = (int32_t)(SS->IMU.z*1000);
	p1.data[_cnt++]=BYTE0(_temp);
	p1.data[_cnt++]=BYTE1(_temp);
    p1.data[_cnt++]=BYTE2(_temp);
	p1.data[_cnt++]=BYTE3(_temp);
    //若使用上位机需要注释的部分
	_temp2=PP->timestamp;
	p1.data[_cnt++]=BYTE0(_temp2);
	p1.data[_cnt++]=BYTE1(_temp2);
    p1.data[_cnt++]=BYTE2(_temp2);
	p1.data[_cnt++]=BYTE3(_temp2);
	
	p1.dataLen = _cnt;
	atkpSendPacket(&p1);
	
}

Axis3f wrz_v;
float    wrz_h;
GPRMC_Data GPS;
DVLdata pos;
DVLdata vloc;
float depth1;
control_t PWM_send;
Axis3f pos_all;
IMUdata acc__;
IMUdata gyro__;
IMUdata PP_;
IMUdata SS_;
//static void atkpSendPeriod(void)
//{
//	static uint16_t count_ms = 1;

//	if(!(count_ms % PERIOD_SENSOR))
//	{
//		Axis3f eurl;
//		getEurlRawData(&eurl);
//		sendSenser(eurl.x,eurl.y,eurl.z);
//	}
//  
//  	if(!(count_ms % 5))
//	{
//		 getDepthData(&depth1);
//         sendDepth(0,depth1);
//	}
//		
//	  	if(!(count_ms % 200))
//	{
//		
//        getDVLposData(&pos);
//	     sendPOS(pos.x,pos.y,pos.z);

//	}
//	
//	
//	   if(!(count_ms % 200))
//	{
//		
//         getDVLposData(&pos_all);
//	     senduserDVLwrp(pos_all.x,pos_all.y,pos_all.z);

//	}
//	
//	
//	
//	  if(!(count_ms % 10))
//	{		
//        getGPSRawData(&GPS);
//		senduserGPS(GPS);
//	}
//	
//	if(!(count_ms % 20))
//	{
//        getconRawData(&PWM_send);
//		senduserPWM(PWM_send);
//	}
////dvl閫熷害
//	if(!(count_ms % 5))
//	{
//	
//		
//		getDVLvelocityData(&wrz_v);
//        getDVLattitudeData(&wrz_h);
//        senduserDVLwrz(wrz_v,wrz_h);
//	}
//	
////鍔犻�熷害锛岃閫熷害
//		if(!(count_ms % PERIOD_SENSOR))
//	{
//		Axis3f acc;
//		Axis3f gyro;
//		
//        getFreeaccRawData(&acc);
//        getGyroRawData(&gyro);
//        sendacc_gyro(acc,gyro);
//		
//	}
//	
//	
//	

//	if(++count_ms>=65535) 
//		count_ms = 1;	
//}
//static void atkpSendPeriod(void)
//{
//	 static uint16_t count_ms_1 = 1;

//		   if(!(count_ms_1 % 200))//200
//	{
//		
//         getDVLposData(&pos_all);
//	     senduserDVLwrp(pos_all.x,pos_all.y,pos_all.z);//32

//	}
//		  	if(!(count_ms_1 % 200))//200
//	{
//		
//        getDVLposData(&pos);//08
//	     sendPOS(pos.x,pos.y,pos.z);

//	}
//		if(!(count_ms_1 % 20))
//	{
//        getconRawData(&PWM_send);
//		senduserPWM(PWM_send);//20
//	}
//		if(!(count_ms_1 % 10))
//	{
//		Axis3f eurl;
//		getEurlRawData(&eurl);//03
//		sendSenser(eurl.x,eurl.y,eurl.z);
//	}
//			if(!(count_ms_1 % PERIOD_SENSOR))//PERIOD_SENSOR 10
//	{
//		Axis3f acc;
//		Axis3f gyro;
//		
//        getFreeaccRawData(&acc);
//        getGyroRawData(&gyro);
//        sendacc_gyro(acc,gyro);//01
//	}
//		  if(!(count_ms_1 % 8))
//	{		
//        getGPSRawData(&GPS);//F1
//		senduserGPS(GPS);
//	}
//		if(!(count_ms_1 % 5))
//	{
//		getDVLvelocityData(&wrz_v);
//        getDVLattitudeData(&wrz_h);
//        senduserDVLwrz(wrz_v,wrz_h);//F2
//	}
//  	if(!(count_ms_1 % 5))
//	{
//		 getDepthData(&depth1);//05
//         sendDepth(0,depth1);
//		
//	}
//		

//	
//	

//	
//	
//	

//	

////dvl閫熷害

//	
////鍔犻�熷害锛岃閫熷害

////	
//	
//	

//	if(++count_ms_1>=65535) 
//	{
//		count_ms_1 = 1;
//	}
//			
//}

static void atkpSendPeriod(void)
{
	 static uint32_t count_ms_1 = 1;
//		if(count_ms_1==100000)//20s发送初始欧拉角
//	{
//		send_InitElur(&Init_Elur);//0x87
//	}
//	if(!(count_ms_1 % 1000))//1HZ
//	{
//		senduserPWM(control);
////		getGPSRawData(&GPS);//F1
////		senduserGPS(GPS);//可以只要有限的，不用时间戳
//	}
			if(!(count_ms_1 % 400))//2.5HZ
	{
			getconRawData(&PWM_send);
		senduserPWM(PWM_send);
	}
		if(!(count_ms_1 % 200))//5HZ
	{
		getDVLvelocityData(&vloc);
		senduserDVLwrz(vloc);//F2  DVL的原始速度数据 注意不能两个放在一起

	}
			if(!(count_ms_1 % 100))//10HZ
	{
		getDVLposData(&pos);
		sendPOS(pos.DVL.x,pos.DVL.y,pos.DVL.z,pos.timestamp);//08  DVL的原始位置数据
		
		
	}
//		   if(!(count_ms_1 % 20))//50hz
//	{
//		//需要发送融合后的位置信息速度信息
//		getFusion_PS(&PP_,&SS_);
//		sendFusion(&PP_,&SS_);//55
//	}
			   if(!(count_ms_1 % 10))//100hz
	{
		getAccRawData(&acc__);
        getGyroRawData(&gyro__);
		sendacc_gyro(acc__,gyro__);//01
	}
	if(++count_ms_1>=4294967295) 
	{
		count_ms_1 = 1;
	}
			
}
void atpkTxTask(void const * argument)//按照一定的频率往队列里发送数据包
{
  /* USER CODE BEGIN atkp_TxTask */
  /* Infinite loop */
  for(;;)
  {
    
    atkpSendPeriod();
	osDelay(1);
  }
  /* USER CODE END atkp_TxTask */
}


uint16_t button = 0;
uint8_t count;
float yaw;
float speed;
uint8_t MODE_ch = 0;

void atkpReceiveAnl(atkp_t *anlPacket)
{
	if(anlPacket->msgID	== 0x01)//PID参数设置
	{
		jcz=((float)(*(uint16_t*)(&(anlPacket->data[0]))))/100;
		//jcz=anlPacket->data[0];//发送的信息有0元素无法发送成功
	}
		if(anlPacket->msgID	== 0x02)//PID开启
	{
		uint8_t jcz1;
		jcz1=anlPacket->data[0];
		if(jcz1==0x26)//AUVSTOP命令
		{
			Mode=0;
			control.thrust=0;
			control.headingOut=127;//小于127左转向，大于127右转
			control.depthOut=127;//小于127下潜，大于127上升
			Control_PWM_Set(&control);
		}
		else if(jcz1==0x24)//开启PID
		{
			Mode=1;
		}
		else//关闭PID
		{
			Mode=0;
		}
			
	}
	if(anlPacket->msgID	== 0x51)
	{
		
		if(MODE_ch == 0){
       control.depthOut = anlPacket->data[3];
       control.headingOut = anlPacket->data[0];
		}
	
			
       control.thrust = anlPacket->data[4]  -  anlPacket->data[5];
				
  }
  
  
  	if(anlPacket->msgID	== 0x50)
	{
		
		sprintf((char*)posreset,"wcr*e2\r\n");
        HAL_UART_Transmit_IT(&huart1,(uint8_t *)posreset, 8); 
		
		HAL_UART_Transmit(&hlpuart1,(uint8_t *)posreset, 8,1000); 
		count++;
  }
  
  
  
    	if(anlPacket->msgID	== 0x41)
	{
		
		uint16_t value = (anlPacket->data[7] << 8) | anlPacket->data[6];
		yaw = value/100;
		uint16_t value2 = (anlPacket->data[9] << 8) | anlPacket->data[8];
		speed = value2/100;
		count++;
  }
  
  
      	if(anlPacket->msgID	== 0x48)
	{
		
		MODE_ch = 1;
		count++;
  }
        	if(anlPacket->msgID	== 0x46)
	{
		
		MODE_ch = 0;
		count++;
  }
  
  
  
} 



void atpkRxTask(void *argument)//接收来自电脑发送的数据包
{
  /* USER CODE BEGIN atpkRxTask */
  atkp_t p;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet( atpkrxHandle, /* 消息队列的句柄 */
                           &p,/* 需要接受的消息内容存放地址 */
                           0,                        /* 接收优先级*/
                           osWaitForever);  /*永远等待 */
       
    atkpReceiveAnl(&p);//解析并实现功能
//    atkpSendPacket(&p);
    //osDelay(1);
  }
  /* USER CODE END atpkRxTask */
}






bool atkpReceivePacketBlocking(atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return osMessageQueuePut( atpkrxHandle, /* 消息队列的句柄*/
                                 p,/* 发送的消息内容 */
                                         0,                   /* 发送优先级*/
                                        osWaitForever); 
  
}

