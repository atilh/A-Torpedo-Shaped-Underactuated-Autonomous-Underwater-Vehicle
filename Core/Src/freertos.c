/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "imu.h"
#include "tim.h"
#include "stdio.h"
#include "debug.h"
#include "atpk.h"
#include "WS2812.h"
#include "depth.h"
#include "atpk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};

osMessageQueueId_t usbtxHandle;
const osMessageQueueAttr_t usbtx_attributes = {
  .name = "usbtx"
};

osMessageQueueId_t depthHandle;
const osMessageQueueAttr_t depth_attributes = {
  .name = "depth"
};

osMessageQueueId_t atpkrxHandle;
const osMessageQueueAttr_t atpkrx_attributes = {
  .name = "atpkrx"
};


osMessageQueueId_t  DVL_wrp_posHandle;
const osMessageQueueAttr_t DVL_wrp_pos_attributes = {
  .name = "DVL_wrp_pos"
};

osMessageQueueId_t  DVL_wrz_velocityHandle;
const osMessageQueueAttr_t DVL_wrz_velocity_attributes = {
  .name = "DVL_wrz_velocity"
};

osSemaphoreId_t  DVLRxcpltHandle;
const osSemaphoreAttr_t DVLRxcplt_attributes = {
  .name = "DvlRxcplt"
};



osMessageQueueId_t  IMU_accHandle;
const osMessageQueueAttr_t IMU_acc_attributes = {
  .name = "IMU_acc"
};
osMessageQueueId_t  IMU_freeaccHandle;
const osMessageQueueAttr_t IMU_freeacc_attributes = {
  .name = "IMU_freeacc"
};
osMessageQueueId_t  IMU_gyroHandle;
const osMessageQueueAttr_t IMU_gyro_attributes = {
  .name = "IMU_gyro"
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMURead */
osThreadId_t IMUReadHandle;
const osThreadAttr_t IMURead_attributes = {
  .name = "IMURead",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for atpk_tx */
osThreadId_t atpk_txHandle;
const osThreadAttr_t atpk_tx_attributes = {
  .name = "atpk_tx",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for usblinktx */
osThreadId_t usblinktxHandle;
const osThreadAttr_t usblinktx_attributes = {
  .name = "usblinktx",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DepthRead */
osThreadId_t DepthReadHandle;
const osThreadAttr_t DepthRead_attributes = {
  .name = "DepthRead",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for usblink_Rx */
osThreadId_t usblink_RxHandle;
const osThreadAttr_t usblink_Rx_attributes = {
  .name = "usblink_Rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for atpk_rx */
osThreadId_t atpk_rxHandle;
const osThreadAttr_t atpk_rx_attributes = {
  .name = "atpk_rx",
  .stack_size = 300 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for stabilizer */
osThreadId_t stabilizerHandle;
const osThreadAttr_t stabilizer_attributes = {
  .name = "stabilizer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for DVLReadTask */
osThreadId_t DVLReadTaskHandle;
const osThreadAttr_t DVLReadTask_attributes = {
  .name = "DVLReadTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for GPSReadTask */
osThreadId_t GPSReadTaskHandle;
const osThreadAttr_t GPSReadTask_attributes = {
  .name = "GPSReadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LED */
osThreadId_t LEDHandle;
const osThreadAttr_t LED_attributes = {
  .name = "LED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usbRxQue */
osMessageQueueId_t usbRxQueHandle;
const osMessageQueueAttr_t usbRxQue_attributes = {
  .name = "usbRxQue"
};
/* Definitions for DVL_eurl */
osMessageQueueId_t DVL_eurlHandle;
const osMessageQueueAttr_t DVL_eurl_attributes = {
  .name = "DVL_eurl"
};
/* Definitions for DVL_attitude */
osMessageQueueId_t DVL_attitudeHandle;
const osMessageQueueAttr_t DVL_attitude_attributes = {
  .name = "DVL_attitude"
};
/* Definitions for GPSData */
osMessageQueueId_t GPSDataHandle;
const osMessageQueueAttr_t GPSData_attributes = {
  .name = "GPSData"
};
/* Definitions for usblinkRxcplt */
osSemaphoreId_t usblinkRxcpltHandle;
const osSemaphoreAttr_t usblinkRxcplt_attributes = {
  .name = "usblinkRxcplt"
};
/* Definitions for Uart2Que */
osMessageQueueId_t Uart2RxQueHandle;
const osMessageQueueAttr_t Uart2RxQue_attributes = {
  .name = "Uart2RxQue"
};
osMessageQueueId_t Uart2TxQueHandle;
const osMessageQueueAttr_t Uart2tx_attributes = {
  .name = "Uart2tx"
};
/* Definitions for Uart2link */
osThreadId_t Uart2link_RxHandle;
const osThreadAttr_t Uart2link_Rx_attributes = {
  .name = "Uart2link_Rx",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
osThreadId_t Uart2linktxHandle;
const osThreadAttr_t Uart2linktx_attributes = {
  .name = "Uart2linktx",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t EKF_UpdataHandle;
const osThreadAttr_t EKF_attributes = {
  .name = "EKF_Updata",
  .stack_size = 256 * 16,//原来是4
  .priority = (osPriority_t) osPriorityHigh,
};
osSemaphoreId_t EKF_IMUHandle;
const osSemaphoreAttr_t EKF_IMU_attributes = {
  .name = "EKF_IMU"
};
osSemaphoreId_t DVL_Handle;
const osSemaphoreAttr_t DVL_attributes = {
  .name = "DVL_"
};
osMessageQueueId_t DVL_Position_QueHandle;
const osMessageQueueAttr_t DVL_Position_Que_attributes = {
  .name = "Uart2RxQue"
};
osMessageQueueId_t DVL_Speed_QueHandle;
const osMessageQueueAttr_t DVL_Speed_Que_attributes = {
  .name = "Uart2RxQue"
};
osMessageQueueId_t EKF_IMU_QueHandle;
const osMessageQueueAttr_t EKF_IMU_Que_attributes = {
  .name = "EKF_IMU_Que"
};
osMessageQueueId_t EKF_Position_QueHandle;
const osMessageQueueAttr_t EKF_Position_Que_attributes = {
  .name = "EKF_Position_Que"
};
osSemaphoreId_t DataSentHandle;
const osSemaphoreAttr_t DataSent_attributes = {
  .name = "EKF_IMU"
};
osThreadId_t EKF_FusionHandle;
const osThreadAttr_t EKF_Fusionattributes = {
  .name = "EKF_Fusion",
  .stack_size = 128 * 4,//原来是4
  .priority = (osPriority_t) osPriorityHigh,//注意优先级
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void IMUReadTask(void *argument);
void atpkTxTask(void *argument);
void usblinktxTask(void *argument);
void DepthReadTask(void *argument);
void usblink_RxTask(void *argument);
void atpkRxTask(void *argument);
void stabilizerTask(void *argument);
void DvlReadTask(void *argument);
void GpsReadTask(void *argument);
void ledTask11(void *argument);
void Uart2linktxTask(void *argument);
void Uart2link_RxTask(void *argument);
void EKF_UpdataTask(void *argument);
void EKF_FusionTask(void *argument);
extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of usblinkRxcplt */
  usblinkRxcpltHandle = osSemaphoreNew(1, 0, &usblinkRxcplt_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	DVL_Handle = osSemaphoreNew(1, 0, &DVL_attributes);//信号量
	DVLRxcpltHandle = osSemaphoreNew(1, 0, &DVLRxcplt_attributes);
	EKF_IMUHandle = osSemaphoreNew(1, 1, &EKF_IMU_attributes);//用于USB
	DataSentHandle = osSemaphoreNew(1, 0, &DataSent_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of usbRxQue */
  usbRxQueHandle = osMessageQueueNew (1024, sizeof(uint8_t), &usbRxQue_attributes);
  usbtxHandle = osMessageQueueNew (1000, sizeof(atkp_t), &usbtx_attributes);//1000
 
  Uart2RxQueHandle = osMessageQueueNew (1024, sizeof(uint8_t), &Uart2RxQue_attributes);
  Uart2TxQueHandle = osMessageQueueNew (100, sizeof(atkp_t), &Uart2tx_attributes);//注意修改
  DVL_Position_QueHandle = osMessageQueueNew (1, sizeof(Axis3f), &DVL_Position_Que_attributes);
  DVL_Speed_QueHandle = osMessageQueueNew (1, sizeof(Axis3f), &DVL_Speed_Que_attributes);
  EKF_IMU_QueHandle = osMessageQueueNew (2, sizeof(Axis3f), &EKF_IMU_Que_attributes);
  EKF_Position_QueHandle = osMessageQueueNew (2, sizeof(IMUdata), &EKF_Position_Que_attributes);//融合后的位置队列加速度
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
   myQueue01Handle = osMessageQueueNew (1, sizeof(Axis3f), &myQueue01_attributes);
   
   depthHandle = osMessageQueueNew (1, sizeof(float), &depth_attributes);
   atpkrxHandle = osMessageQueueNew (10, sizeof(atkp_t), &atpkrx_attributes);
   
   
   DVL_wrp_posHandle = osMessageQueueNew (1, sizeof(DVLdata), &DVL_wrp_pos_attributes);
   DVL_wrz_velocityHandle = osMessageQueueNew (1, sizeof(DVLdata), &DVL_wrz_velocity_attributes);
   DVL_eurlHandle = osMessageQueueNew (1, sizeof(Axis3f), &DVL_eurl_attributes);
   DVL_attitudeHandle = osMessageQueueNew (1, sizeof(float), &DVL_attitude_attributes);
   
   
    GPSDataHandle = osMessageQueueNew (1, sizeof(GPRMC_Data), &GPSData_attributes);
   
   IMU_accHandle         = osMessageQueueNew (1, sizeof(IMUdata), &IMU_acc_attributes);
   IMU_freeaccHandle  = osMessageQueueNew (1, sizeof(Axis3f), &IMU_freeacc_attributes);
   IMU_gyroHandle        = osMessageQueueNew (1, sizeof(IMUdata), & IMU_gyro_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMURead */
  IMUReadHandle = osThreadNew(IMUReadTask, NULL, &IMURead_attributes);

  /* creation of atpk_tx */
  atpk_txHandle = osThreadNew(atpkTxTask, NULL, &atpk_tx_attributes);

  /* creation of usblinktx */
  usblinktxHandle = osThreadNew(usblinktxTask, NULL, &usblinktx_attributes);

  /* creation of DepthRead */
  DepthReadHandle = osThreadNew(DepthReadTask, NULL, &DepthRead_attributes);

  /* creation of usblink_Rx */
  usblink_RxHandle = osThreadNew(usblink_RxTask, NULL, &usblink_Rx_attributes);

  /* creation of atpk_rx */
  atpk_rxHandle = osThreadNew(atpkRxTask, NULL, &atpk_rx_attributes);

  /* creation of stabilizer */
  stabilizerHandle = osThreadNew(stabilizerTask, NULL, &stabilizer_attributes);

  /* creation of DVLReadTask */
  DVLReadTaskHandle = osThreadNew(DvlReadTask, NULL, &DVLReadTask_attributes);

  /* creation of GPSReadTask */
  GPSReadTaskHandle = osThreadNew(GpsReadTask, NULL, &GPSReadTask_attributes);

  /* creation of LED */
  LEDHandle = osThreadNew(ledTask11, NULL, &LED_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  Uart2linktxHandle = osThreadNew(Uart2linktxTask, NULL, &Uart2linktx_attributes);
  Uart2link_RxHandle = osThreadNew(Uart2link_RxTask, NULL, &Uart2link_Rx_attributes);
  EKF_UpdataHandle= osThreadNew(EKF_UpdataTask, NULL, &EKF_attributes);
  EKF_FusionHandle= osThreadNew(EKF_FusionTask, NULL, &EKF_Fusionattributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
uint8_t CPU_RunInfo[400]; 
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  HAL_TIM_Base_Start_IT(&htim17);
  Axis3f E;
  float dep;
  osDelay(8000);
// uint8_t dvlresetpos[]={"wcr,*64\r\n"};
// uint8_t dvlcommande[]={"wcv,*30\r\n"};
//	printf("发送！！！！！");
//	 HAL_UART_Transmit(&huart1,(uint8_t *)dvlcommande, sizeof(dvlcommande),1000); 
  /* Infinite loop */
  for(;;)
  {
    
    //ws2812_All_LED_one_Color_breath(10,C_pink,0);
    ///ws2812_Roll_on_Color_Ring(10);
	// ws2812_police();
	  

//	 getEurlRawData(&E);
//	 getDepthData(&dep);
//	 printf("Roll: %.2f,Pitch:%.2f,Yaw:%.2f\r\n",E.x,E.y,E.z);
//	 printf("DEPTH:%.2f",dep);
//	  
//         memset(CPU_RunInfo,0,400);                          //��Ϣ����������
//         vTaskList((char *)&CPU_RunInfo);  //��ȡ��������ʱ����Ϣ
//	    printf("---------------------------------------------\r\n");
//        printf("任务名    任务状态 优先级     剩余栈     任务序号\r\n");
//       
//         printf("%s", CPU_RunInfo);
//         printf("---------------------------------------------\r\n");

//         memset(CPU_RunInfo,0,400);                          //��Ϣ����������
//         vTaskGetRunTimeStats((char *)&CPU_RunInfo);
//          printf("任务名         运行计数    使用率\r\n");
//         printf("%s", CPU_RunInfo);
//         printf("---------------------------------------------\r\n\n");

         osDelay(1000);   /* ��ʱ500��tick */

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMUReadTask */
/**
* @brief Function implementing the IMURead thread.
* @param argument: Not used
* @retval None
*/\

/* USER CODE END Header_IMUReadTask */
__weak void IMUReadTask(void *argument)
{
  /* USER CODE BEGIN IMUReadTask */
   
  /* Infinite loop */
  for(;;)
  {
   osDelay(1);
  }
  /* USER CODE END IMUReadTask */
}

/* USER CODE BEGIN Header_atpkTxTask */
/**
* @brief Function implementing the atpk_tx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_atpkTxTask */
__weak void atpkTxTask(void *argument)
{
  /* USER CODE BEGIN atpkTxTask */
  
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1000);
  }
  /* USER CODE END atpkTxTask */
}

/* USER CODE BEGIN Header_usblinktxTask */
/**
* @brief Function implementing the usblinktx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usblinktxTask */
__weak void usblinktxTask(void *argument)
{
  /* USER CODE BEGIN usblinktxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usblinktxTask */
}

/* USER CODE BEGIN Header_DepthReadTask */
/**
* @brief Function implementing the DepthRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DepthReadTask */
__weak void DepthReadTask(void *argument)
{
  /* USER CODE BEGIN DepthReadTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DepthReadTask */
}

/* USER CODE BEGIN Header_usblink_RxTask */
/**
* @brief Function implementing the usblink_Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usblink_RxTask */
__weak void usblink_RxTask(void *argument)
{
  /* USER CODE BEGIN usblink_RxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usblink_RxTask */
}

/* USER CODE BEGIN Header_atpkRxTask */
/**
* @brief Function implementing the atpk_rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_atpkRxTask */
__weak void atpkRxTask(void *argument)
{
  /* USER CODE BEGIN atpkRxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END atpkRxTask */
}

/* USER CODE BEGIN Header_stabilizerTask */
/**
* @brief Function implementing the stabilizer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_stabilizerTask */
__weak void stabilizerTask(void *argument)
{
  /* USER CODE BEGIN stabilizerTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END stabilizerTask */
}

/* USER CODE BEGIN Header_DvlReadTask */
/**
* @brief Function implementing the DVLReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DvlReadTask */
__weak void DvlReadTask(void *argument)
{
  /* USER CODE BEGIN DvlReadTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DvlReadTask */
}

/* USER CODE BEGIN Header_GpsReadTask */
/**
* @brief Function implementing the GPSReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GpsReadTask */
__weak void GpsReadTask(void *argument)
{
  /* USER CODE BEGIN GpsReadTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GpsReadTask */
}

/* USER CODE BEGIN Header_ledTask11 */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ledTask11 */
void ledTask11(void *argument)
{
  /* USER CODE BEGIN ledTask11 */
  /* Infinite loop */
  for(;;)
  {
	//  ws2812_DualFlow(C_indigo, 500);
	 // ws2812_BreathingWave(C_indigo, 50, 14);
	  //ws2812_RainbowMeteor(10, 5, 0.1, 0);
//	  if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)==1&&osSemaphoreGetCount(DataSentHandle)==0)
//	  {
//		osSemaphoreRelease(DataSentHandle);
//	  }
	  if(MODE_ch == 0){
	   ws2812_SmoothMeteor(C_pink, 60, 6, 0) ;
	  }
	  else if(MODE_ch == 1){
	  ws2812_SmoothMeteor(C_pink, 10, 8, 1) ;
	  }
	  
	//ws2812_police();
    //osDelay(1);
  }
  /* USER CODE END ledTask11 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
__weak void Uart2link_RxTask(void *argument)
{
  /* USER CODE BEGIN usblink_RxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usblink_RxTask */
}
__weak void Uart2linktxTask(void *argument)
{
  /* USER CODE BEGIN usblinktxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usblinktxTask */
}
__weak void EKF_UpdataTask(void *argument)
{
  /* USER CODE BEGIN usblinktxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usblinktxTask */
}
/* USER CODE END Application */
__weak void EKF_FusionTask(void *argument)
{
  /* USER CODE BEGIN usblinktxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usblinktxTask */
}
/* USER CODE END Application */
