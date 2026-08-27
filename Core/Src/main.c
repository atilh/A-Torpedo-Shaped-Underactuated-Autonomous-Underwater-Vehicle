/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <main.h>
#include <cmsis_os.h>
#include <dma.h>
#include <usart.h>
#include <memorymap.h>
#include <spi.h>
#include <tim.h>
#include <usb_device.h>
#include <gpio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <imu.h>
#include <depth.h>
#include <stdlib.h>
#include <ws2812.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <DVL.h>
#include <GPS.h>
#include "moter.h"



Axis3f  EulerRaw2;

uint8_t IMUbufferReset[8]={0xFA,0xFF,0xA4,0x02,0x00,0x01,0x5A};
uint8_t IMUbufferReset_ALL[8]={0xFA,0xFF,0xA4,0x02,0x00,0x03,0x58};



extern osMessageQueueId_t myQueue01Handl;
extern osMessageQueueId_t depthHandle;
extern uint8_t depthbuffer[128];
extern osMessageQueueId_t Uart2RxQueHandle;
extern osMessageQueueId_t EKF_IMU_QueHandle;
extern osSemaphoreId_t DataSentHandle;
extern control_t 	control;

int fputc(int ch,FILE *f)
{
   HAL_UART_Transmit(&hlpuart1,(uint8_t *)&ch,1,0xFFFF);//hurat1Ϊ´®¿ںţ¬¸ù¾ݗԼºǩ¿ö½øАѡԱ
   return ch;
}







/*TESTTESTT________________________________________________*/



uint8_t ping2[256] = {0};
uint8_t lsuartbuffer[1024]={0};

/*TESTTESTT________________________________________________*/







/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
  
     if(huart->Instance==UART8){ 
       float depth,rem;
       sscanf((char*)depthbuffer, "T=%fD=%f", &rem, &depth);//解析字符串数据
       osMessageQueuePut( depthHandle, &depth,0, 0 );//放入队列
       
       //HAL_UARTEx_ReceiveToIdle_DMA(&huart8,(uint8_t *)depthbuffer, sizeof(depthbuffer));
        
     }
	 
	  if(huart->Instance==USART6){ 

	 printf("begin\r\n");
	  HAL_UART_Transmit(&hlpuart1,(uint8_t *)GPSbuffer, Size,1000); 
//	   HAL_UART_Transmit(&hlpuart1,(uint8_t *)"\r\n", 2,1000); 
		  printf("End\r\n");
         state_read(GPSbuffer);
     
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart6,(uint8_t *)GPSbuffer, sizeof(GPSbuffer));
     }

	 
	  if(huart->Instance==USART1){ 

//       HAL_UART_Transmit(&hlpuart1,(uint8_t *)"begin\r\n\r\n\r\n", sizeof("begin\r\n"),1000); 		  
     
           if(Size<50){
		  HAL_UART_Transmit(&hlpuart1,(uint8_t *)DVL, Size,1000); 
            }
//		SizeofDVL = Size;  
         
		process_dma_buffer(DVL, Size);
		
		osSemaphoreRelease(DVLRxcpltHandle);
       //HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *)DVL, sizeof(DVL));
     
     
     }
	  
	 if(huart->Instance==LPUART1){ 

	 
		// CDC_Transmit_FS(IMU, Size);
	   //CDC_Transmit_FS(lsuartbuffer,Size);	  
	   //HAL_UART_Transmit(&huart4,(uint8_t *)lsuartbuffer, Size,1000); 
		   
		HAL_UART_Transmit(&hlpuart1,(uint8_t *)lsuartbuffer, Size,1000);  
       HAL_UART_Transmit(&huart4,(uint8_t *)lsuartbuffer, Size,1000); 
       // memset(DVL,0,sizeof(DVL));		  
       HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1,(uint8_t *)lsuartbuffer, sizeof(lsuartbuffer));
     
     
     }
	 
	 	 if(huart->Instance==USART2){ 

		int i,j;
		j=30;//最大长度
		// CDC_Transmit_FS(DVL, Size);
	   //CDC_Transmit_FS(lsuartbuffer,Size);	  
	   //HAL_UART_Transmit_IT(&huart1,(uint8_t *)lsuartbuffer, Size); 
		   
//		HAL_UART_Transmit(&hlpuart1,(uint8_t *)ping2, Size,1000);  
       //HAL_UART_Transmit(&hlpuart1,(uint8_t *)"ok\r\n", sizeof("ok\r\n"),1000); 
       // memset(DVL,0,sizeof(DVL));	
			 while(j>0)//寻找最后一个零向量的索引，注意校验位也可能是0，解决问题不彻底
		{
			if((*(ping2+j))!=0x00)
			{
				break;
			}
			j--;
		}
		for(i=0;i<(j+1);i++)	//0数据截断无法读取，是一个bug，
		{
			osMessageQueuePut( Uart2RxQueHandle,(ping2+i),0,0);
		}	
       	HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t *)ping2, sizeof(ping2));
     
     
     }
		 
	 if(huart->Instance==UART4){ 
	   //HAL_UART_Transmit(&hlpuart1,(uint8_t *)IMU, Size,1000);  
		 IMUdata acc;
		 IMUdata gyro;
	     parse_mti630_data(IMU, Size,&mti_data);
//		printf("Roll: %.2f,Pitch:%.2f,Yaw:%.2f\r\n",mti_data.eurl.x,mti_data.eurl.y,mti_data.eurl.z);
//		 printf("XX: %.2f,YY:%.2f,ZZ:%.2f\r\n",mti_data.acc.x,mti_data.acc.y,mti_data.acc.z);
		 acc.IMU=mti_data.acc;
		 acc.timestamp=getRunTimeCounterValue();
		 gyro.IMU=mti_data.gyro;
		 gyro.timestamp=acc.timestamp;
		 osMessageQueuePut(EKF_IMU_QueHandle, &mti_data.acc, 0,  0 );//先加速度计再陀螺仪
		 osMessageQueuePut(EKF_IMU_QueHandle, &mti_data.gyro, 0,  0 );
         osMessageQueuePut( myQueue01Handle, &mti_data.eurl, 0,  0 ); 
		 osMessageQueuePut( IMU_accHandle, &acc, 0,  0 ); 
		 osMessageQueuePut( IMU_freeaccHandle, &mti_data.freeacc, 0,  0 ); 
		 osMessageQueuePut( IMU_gyroHandle, &gyro, 0,  0 ); 
//		 osSemaphoreRelease(EKF_IMUHandle);
		 
//      HAL_UARTEx_ReceiveToIdle_DMA(&huart4,(uint8_t *)IMU, sizeof(IMU));
     
     
     }

	 
	 
	 

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  
//      if(huart->Instance==UART4){ 
//      
//         parse_ahrs_data(imu_buffer, sizeof(imu_buffer),&EulerRaw2);
//         //memset(imu_buffer,0,sizeof(imu_buffer)); 
//         osMessageQueuePut( myQueue01Handle, 
//                                 &EulerRaw2,
//                                         0,                  
//                                         0 ); 
//        }

}
 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) {
			osSemaphoreRelease(DataSentHandle);
    }
	
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile unsigned long CPU_RunTime = 0UL; 
void configureTimerForRunTimeStats(void)
 {
     CPU_RunTime = 0UL;
 }

 unsigned long getRunTimeCounterValue(void)
 {
     return CPU_RunTime;
 }
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  
//  set_pixel_rgb(1,0);
//  set_pixel_rgb(11,0);
//  set_pixel_rgb(12,0);
//  set_pixel_rgb(13,0);
////  vTaskSuspendAll();
//  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 1500);
//  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 1500);
//  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 1000);
//  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 1000);
//  while(1){
//    
//  }
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART4_Init();
  MX_LPUART1_UART_Init();
  MX_TIM17_Init();
  MX_UART8_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI4_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
 
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
  
  ws2812_AllShutOff();
  HAL_Delay(4000);
  HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1,(uint8_t *)lsuartbuffer, sizeof(lsuartbuffer));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t *)ping2, sizeof(ping2));
  //control.thrust=200;
  //Control_PWM_Set(&control);   
 //HAL_UART_Transmit(&huart4,(uint8_t *)IMUbufferReset, sizeof(IMUbufferReset),1000);

	
	
  // HAL_UART_Transmit(&huart6,(uint8_t *)GPSbuffer, sizeof(GPSbuffer),1000);
  //HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1,(uint8_t *)lsuartbuffer, sizeof(lsuartbuffer));
  
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t *)ping2, sizeof(ping2));
 // HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1,(uint8_t *)lsuartbuffer, sizeof(lsuartbuffer));
  //  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *)DVL, sizeof(DVL));
 
 
  //HAL_UARTEx_ReceiveToIdle_DMA(&huart4,(uint8_t *)IMU, sizeof(IMU));
  //
	
	
	
	
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM16 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
 if(htim->Instance == TIM17 )
     {
         CPU_RunTime++;

     }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM16)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
