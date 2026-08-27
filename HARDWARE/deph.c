#include "depth.h"

float Depth;

uint8_t depthbuffer[128] = {0};











void DepthReadTask(void *argument)
{
  /* USER CODE BEGIN DepthReadTask */
  /* Infinite loop */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart8,(uint8_t *)depthbuffer, sizeof(depthbuffer));
  for(;;)
  {
    osMessageQueueGet(depthHandle, /* 消息队列的句柄 */
                           &Depth,/* 需要接受的消息内容存放地址 */
                           0,                        /* 接收优先级*/
                           osWaitForever);  /*等待1s */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8,(uint8_t *)depthbuffer, sizeof(depthbuffer));
    
   // osDelay(1);
  }
  /* USER CODE END DepthReadTask */
}



void getDepthData(float* f)
{
	*f = Depth;
}













