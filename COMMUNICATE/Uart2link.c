#include "Uart2link.h"
#include "usart.h"
#include "debug.h"
//#include <stm32h7xx_hal_uart.h>
bool Uart2GetDataWithTimout(uint8_t *c);
extern osMessageQueueId_t Uart2TxQueHandle;
extern osSemaphoreId_t DataSentHandle;
extern uint8_t ping2[256];
uint8_t *Uart2linkrx;
uint32_t *Uart2rslen ;
static atkp_t rxPacket;
static enum
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
  waitForChksum2,
}rxState;

//uint8_t A = 0;
/*usb连接发送atkpPacket*/
bool Uart2linkSendPacket(const atkp_t *p)
{
  
	return osMessageQueuePut(Uart2TxQueHandle, p, 0,0);	
}


atkp_t p2_Uart2;
void Uart2linktxTask(void const * argument)//Usb发送任务
{
	
  uint8_t sendBuffer1[64];
	uint8_t cksum;
  uint8_t addsum;
	uint8_t dataLen;
	while(1)
	{
    
		osMessageQueueGet(Uart2TxQueHandle, &p2_Uart2,0, osWaitForever);//从队列里读取数据
		cksum = 0;
    addsum = 0;
		sendBuffer1[0] = UP_BYTE1;//起始位

		sendBuffer1[1] = UP_BYTE2;//起始位

		sendBuffer1[2] = p2_Uart2.msgID;//包的Id

		sendBuffer1[3] = p2_Uart2.dataLen;//包的长度

		memcpy(&sendBuffer1[4], p2_Uart2.data, p2_Uart2.dataLen);//储存到数组
		
		for (int i = 0; i < p2_Uart2.dataLen+4; i++)
		{
			cksum += sendBuffer1[i];
            addsum+= cksum;
		}
		dataLen = p2_Uart2.dataLen + 6;
		sendBuffer1[dataLen - 2] = cksum;
    sendBuffer1[dataLen - 1] = addsum;//校验位
    HAL_UART_Transmit_IT(&huart2,(uint8_t *)sendBuffer1, dataLen);  
//    while(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TC)!=1);
		osSemaphoreAcquire(DataSentHandle,osWaitForever);
//	osDelay(1);
//		CDC_Transmit_FS(sendBuffer1,dataLen);//发送一个数据包USB
  	
  }
}



uint8_t bud7[50];
void Uart2link_RxTask(void const * argument)//接收来自电脑发送的数据包
{
  /* USER CODE BEGIN usblink_RxTask */
  uint8_t c;
	uint8_t dataIndex = 0;
	uint8_t cksum = 0;
  uint8_t addsum = 0;
	rxState = waitForStartByte1;
	int ii=0;
 HAL_UARTEx_ReceiveToIdle_DMA(&huart2,(uint8_t *)ping2, sizeof(ping2));
  /* Infinite loop */
  for(;;)
  {
	  
    if (Uart2GetDataWithTimout(&c))//能否读到队列？？？
		{
			bud7[ii]=c;
			ii++;
			switch(rxState)
			{
				case waitForStartByte1:
					rxState = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
					cksum = c;
          addsum = cksum;

					break;
				case waitForStartByte2:
					rxState = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte1;
					cksum += c;
          addsum += cksum;
         

					break;
				case waitForMsgID:
					rxPacket.msgID = c;
					rxState = waitForDataLength;
					cksum += c;
          addsum += cksum;
   				break;
				
        case waitForDataLength:
					if (c <= ATKP_MAX_DATA_SIZE)
					{
						rxPacket.dataLen = c;
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
						//rxState =waitForData;
						cksum += c;
            addsum += cksum;

					} else 
					{
						rxState = waitForStartByte1;
					}
					break;
				
        case waitForData:
					rxPacket.data[dataIndex] = c;
					dataIndex++;
					cksum += c;
          addsum += cksum;
 
					if (dataIndex == rxPacket.dataLen)
					{
						rxState = waitForChksum1;
					}
					break;
				
        case waitForChksum1:
					if (cksum == c)	/*所有校验正确*/
					{

						rxState = waitForChksum2;
					} 
					else	/*校验错误*/
					{
						rxState = waitForStartByte1;	
						IF_DEBUG_ASSERT(1);
					}
					
					break;
          
        case waitForChksum2:
					if (addsum == c)	/*所有校验正确*/
					{   
						atkpReceivePacketBlocking(&rxPacket);  
					} 
					else	/*校验错误*/
					{
						rxState = waitForStartByte1;	
						IF_DEBUG_ASSERT(1);
					}
          addsum = 0;
					rxState = waitForStartByte1;
					break;
 
				default:
					ASSERT(0);
					break;
			}
		}
		else	/*超时处理*/
		{
			rxState = waitForStartByte1;
		}

  }

  /* USER CODE END usblink_RxTask */
}
bool Uart2GetDataWithTimout(uint8_t *c)
{
//  osStatus_t xReturn = osOK;
	if (osMessageQueueGet(Uart2RxQueHandle, c,0, 1000) == osOK)	/*????usbDataDelivery(1024??????)???*/
	{
		return true;
	}
	*c = 0;
	return false;
}