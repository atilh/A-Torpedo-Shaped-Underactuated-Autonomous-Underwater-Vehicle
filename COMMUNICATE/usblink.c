#include "usblink.h"
#include "usart.h"
#include "debug.h"
extern osSemaphoreId_t EKF_IMUHandle;
extern osMessageQueueId_t usbtxHandle;
uint8_t *usblinkrx;
uint32_t *usbrslen ;
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

uint8_t A = 0;
/*usb连接发送atkpPacket*/
bool usblinkSendPacket(const atkp_t *p)
{
  
//	return osMessageQueuePut(usbtxHandle, p, 0,0);	
	return osMessageQueuePut(usbtxHandle, p, 0,0);
}


atkp_t p2;
atkp_t p3;
void usblinktxTask(void const * argument)//Usb发送任务
{
	
  uint8_t sendBuffer1[128];//最大发送64字节
	uint8_t cksum;
  uint8_t addsum;
	uint8_t dataLen=0;
	uint8_t Len=0;
	osStatus_t Status_Now=0;
	int j=0;
	uint8_t flag=0;
	while(1)
	{	
		if(flag==1)
		{
			flag=0;//清除标志位
			cksum = 0;
			addsum = 0;
			sendBuffer1[Len+0] = UP_BYTE1;//起始位

			sendBuffer1[Len+1] = UP_BYTE2;//起始位

			sendBuffer1[Len+2] = p2.msgID;//包的Id

			sendBuffer1[Len+3] = p2.dataLen;//包的长度

			memcpy(&sendBuffer1[Len+4], p2.data, p2.dataLen);//储存到数组

			for (int i = 0; i < p2.dataLen+4; i++)
			{
				cksum += sendBuffer1[Len+i];
				addsum+= cksum;
			}
			dataLen = p2.dataLen + 6;
			sendBuffer1[Len+dataLen - 2] = cksum;
			sendBuffer1[Len+dataLen - 1] = addsum;//校验位
			Len+=dataLen;
			
		}
		while(Len<64)
		{
			osMessageQueueGet(usbtxHandle, &p2,0, osWaitForever);//从队列里读取数据
			if(p2.dataLen+6+Len>64)
			{
				flag=1;//有剩余的包
				break;
			}
			cksum = 0;
			addsum = 0;
			sendBuffer1[Len+0] = UP_BYTE1;//起始位

			sendBuffer1[Len+1] = UP_BYTE2;//起始位

			sendBuffer1[Len+2] = p2.msgID;//包的Id

			sendBuffer1[Len+3] = p2.dataLen;//包的长度

			memcpy(&sendBuffer1[Len+4], p2.data, p2.dataLen);//储存到数组

			for (int i = 0; i < p2.dataLen+4; i++)
			{
				cksum += sendBuffer1[Len+i];
				addsum+= cksum;
			}
			dataLen = p2.dataLen + 6;
			sendBuffer1[Len+dataLen - 2] = cksum;
			sendBuffer1[Len+dataLen - 1] = addsum;//校验位
			Len+=dataLen;
		}
			CDC_Transmit_FS(sendBuffer1,Len);//发送一个数据包USB
			Len=0;
			osSemaphoreAcquire(EKF_IMUHandle,osWaitForever);
	}

			
 

}




void usblink_RxTask(void const * argument)//接收来自电脑发送的数据包
{
  /* USER CODE BEGIN usblink_RxTask */
  uint8_t c;
	uint8_t dataIndex = 0;
	uint8_t cksum = 0;
  uint8_t addsum = 0;
	rxState = waitForStartByte1;

  /* Infinite loop */
  for(;;)
  {
    if (usbGetDataWithTimout(&c))//能否读到队列？？？
		{

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
					if (c <= ATKP_MAX_DATA_SIZE)//30
					{
						rxPacket.dataLen = c;
						dataIndex = 0;
						rxState = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
						//rxState=waitForData;
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