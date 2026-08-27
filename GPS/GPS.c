#include "GPS.h"
#include "stdio.h"

uint8_t GPSbuffer[2048]={0};
GPRMC_Data  GPS_data_Que;
	GPRMC_Data  GPS_data;
void state_read(uint8_t s[])
{
	

	int i = 0;
	int j = 0;
	while (s[i++] != '$');
	while (s[i++] != '$');
	i = i + 5;
	if (s[i - 2] == 'M' && s[i - 1] == 'C')//
	{
		i = seek_i(i, s);
		GPS_data.GPS_bit.state_bit = ((s[i] == 'A')|(s[i] == 'D'));
		GPS_data.GPS_bit.RTK_bit = (s[i] == 'D');
		i = seek_i(i, s);
		if (GPS_data.GPS_bit.state_bit == 1)
		{
			for (j = 0, GPS_data.latitude_deg = 0;j < 2;j++)
			{
				GPS_data.latitude_deg = s[i++] - '0' + GPS_data.latitude_deg * 10;//����γ�ȵĶ�
			}
			GPS_data.latitude_min = transs(s, i);//����γ�ȵķ�
			i = seek_i(i, s);
			GPS_data.GPS_bit.latitude_bit = (s[i] == 'N');//1��0��
			i = seek_i(i, s);
			for (j = 0, GPS_data.longitude_deg = 0;j < 3;j++)
			{
				GPS_data.longitude_deg = s[i++] - '0' + GPS_data.longitude_deg * 10;//���㾭�ȵĶ�
			}
			GPS_data.longitude_min = transs(s, i);//����γ�ȵķ�
			i = seek_i(i, s);
			GPS_data.GPS_bit.longitude_bit = (s[i] == 'E');//1��0��
			i = seek_i(i, s);
			i = seek_i(i, s);
			for (j = 0;j < 4;j++)
			{
				i = seek_i(i, s);
			}
			
			while (s[i++] != '$');
			if (s[i + 2] == 'H')
			{
				i = seek_i(i, s);
				if (!isdigit(s[i]))
				{

				}
				else
				{
					GPS_data.heading_angle = transs(s, i);
				}
			}
		}
		else
		{
            GPS_data.GPS_bit.latitude_bit = 0;
	        GPS_data.GPS_bit.longitude_bit = 0;		
			GPS_data.GPS_bit.state_bit = 0;		
			GPS_data.GPS_bit.RTK_bit = 0;	
			GPS_data.heading_angle=1;
			GPS_data.latitude_deg=1;
			GPS_data.latitude_min=10;
			GPS_data.longitude_deg=80;
			GPS_data.longitude_min=120;
		}
	}
	else
	{
		
		printf("ERROR!!!!\r\n");
	}
	
	 osMessageQueuePut(GPSDataHandle, &GPS_data, 0,  0); 
}
int seek_i(int i, uint8_t s[])
{
	while (s[++i] != ',');
	while (s[++i] == ' ');
	return i;
}
float transs(uint8_t s[], int i)//���ַ���ת��ΪС��
{

	float val, d;
	for (val = 0.0;isdigit(s[i]);i++)
	{
		val = val * 10.0 + s[i] - '0';
	}
	i++;
	for (d = 1.0;isdigit(s[i]);i++)
	{
		val = val * 10.0 + s[i] - '0';
		d *= 10.0;
	}
	return   val / d;
}









void GpsReadTask(void *argument)
{
  /* USER CODE BEGIN GpsReadTask */
  /* Infinite loop */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,(uint8_t *)GPSbuffer, sizeof(GPSbuffer));
  for(;;)
  {
	  
	  
	  
	osMessageQueueGet( GPSDataHandle, &GPS_data_Que, 0, osWaitForever);  

	  
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6,(uint8_t *)GPSbuffer, sizeof(GPSbuffer));
    //osDelay(1);
  }
  /* USER CODE END GpsReadTask */
}


void getGPSRawData(GPRMC_Data *gps)
{
	//*gps = GPS_data_Que;
	*gps = GPS_data;
}










