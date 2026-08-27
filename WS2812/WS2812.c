
/************************************************************
	* @FileName:     	ws2812.c
	* @Author:				Hermes
	* @Version:				
	* @Date:					
	* @Description:  SPI + DMA ·½Ê½ÊµÏÖ¹éÁãÂë·¢ËÍ µÆ´øÇý¶¯Ä£¿é
*************************************************************/
#include "WS2812.h" 
#include "string.h"
uint8_t  ws2812_data_buffer[WS2812_LED_NUM][24] ;
uint8_t  dma_data_buffer[15*24+2];
RGB_Color  rgb_color;
HSV_Color  hsv_color;








/**
 * @Description  	WS2812µÆ´ø³õÊ¼»¯Ö÷µ÷º¯Êý
 * @Param     	  {void}
 * @Return    	  {void}
*/
void ws2812_Init(void){

  ws2812_AllShutOff();
  HAL_Delay(WS2812_LED_NUM * 10);
}


/**
 * @Description  	WS2812 Æô¶¯DMA´«Êä
 * @Param     	  {void}
 * @Return    	  {void}
*/
void ws2812_Send_Data(void){
  //memset(dma_data_buffer,0,sizeof(dma_data_buffer));
  dma_data_buffer[0] = 0x00;
  memcpy(&dma_data_buffer[1],ws2812_data_buffer,sizeof(ws2812_data_buffer));
//
  HAL_SPI_Transmit_DMA(&hspi4,(uint8_t*)dma_data_buffer,sizeof(dma_data_buffer)); 
}

//½«ÈýÔ­É«µ¥¶ÀÊý¾ÝºÏ²¢Îª24Î»Êý¾Ý
uint32_t ws281x_color(uint8_t red, uint8_t green, uint8_t blue)
{
  return green << 16 | red << 8 | blue;
}

/**
 * @Description  	WS2812 ÉèÖÃµÚn¸öµÆÖéµÄÑÕÉ«
* @Param     n:µÚ¼¸¸öµÆÖé   red:0-255   green:0-255    blue:0-255 	   eg:yellow:255 255 0
 * @Return    	  
*/

//Éè¶¨µÚn¸öµÆÖéµÄÑÕÉ«
void ws281x_setPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue)
{
  uint8_t i;
  
  if(n < WS2812_LED_NUM)
  {
    for(i = 0; i < 24; ++i)
    {
      ws2812_data_buffer[n][i] = (((ws281x_color(red,green,blue) << i) & 0X800000) ? SIG_1 : SIG_0);
    }
  }
	ws2812_Send_Data();
	
}


/**
 * @Description  	WS2812 ÉèÖÃµÆÖéÑÕÉ«£¨¹Ì¶¨µÄ£©
* @Param       		 n:µÚ¼¸¸öµÆÖé   color:ÄÄÖÖÑÕÉ«£¨0-7£©
 * @Return    	  
*/
void set_pixel_rgb(uint16_t n,uint8_t color)
{
	switch(color)
	{
		case Red: 
			ws281x_setPixelRGB(n,255,0,0);
			break;
		case Green: 
			ws281x_setPixelRGB(n,0,255,0);
			break;
		case Blue: 
			ws281x_setPixelRGB(n,0,0,255);
			break;
		case Yellow: 
			ws281x_setPixelRGB(n,255,255,0);
			break;
		case Purple: 
			ws281x_setPixelRGB(n,255,0,255);
			break;
		case Orange: 
			ws281x_setPixelRGB(n,255,125,0);
			break;
		case Indigo: 
			ws281x_setPixelRGB(n,0,255,255);
			break;
		case White:
			ws281x_setPixelRGB(n,255,255,255);
			break;
	  case Sexy:
      ws281x_setPixelRGB(n,160,32,240);
			break;
	}

}



//ÉèÖÃ¹Ø±ÕµÚn¸öµÆÖé
void ws281x_ShutoffPixel(uint16_t n)
{
  uint8_t i;
  
  if(n < WS2812_LED_NUM)
  {
    for(i = 0; i < 24; ++i)
    {
      ws2812_data_buffer[n][i] = SIG_0;
    }
  }
	ws2812_Send_Data();

}



/**
 * @Description  	WS2812¹Ø±ÕËùÓÐµÆ¹â		1. ·¢ËÍWS2812_LED_NUM * 24Î»µÄ 0 Âë
																
 * @Param     	  {void}
 * @Return    	  {void}
*/
void ws2812_AllShutOff(void){
	uint16_t i;
  uint8_t j;
  
  for(i = 0; i < WS2812_LED_NUM; i++)
  {
    for(j = 0; j < 24; j++)
    {
      ws2812_data_buffer[i][j] = SIG_0;
    }
  }
  ws2812_Send_Data();
	HAL_Delay(10*WS2812_LED_NUM);
}


/**
 * @Description  	WS2812ÉèÖÃÄ³Ò»Î»µÄLEDµÄÑÕÉ« µ«²»·¢ËÍ
 * @Param     	  {uint16_t LED_index ,uint32_t GRB_color}
 * @Return    	  {void}
*/
void ws2812_Set_one_LED_Color(uint16_t LED_index ,uint32_t GRB_color){
  uint8_t i = 0;
	uint32_t cnt = 0x800000;
  if(LED_index < WS2812_LED_NUM){
    for(i = 0; i < 24; ++i){
			if(GRB_color & cnt){
				ws2812_data_buffer[LED_index][i] = SIG_1;
			}
			else{
				ws2812_data_buffer[LED_index][i] = SIG_0;
			}
			cnt >>= 1;
    }
  }
}




void ws2812_Toggle(uint16_t LED_index,uint32_t GRB_color,uint8_t flag){

   
  
   if(flag){
   
   set_pixel_rgb(LED_index,GRB_color);
   }
   else{
   
   ws281x_ShutoffPixel(LED_index);
   }

   

}

/**
 * @Description  	WS2812 É«»·×ª»¯ 0-255»Ò¶ÈÖµ×ª»»ÎªGRBÖµ
 * @Param     	  {uint8_t LED_gray}
 * @Return    	  {uint32_t}
*/
uint32_t ws2812_LED_Gray2GRB(uint8_t LED_gray){
	LED_gray = 0xFF - LED_gray;
	if(LED_gray < 85){
		return (((0xFF - 3 * LED_gray)<<8) | (3 * LED_gray));
	}
	if(LED_gray < 170){
		LED_gray = LED_gray - 85;
		return (((3 * LED_gray)<<16) | (0xFF - 3 * LED_gray));
	}
	LED_gray = LED_gray - 170;
	return (((0xFF - 3 * LED_gray)<<16) | ((3 * LED_gray)<<8));
}


/**
 * @Description  	WS2812 »Ò¶ÈÖµÇý¶¯½¥±äÐ§¹û ÑØÉ«»·×ª¶¯
 * @Param     	  {uint16_t interval_time} ½¥±ä¼ä¸ôÊ±¼ä
 * @Return    	  {void}
*/
void ws2812_Roll_on_Color_Ring(uint16_t interval_time){
	uint8_t i = 0;
	uint16_t j = 0;
	for(i = 0;i <= 255;i++){
		for(j = 0;j < WS2812_LED_NUM;j++){
			ws2812_Set_one_LED_Color(j, ws2812_LED_Gray2GRB(i));
		}
		ws2812_Send_Data();
		osDelay(interval_time);
	}
}

/**
 * @Description  	WS2812 µ¥É«ºôÎüµÆ °µ->ÁÁ->°µ
 * @Param     	  {uint16_t interval_time, uint32_t GRB_color} ½¥±ä¼ä¸ôÊ±¼ä
 * @Return    	  {void}
*/
void ws2812_All_LED_one_Color_breath(uint16_t interval_time, uint32_t GRB_color,uint16_t LED_index){
	uint8_t i = 0;
	uint16_t j = 0;
	rgb_color.G = GRB_color>>16;
	rgb_color.R = GRB_color>>8;
	rgb_color.B = GRB_color;
	for(i=1;i<=100;i++){
		__brightnessAdjust(i/100.0f, rgb_color);
		
			ws2812_Set_one_LED_Color(LED_index, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+1, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//          ws2812_Set_one_LED_Color(LED_index+2, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+3, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    ws2812_Set_one_LED_Color(LED_index+4, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+5, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    ws2812_Set_one_LED_Color(LED_index+6, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+7, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    		  ws2812_Set_one_LED_Color(LED_index+8, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    ws2812_Set_one_LED_Color(LED_index+9, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+10, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
		ws2812_Send_Data();
		osDelay(interval_time);
	}
	for(i=100;i>=1;i--){
		__brightnessAdjust(i/100.0f, rgb_color);
		ws2812_Set_one_LED_Color(LED_index, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+1, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    ws2812_Set_one_LED_Color(LED_index+2, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+3, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    ws2812_Set_one_LED_Color(LED_index+4, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+5, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    ws2812_Set_one_LED_Color(LED_index+6, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+7, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//      ws2812_Set_one_LED_Color(LED_index+8, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//    ws2812_Set_one_LED_Color(LED_index+9, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
//		  ws2812_Set_one_LED_Color(LED_index+10, ((rgb_color.G<<16) | (rgb_color.R<<8) | (rgb_color.B)));
		ws2812_Send_Data();
		osDelay(interval_time);
	}
}

/**
 * @Description  	ÅÜÂíµÆÐ§¹û
* @Param     interval_time:¼ä¸ôÊ±¼ä
 * @Return    	NONE  
*/
///void horse_race_lamp(uint16_t interval_time)
//{
//	u8 i,color;
//	
//	
//  for(i = 0; i < WS2812_LED_NUM; i++)
//  {
////		ws281x_setPixelRGB(i,255,255,0);
//		color = rand()%7;
//		set_pixel_rgb(i,color);//Ëæ»úÑÕÉ«
//		ws281x_ShutoffPixel(i-1);
//		delay_ms(interval_time);
//  }
//	ws281x_ShutoffPixel(WS2812_LED_NUM-1);
//	delay_ms(interval_time);
//}


/**
 * @Description  	Á÷Ë®µÆÐ§¹û
* @Param     interval_time:¼ä¸ôÊ±¼ä  red:0-255 green:0-255 blue:0-255
 * @Return    	NONE  
*/
///void Running_water_lamp( uint8_t red ,uint8_t green ,uint8_t blue, uint16_t interval_time )
//{
///	uint16_t i;
//  
//  for(i = 0; i < WS2812_LED_NUM; i++)
//  {
//		ws281x_setPixelRGB(i,red,green,blue);
//		delay_ms(interval_time);
//  }
//	ws2812_AllShutOff();
//	delay_ms(interval_time);
//}
/**
 * @Description  	µãÁÁËùÓÐµÆ
* @Param     	 NONE
 * @Return    	NONE  
*/
void ws2812_AllOpen(uint8_t red ,uint8_t green ,uint8_t blue)
{
	uint16_t i,j;
  
	for(j = 0;j<WS2812_LED_NUM;j++)
  {
    for(i = 0; i < 24; ++i)
    {
      ws2812_data_buffer[j][i] = (((ws281x_color(red,green,blue) << i) & 0X800000) ? SIG_1 : SIG_0);
    }
  }
	ws2812_Send_Data();
	HAL_Delay(10);
}



/**
 * @Description  	Ëæ»úµãÁÁRGBµÆ
* @Param     interval_time:¼ä¸ôÊ±¼ä
 * @Return    	NONE  
*/
///uint8_t tmp_flag[WS2812_LED_NUM];


///void srand_lamp(uint16_t interval_time)
//{
///	static uint8_t tmp,i;
//	uint8_t k,color;

//	tmp = rand()%(WS2812_LED_NUM);
//	color = rand()%7;
//	if(i==0) //Ö»×öÒ»´Î
//	{
//		memset(tmp_flag,50,WS2812_LED_NUM);
//		tmp_flag[i] = tmp;
//		set_pixel_rgb(tmp,color);
//		delay_ms(interval_time);
//		i++;
//	
//	}
//	else if(i>=WS2812_LED_NUM)
//	{
//		return ;
//	}
//		
//	for(k=0;k<i;k++)
//	{
//		if(tmp == tmp_flag[k])//ÏàÍ¬¾ÍÍË³ö
//		{
//			return ;
//		}
//		
//	}

//	//±éÀúÍê³É
//	tmp_flag[i] = tmp;
//	set_pixel_rgb(tmp,color);
//	delay_ms(interval_time);
//	i++;


//}

void ws2812_police_group(uint8_t group,uint32_t GRB_color){

	
	if(group == 1){
	      
		  ws2812_Set_one_LED_Color(0,GRB_color);
          ws2812_Set_one_LED_Color(1,GRB_color);
	      ws2812_Set_one_LED_Color(2,GRB_color);

	}
	else  if(group == 2){
	      
		  ws2812_Set_one_LED_Color(3,GRB_color);

	}
    else  if(group == 3){
	      
	ws2812_Set_one_LED_Color(4,GRB_color);
	ws2812_Set_one_LED_Color(5,GRB_color);
	ws2812_Set_one_LED_Color(6,GRB_color);

	}
    else  if(group == 4){
	      
	ws2812_Set_one_LED_Color(7,GRB_color);
	ws2812_Set_one_LED_Color(8,GRB_color);
	ws2812_Set_one_LED_Color(9,GRB_color);
	}
    else  if(group == 5){
	      
	 ws2812_Set_one_LED_Color(10,GRB_color);

	}
    else  if(group == 6){
	      
	ws2812_Set_one_LED_Color(11,GRB_color);
	ws2812_Set_one_LED_Color(12,GRB_color);
	ws2812_Set_one_LED_Color(13,GRB_color);

	}



}





void ws2812_police()
{
	
	for(int i = 0 ; i<20;i++){
	ws2812_police_group(1,C_Red);
	ws2812_police_group(2,C_Red);
	ws2812_police_group(3,C_Red);
	ws2812_police_group(4,OFF);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,OFF);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,OFF);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,OFF);
	ws2812_police_group(4,OFF);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,OFF);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,C_Red);
	ws2812_police_group(2,C_Red);
	ws2812_police_group(3,C_Red);
	ws2812_police_group(4,OFF);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,OFF);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,OFF);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,OFF);
	ws2812_police_group(4,OFF);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,OFF);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,OFF);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,OFF);
	ws2812_police_group(4,C_Blue);
	ws2812_police_group(5,C_Blue);
	ws2812_police_group(6,C_Blue);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,OFF);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,OFF);
	ws2812_police_group(4,OFF);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,OFF);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,OFF);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,OFF);
	ws2812_police_group(4,C_Blue);
	ws2812_police_group(5,C_Blue);
	ws2812_police_group(6,C_Blue);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,OFF);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,OFF);
	ws2812_police_group(4,OFF);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,OFF);
	ws2812_Send_Data();
	osDelay(100);
    }
	
	
	
	
	
	
	for(int i = 0 ; i<20;i++){
	ws2812_police_group(1,C_Blue);
	ws2812_police_group(2,C_white);
	ws2812_police_group(3,C_Red);
	ws2812_police_group(4,C_Blue);
	ws2812_police_group(5,C_white);
	ws2812_police_group(6,C_Red);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,C_Red);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,C_Blue);
	ws2812_police_group(4,C_Red);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,C_Blue);
	ws2812_Send_Data();
	osDelay(100);
	
	ws2812_police_group(1,C_Blue);
	ws2812_police_group(2,C_white);
	ws2812_police_group(3,C_Red);
	ws2812_police_group(4,C_Blue);
	ws2812_police_group(5,C_white);
	ws2812_police_group(6,C_Red);
	ws2812_Send_Data();
	osDelay(100);
	ws2812_police_group(1,C_Red);
	ws2812_police_group(2,OFF);
	ws2812_police_group(3,C_Blue);
	ws2812_police_group(4,C_Red);
	ws2812_police_group(5,OFF);
	ws2812_police_group(6,C_Blue);
	ws2812_Send_Data();
	osDelay(100);
	
    }

	
}




/***********************************************************
										Private Function
************************************************************/
/**
 * @Description  	»ñµÃÁ½Êý×î´óÖµ
 * @Param     	  {float a,float b}
 * @Return    	  {float}
*/
float __getMaxValue(float a, float b){
	return a>=b?a:b;
}

/**
 * @Description  	»ñµÃÁ½Êý×îÐ¡Öµ
 * @Param     	  {void}
 * @Return    	  {void}
*/
float __getMinValue(float a, float b){
	return a<=b?a:b;
}


/**
 * @Description  	RGB ×ªÎª HSV
 * @Param     	  {RGB_Color RGB, HSV_Color *HSV}
 * @Return    	  {void}
*/
void __RGB_2_HSV(RGB_Color RGB, HSV_Color *HSV){
	float r,g,b,minRGB,maxRGB,deltaRGB;
	
	r = RGB.R/255.0f;
	g = RGB.G/255.0f;
	b = RGB.B/255.0f;
	maxRGB = __getMaxValue(r, __getMaxValue(g,b));
	minRGB = __getMinValue(r, __getMinValue(g,b));
	deltaRGB = maxRGB - minRGB;
	
	HSV->V = deltaRGB;
	if(maxRGB != 0.0f){
		HSV->S = deltaRGB / maxRGB;
	}
	else{
		HSV->S = 0.0f;
	}
	if(HSV->S <= 0.0f){
		HSV->H = 0.0f;
	}
	else{
		if(r == maxRGB){
			HSV->H = (g-b)/deltaRGB;
    }
    else{
			if(g == maxRGB){
        HSV->H = 2.0f + (b-r)/deltaRGB;
      }
      else{
				if (b == maxRGB){
					HSV->H = 4.0f + (r-g)/deltaRGB;
        }
      }
    }
    HSV->H = HSV->H * 60.0f;
    if (HSV->H < 0.0f){
			HSV->H += 360;
    }
    HSV->H /= 360;
  }
}


/**
 * @Description  	HSV ×ªÎª RGB
 * @Param     	  {void}
 * @Return    	  {void}
*/
void __HSV_2_RGB(HSV_Color HSV, RGB_Color *RGB){
	float R,G,B,aa,bb,cc,f;
  int k;
  if (HSV.S <= 0.0f)
		R = G = B = HSV.V;
  else{
		if (HSV.H == 1.0f){
			HSV.H = 0.0f;
		}
    HSV.H *= 6.0f;
    k = (int)floor(HSV.H);
    f = HSV.H - k;
    aa = HSV.V * (1.0f - HSV.S);
    bb = HSV.V * (1.0f - HSV.S * f);
    cc = HSV.V * (1.0f -(HSV.S * (1.0f - f)));
    switch(k){
      case 0:
       R = HSV.V; 
       G = cc; 
       B =aa;
       break;
      case 1:
       R = bb; 
       G = HSV.V;
       B = aa;
       break;
      case 2:
       R =aa;
       G = HSV.V;
       B = cc;
       break;
      case 3:
       R = aa;
       G = bb;
       B = HSV.V;
       break;
      case 4:
       R = cc;
       G = aa;
       B = HSV.V;
       break;
      case 5:
       R = HSV.V;
       G = aa;
       B = bb;
       break;
    }
  }
  RGB->R = (unsigned char)(R * 255);
  RGB->G = (unsigned char)(G * 255);
  RGB->B = (unsigned char)(B * 255);
}


/**
 * @Description  	ÁÁ¶Èµ÷½Ú
 * @Param     	  {void}
 * @Return    	  {void}
*/
void __brightnessAdjust(float percent, RGB_Color RGB){
	if(percent < 0.01f){
		percent = 0.01f;
	}
	if(percent > 1.0f){
		percent = 1.0f;
	}
	__RGB_2_HSV(RGB, &hsv_color);
	hsv_color.V = percent;
	__HSV_2_RGB(hsv_color, &rgb_color);
}
/************************************************************
														EOF
*************************************************************/
void ws2812_DualFlow(uint32_t color, uint16_t speed) {
    for(uint16_t i = 0; i < WS2812_LED_NUM; i++) {
        // æ­£å‘æµåŠ¨
        ws2812_Set_one_LED_Color(i, color);
        ws2812_Set_one_LED_Color((i==0) ? WS2812_LED_NUM-1 : i-1, OFF);
        ws2812_Send_Data();
        osDelay(speed);
    }
    
    for(uint16_t i = WS2812_LED_NUM-1; i > 0; i--) {
        // åå‘æµåŠ¨
        ws2812_Set_one_LED_Color(i, color);
        ws2812_Set_one_LED_Color((i==WS2812_LED_NUM-1) ? 0 : i+1, OFF);
        ws2812_Send_Data();
        osDelay(speed);
    }
}



void ws2812_BreathingWave(uint32_t base_color, uint16_t speed, uint8_t wave_length) {
    RGB_Color base_rgb;
    base_rgb.G = (base_color >> 16) & 0xFF;
    base_rgb.R = (base_color >> 8) & 0xFF;
    base_rgb.B = base_color & 0xFF;

    float phase = 0.0f;
    while (1) {
        for (int i = 0; i < WS2812_LED_NUM; i++) {
            // è®¡ç®—æ­£å¼¦æ³¢äº®åº¦
            float brightness = 0.5f * (1 + sin(2 * 3.14 * (phase + (float)i / wave_length)));
            __brightnessAdjust(brightness, base_rgb);
            uint32_t grb = (rgb_color.G << 16) | (rgb_color.R << 8) | rgb_color.B;
            ws2812_Set_one_LED_Color(i, grb);
        }
        ws2812_Send_Data();
        phase += 0.1f; // æŽ§åˆ¶æ³¢æµªç§»åŠ¨
        if (phase > 1.0f) phase -= 1.0f;
        osDelay(speed);
    }
}


void ws2812_SmoothMeteor(uint32_t color, uint16_t speed, uint8_t tail_length, uint8_t reverse) {
    static int last_positions[20] = {-1};
    static int last_head = -1;

    RGB_Color meteor_color;
    meteor_color.G = (color >> 16) & 0xFF;
    meteor_color.R = (color >> 8) & 0xFF;
    meteor_color.B = color & 0xFF;

    for (int i = 0; i < WS2812_LED_NUM + tail_length; i++) {
        // æ¸…é™¤ä¸Šä¸€å¸§çš„æ‹–å°¾å’Œå¤´éƒ¨
        for (int j = 0; j < tail_length; j++) {
            int old_pos = last_positions[j];
            if (old_pos >= 0 && old_pos < WS2812_LED_NUM) {
                ws2812_Set_one_LED_Color(old_pos, OFF);
            }
        }
        if (last_head >= 0 && last_head < WS2812_LED_NUM) {
            ws2812_Set_one_LED_Color(last_head, OFF);
        }

        // è®¡ç®—å½“å‰å¸§çš„æ‹–å°¾ä½ç½®
        int current_head = reverse ? (WS2812_LED_NUM - 1 - i) : i;
        for (int j = tail_length - 1; j >= 0; j--) { // å…³é”®ä¿®æ”¹ï¼šåå‘éåŽ†j
            int offset = tail_length - 1 - j;
            int pos = reverse ? (current_head - offset) : (current_head + offset);
            if (pos >= 0 && pos < WS2812_LED_NUM) {
                // äº®åº¦è®¡ç®—ï¼šå¤´éƒ¨æœ€äº®ï¼Œå°¾éƒ¨è¡°å‡
                float brightness = pow(0.7f, j); // j=0æ—¶äº®åº¦æœ€é«˜
                brightness = pow(brightness, 2.2); // ä¼½é©¬æ ¡æ­£
                __brightnessAdjust(brightness, meteor_color);
                uint32_t grb = (rgb_color.G << 16) | (rgb_color.R << 8) | rgb_color.B;
                ws2812_Set_one_LED_Color(pos, grb);
                last_positions[offset] = pos; // è®°å½•ä½ç½®
            }
        }
        last_head = current_head;

        ws2812_Send_Data();
        osDelay(speed);
    }
    memset(last_positions, -1, sizeof(last_positions));
    last_head = -1;
}