
#ifndef __WS2812_H
#define __WS2812_H

#include "stm32h7xx_hal.h"
#include "spi.h"
#include "cmsis_os.h"

#define WS2812_LED_NUM 	15



#define 		SIG_1 		0XE0
#define 		SIG_0  		0X80


//       ÑÕÉ«     GRB
#define	C_Red		0x00FF00
#define	C_Green		0xFF0000
#define	C_Blue		0x0000FF
#define	C_Yellow   	0xFFFF00
#define	C_Purple	0x00FFFF
#define	C_Orange	0x7DFF00
#define	C_indigo	0xFF00FF
#define	C_white 	0xFFFFFF
#define	C_pink 	0x70DB93
#define	OFF 	    0x000000


enum Color
{
	Red,
	Green,
	Blue,
	Yellow,
	Purple,
	Orange,
	Indigo,
	White,
  Sexy,
};

typedef struct RGB_Color{
	unsigned char R;
	unsigned char G;
	unsigned char B;
	unsigned char l;
}RGB_Color;

typedef struct HSV_Color{
	float H;
	float S;
	float V;
}HSV_Color;

void ws2812_Init(void);
void ws2812_Send_Data(void);
void ws2812_AllShutOff(void);

void ws2812_Set_one_LED_Color(uint16_t LED_index ,uint32_t GRB_color);
void ws2812_Roll_on_Color_Ring(uint16_t interval_time);

void ws2812_All_LED_one_Color_breath(uint16_t interval_time, uint32_t GRB_color,uint16_t LED_index);
void ws281x_setPixelRGB(uint16_t n ,uint8_t red, uint8_t green, uint8_t blue);
void ws281x_ShutoffPixel(uint16_t n);
void horse_race_lamp(uint16_t interval_time);
void Running_water_lamp( uint8_t red ,uint8_t green ,uint8_t blue, uint16_t interval_time );
void srand_lamp(uint16_t interval_time);
void set_pixel_rgb(uint16_t n,uint8_t color);
void ws2812_AllOpen(uint8_t red ,uint8_t green ,uint8_t blue);

void ws2812_Toggle(uint16_t LED_index,uint32_t GRB_color,uint8_t flag);
void ws2812_police_group(uint8_t group,uint32_t GRB_color);
void ws2812_police();
/************************************************************
											Peivate Function
************************************************************/
void __brightnessAdjust(float percent, RGB_Color RGB);
void ws2812_DualFlow(uint32_t color, uint16_t speed);
void ws2812_SmoothMeteor(uint32_t color, uint16_t speed, uint8_t tail_length, uint8_t reverse) ;
void ws2812_BreathingWave(uint32_t base_color, uint16_t speed, uint8_t wave_length);
#endif

/************************************************************
														EOF
*************************************************************/

