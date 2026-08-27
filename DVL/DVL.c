#include "DVL.h"
#include "imu.h"
#include "stdio.h"
#include "stabilizer_types.h"
uint8_t DVL[2048]={0};

DVL_wrp_data wrp;
DVL_wrz_data  wrz;
float Time_;
unsigned long Time_IMU;
extern unsigned long getRunTimeCounterValue(void);
//DVL_wrp_pos_data wrp_data_save;
//DVL_wrz_velocity_data  wrz_data_save;

Axis3f   dvl_velocity;
Axis3f   dvl_pos;
Axis3f   dvl_eurl;
float   dvl_attitude;
uint8_t posreset[] = {"wcr*e2\r\n"};
uint8_t flag=0;
//uint8_t  posreset1[]= {'w','c','r','*','e','2',"\r","\n"};

uint8_t gyroreset[] = {"wcg*89\r\n"};

static const uint8_t lookup_table[256] = {
    0x00U,0x07U,0x0EU,0x09U,0x1CU,0x1BU,0x12U,0x15U,
    0x38U,0x3FU,0x36U,0x31U,0x24U,0x23U,0x2AU,0x2DU,
    0x70U,0x77U,0x7EU,0x79U,0x6CU,0x6BU,0x62U,0x65U,
    0x48U,0x4FU,0x46U,0x41U,0x54U,0x53U,0x5AU,0x5DU,
    0xE0U,0xE7U,0xEEU,0xE9U,0xFCU,0xFBU,0xF2U,0xF5U,
    0xD8U,0xDFU,0xD6U,0xD1U,0xC4U,0xC3U,0xCAU,0xCDU,
    0x90U,0x97U,0x9EU,0x99U,0x8CU,0x8BU,0x82U,0x85U,
    0xA8U,0xAFU,0xA6U,0xA1U,0xB4U,0xB3U,0xBAU,0xBDU,
    0xC7U,0xC0U,0xC9U,0xCEU,0xDBU,0xDCU,0xD5U,0xD2U,
    0xFFU,0xF8U,0xF1U,0xF6U,0xE3U,0xE4U,0xEDU,0xEAU,
    0xB7U,0xB0U,0xB9U,0xBEU,0xABU,0xACU,0xA5U,0xA2U,
    0x8FU,0x88U,0x81U,0x86U,0x93U,0x94U,0x9DU,0x9AU,
    0x27U,0x20U,0x29U,0x2EU,0x3BU,0x3CU,0x35U,0x32U,
    0x1FU,0x18U,0x11U,0x16U,0x03U,0x04U,0x0DU,0x0AU,
    0x57U,0x50U,0x59U,0x5EU,0x4BU,0x4CU,0x45U,0x42U,
    0x6FU,0x68U,0x61U,0x66U,0x73U,0x74U,0x7DU,0x7AU,
    0x89U,0x8EU,0x87U,0x80U,0x95U,0x92U,0x9BU,0x9CU,
    0xB1U,0xB6U,0xBFU,0xB8U,0xADU,0xAAU,0xA3U,0xA4U,
    0xF9U,0xFEU,0xF7U,0xF0U,0xE5U,0xE2U,0xEBU,0xECU,
    0xC1U,0xC6U,0xCFU,0xC8U,0xDDU,0xDAU,0xD3U,0xD4U,
    0x69U,0x6EU,0x67U,0x60U,0x75U,0x72U,0x7BU,0x7CU,
    0x51U,0x56U,0x5FU,0x58U,0x4DU,0x4AU,0x43U,0x44U,
    0x19U,0x1EU,0x17U,0x10U,0x05U,0x02U,0x0BU,0x0CU,
    0x21U,0x26U,0x2FU,0x28U,0x3DU,0x3AU,0x33U,0x34U,
    0x4EU,0x49U,0x40U,0x47U,0x52U,0x55U,0x5CU,0x5BU,
    0x76U,0x71U,0x78U,0x7FU,0x6AU,0x6DU,0x64U,0x63U,
    0x3EU,0x39U,0x30U,0x37U,0x22U,0x25U,0x2CU,0x2BU,
    0x06U,0x01U,0x08U,0x0FU,0x1AU,0x1DU,0x14U,0x13U,
    0xAEU,0xA9U,0xA0U,0xA7U,0xB2U,0xB5U,0xBCU,0xBBU,
    0x96U,0x91U,0x98U,0x9FU,0x8AU,0x8DU,0x84U,0x83U,
    0xDEU,0xD9U,0xD0U,0xD7U,0xC2U,0xC5U,0xCCU,0xCBU,
    0xE6U,0xE1U,0xE8U,0xEFU,0xFAU,0xFDU,0xF4U,0xF3U,
};

uint8_t crc8(uint8_t* message, int message_length) {
    uint8_t checksum = 0;
    while (message_length > 0) {
        checksum = lookup_table[*message ^ checksum];
        message++;
        message_length--;
    }
    return checksum;
}


// 解析后的数据结构体
typedef struct {
    float vx, vy, vz;
    char valid;
    float altitude, fom;
    char covariance[64];
    uint16_t time_of_validity, time_of_transmission;
    float time;
    uint8_t status;
} VelocityReport;

typedef struct {
    double time_stamp;
    float x, y, z, pos_std;
    float roll, pitch, yaw;
    uint8_t status;
} DeadReckoningReport;

// 辅助函数：十六进制字符转数值
static uint8_t hex_char_to_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    return 0xFF;
}

// 校验码转换
uint8_t hex_to_byte(const char* hex) {
    uint8_t high = hex_char_to_val(hex[0]);
    uint8_t low = hex_char_to_val(hex[1]);
    if (high == 0xFF || low == 0xFF) return 0xFF;
    return (high << 4) | low;
}

// 分割字段函数
int split_fields(char* str, char** fields, int max_fields) {
    int count = 0;
    fields[count++] = str;
    for (char* p = str; *p && count < max_fields; p++) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

// 处理单行数据
void process_line(const char* line, size_t len) {
    char buffer[256];
    if (len >= sizeof(buffer) - 1) return;
    memcpy(buffer, line, len);
    buffer[len] = '\0';

    char* asterisk = strrchr(buffer, '*');
    if (!asterisk || (buffer + len - asterisk) != 3) return;

    *asterisk = '\0';
    char* checksum_str = asterisk + 1;
    uint8_t checksum = hex_to_byte(checksum_str);
    if (checksum == 0xFF) return;

    if (crc8((uint8_t*)buffer, strlen(buffer)) != checksum) return;

    // 分割字段
    char* fields[20];
    int num_fields = split_fields(buffer, fields, 20);

    // 解析 Velocity Report
    if (num_fields >= 12 && strcmp(fields[0], "wrz") == 0) {
//        VelocityReport report;
//        report.vx = atof(fields[1]);  
//        report.vy = atof(fields[2]);
//        report.vz = atof(fields[3]);
//        report.valid = fields[4][0];
//        report.altitude = atof(fields[5]);
//        report.fom = atof(fields[6]);
//        strncpy(report.covariance, fields[7], sizeof(report.covariance) - 1);
//        report.time_of_validity = atoi(fields[8]);
//        report.time_of_transmission = atoi(fields[9]);
//        report.time = atof(fields[10]);
//        report.status = atoi(fields[11]);
 //printf("WRZ VX:%.2f VY:%.2f  VZ:%.2f H:%.2f\r\n", report.vx,report.vy,report.vz, report.altitude);		
//		Axis3f   velocity;
		DVLdata velocity;
		float attitude;

		velocity.DVL.x = atof(fields[1]);  
        velocity.DVL.y = atof(fields[2]);
        velocity.DVL.z = atof(fields[3]);
		fields[8]+=9;
		fields[9]+=9;
		velocity.time_of_validity=atoi(fields[8]);
		velocity.time_of_transmission=atoi(fields[9]);
		velocity.timestamp=getRunTimeCounterValue();
		attitude    = atof(fields[5]);
		
//		osSemaphoreRelease( DVL_Handle );
//		osMessageQueuePut( DVL_Speed_QueHandle, &velocity, 0,  0 );
		osMessageQueuePut( DVL_wrz_velocityHandle, &velocity, 0,  0 ); 
        osMessageQueuePut( DVL_attitudeHandle, &attitude, 0,  0 ); 

		

    }

    // 解析 Dead Reckoning Report
    else if (num_fields >= 10 && strcmp(fields[0], "wrp") == 0) {
//        DeadReckoningReport report;
//        report.time_stamp = atof(fields[1]);
//        report.x = atof(fields[2]);
//        report.y = atof(fields[3]);
//        report.z = atof(fields[4]);
//        report.pos_std = atof(fields[5]);
//        report.roll = atof(fields[6]);
//        report.pitch = atof(fields[7]);
//        report.yaw = atof(fields[8]);
//        report.status = atoi(fields[9]);
// printf("WRP X:%.2f Y:%.2f  Z:%.2f  r:%.2f p:%.2f y:%.2f\r\n",report.x,report.y,report.z,report.roll,report.pitch,report.yaw);		
          DVLdata pos;
		  Axis3f eurl;

          pos.DVL.x = atof(fields[2]);
          pos.DVL.y = atof(fields[3]);
          pos.DVL.z = atof(fields[4]);
		  pos.timestamp=getRunTimeCounterValue();
		  eurl.x  = atof(fields[6]);
          eurl.y  = atof(fields[7]);
          eurl.z  = atof(fields[8]);
//
//		dvl_pos->x  = atof(fields[2]);
//		dvl_pos->y  = atof(fields[3]);
//		dvl_pos->z  = atof(fields[4]);
     
//          printf("WRP X:%.2f Y:%.2f  Z:%.2f \r\n",wrp->pos.x,wrp->pos.y,wrp->pos.z);//wrp->eurl.x,wrp->eurl.y,wrp->eurl.z);
         
//		 osSemaphoreRelease( DVL_Handle );
//		 osMessageQueuePut(DVL_Position_QueHandle, &pos, 0,  0 );
         osMessageQueuePut(DVL_wrp_posHandle, &pos, 0,  0 );
//		 if(xReturn_wrp_p==osOK){
//		printf("wrpPOK!");
//		}else if(xReturn_wrp_p!=osOK){
//		printf("wrpPNO!");
//		}
		
         osMessageQueuePut(DVL_eurlHandle, &eurl, 0,  0 ); 
//		 if(xReturn_wrp_e==osOK){
//		printf("wrpEOK!");
//		}else if(xReturn_wrp_e!=osOK){
//		printf("wrpENO!");
//		}

    }
}

// 从DMA缓冲区调用
void process_dma_buffer(uint8_t* buf, size_t len) {
    char* start = (char*)buf;
    char* end = start + len;
    char* line_start = start;

    for (char* p = start; p < end; ++p) {
        if (*p == '\r' || *p == '\n') {
            size_t line_len = p - line_start;
            if (line_len > 0) process_line(line_start, line_len);
            while (p < end && (*p == '\r' || *p == '\n')) ++p;
            line_start = p;
        }
    }

    if (line_start < end) {
        process_line(line_start, end - line_start);
    }
}

void DvlReadTask(void *argument)
{
  /* USER CODE BEGIN DvlReadTask */
  osStatus_t xReturn_wrp_p = osOK;	
  osStatus_t xReturn_wrz_v = osOK;	
	osStatus_t xReturn_wrp_e = osOK;	
  osStatus_t xReturn_wrz_a= osOK;	
	
   HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *)DVL, sizeof(DVL));
	
	Time_IMU=getRunTimeCounterValue();
  /* Infinite loop */
  for(;;)
  {
	  
	  osSemaphoreAcquire(DVLRxcpltHandle,osWaitForever); 
//	  if(count_=osSemaphoreGetCount(DVLRxcpltHandle);
       xReturn_wrp_p =  osMessageQueueGet( DVL_wrp_posHandle, &wrp.pos, 0, 0);  
	   xReturn_wrz_v =  osMessageQueueGet( DVL_wrz_velocityHandle, &wrz.velocity, 0, 0);
	   xReturn_wrp_e =  osMessageQueueGet( DVL_eurlHandle, &wrp.eurl, 0, 0);  
	   xReturn_wrz_a  = osMessageQueueGet( DVL_attitudeHandle, &wrz.attitude, 0, 0);  
	  if(xReturn_wrz_v==osOK)
	  {
		 osSemaphoreRelease(DVL_Handle);
	  }
//	printf("WRP X:%.2f Y:%.2f  Z:%.2f \r\n",wrp.pos.x,wrp.pos.y,wrp.pos.z);//wrp->eurl.x,wrp->eurl.y,wrp->eurl.z);
	   //if(xReturn_wrp_p==osOK){//测频率拆下DVL测频率&&xReturn_wrz_v == osOK
//         printf("wrpreadOK!\r\n");
//		 printf("WRP X:%.2f Y:%.2f  Z:%.2f \r\n",wrp.pos.x,wrp.pos.y,wrp.pos.z);//wrp->eurl.x,wrp->eurl.y,wrp->eurl.z);
//		 printf("WRZ X:%.2f Y:%.2f  Z:%.2f \r\n",dvl_velocity.x,dvl_velocity.y,dvl_velocity.z);

//		    *dvl_pos = wrp_data_save->pos;
//		    *dvl_eurl = wrp_data_save->eurl;
	  //}
//	   else if(xReturn_wrp_p != osOK){
//		  printf("wrpreadNO!\r\n");
////	        *dvl_velocity = wrz_data_save->velocity;
////		    *dvl_attitude = wrz_data_save->altitude;
//	  }
//	  
//	  if(xReturn_wrz_v == osOK){
////	        *dvl_velocity = wrz_data_save->velocity;
////		    *dvl_attitude = wrz_data_save->altitude;
//		   printf("WRZ X:%.2f Y:%.2f  Z:%.2f \r\n",dvl_velocity.x,dvl_velocity.y,dvl_velocity.z);//wrp->eurl.x,wrp->eurl.y,wrp->eurl.z);
//	  }

	  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *)DVL, sizeof(DVL));

  }
  /* USER CODE END DvlReadTask */
}

void getDVLvelocityData(DVLdata* get){
        *get =  wrz.velocity;
}

void getDVLattitudeData(float* get){
        *get =  wrz.attitude;
}

void getDVLposData(DVLdata* get){
        *get =  wrp.pos;
}

void getDVLeurlData(Axis3f* get){
        *get =  wrp.eurl;
}
void Reset_Dr(void)//测试
{
	uint8_t Command_Re[5]="wcr\n";
	HAL_UART_Transmit(&huart1,(uint8_t *)Command_Re, 4,100);
}
