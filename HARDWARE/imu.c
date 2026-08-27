#include "imu.h"

// 数据标识定义
#define XDI_UtcTime        0x1010
#define XDI_EulerAngles    0x2030
#define XDI_Acceleration   0x4020
#define XDI_FreeAccel      0x4030
#define XDI_RateOfTurn     0x8020
#define XDI_StatusWord     0xE020


Axis3f   EulerRaw;
IMUdata   AccRaw;
Axis3f   FreeaccRaw;
IMUdata   GyroRaw;


uint8_t IMU[256];
XsensData_t  mti_data;



// 大端转小端辅助函数
static inline uint32_t be32_to_le(const uint8_t* data) {
    return ((uint32_t)data[0] << 24) |
        ((uint32_t)data[1] << 16) |
        ((uint32_t)data[2] << 8) |
        data[3];
}

static inline float be32_float_to_float(const uint8_t* data) {
    union {
        uint32_t u32;
        float f;
    } converter;

    converter.u32 = be32_to_le(data);
    return converter.f;
}

void parse_mti630_data(uint8_t* imubuffer, uint16_t Size,XsensData_t *mti_data) {
    uint8_t* ptr = imubuffer;
    uint16_t payload_length = ptr[3];  // LEN字段

 
    

    ptr += 4; // 跳过FA FF MID LEN

    while (ptr < imubuffer + payload_length + 4) { // +4包含校验字节前
        uint16_t data_id = (ptr[0] << 8) | ptr[1];
        uint8_t data_len = ptr[2];
        uint8_t* data = ptr + 3;

        switch (data_id) {
        case XDI_UtcTime:
            if (data_len == 12) {
				
//                mti_data->Utc.nanoseconds= be32_to_le(data);
//               mti_data->Utc.year = (data[4] << 8) | data[5];
//               mti_data->Utc.month= data[6];
//                mti_data->Utc.day= data[7];
//                mti_data->Utc.hour= data[8];
//                mti_data->Utc.minute= data[9];
//                mti_data->Utc.second = data[10];
//                mti_data->Utc.valid_flags = data[11];
            }
            break;

        case XDI_EulerAngles:
            if (data_len == 12) {
                mti_data->eurl.x = be32_float_to_float(data);
                mti_data->eurl.y = be32_float_to_float(data + 4);
                mti_data->eurl.z = be32_float_to_float(data + 8) ;
                if(mti_data->eurl.z  < 0  ){
				
				     mti_data->eurl.z += 360;
				}
            }
            break;

        case XDI_Acceleration:
            if (data_len == 12) {
                mti_data->acc.x  = be32_float_to_float(data);
                mti_data->acc.y = be32_float_to_float(data + 4);
                mti_data->acc.z = be32_float_to_float(data + 8);
            }
            break;

        case XDI_FreeAccel:
            if (data_len == 12) {
                mti_data->freeacc.x  = be32_float_to_float(data);
                mti_data->freeacc.y= be32_float_to_float(data + 4);
                mti_data->freeacc.z= be32_float_to_float(data + 8);
            }
            break;

        case XDI_RateOfTurn:
            if (data_len == 12) {
                mti_data->gyro.x = be32_float_to_float(data);
                mti_data->gyro.y = be32_float_to_float(data + 4);
                mti_data->gyro.z  = be32_float_to_float(data + 8);
            }
            break;

        case XDI_StatusWord:
            if (data_len == 4) {
                //mti_data->status = be32_to_le(data);
            }
            break;
        }

        ptr += 3 + data_len; // 移动到下一个数据包
    }
}

/* USER CODE END Includes */


void IMUReadTask(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  osStatus_t xReturn = osOK;
     
   osDelay(100);
   HAL_UARTEx_ReceiveToIdle_DMA(&huart4,(uint8_t *)IMU, sizeof(IMU));

  /* Infinite loop */
  for(;;)
  {

      xReturn =   osMessageQueueGet( myQueue01Handle, &EulerRaw, 0, osWaitForever); 
      osMessageQueueGet( IMU_accHandle, &AccRaw, 0, osWaitForever); 
	  osMessageQueueGet( IMU_freeaccHandle, &FreeaccRaw, 0, osWaitForever); 
	  osMessageQueueGet( IMU_gyroHandle, &GyroRaw, 0, osWaitForever); 
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, IMU, sizeof(IMU));

  }
  /* USER CODE END IMU_Task */
}

void getEurlRawData(Axis3f *Eurl)
{
	*Eurl = EulerRaw;
}

void getAccRawData(IMUdata *Acc)
{
	*Acc = AccRaw;
}

void getFreeaccRawData(Axis3f *FreeAcc)
{
	*FreeAcc = FreeaccRaw;
}

void getGyroRawData(IMUdata *Gyro)
{
	*Gyro = GyroRaw;
}


