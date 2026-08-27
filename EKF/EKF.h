#ifndef _EKF_H
#define _EKF_H
#include <stabilizer_types.h>
#include <string.h>
#include "cmsis_os2.h"
#include <string.h>
//const float ax_b=0.000098;//加速度计零偏，注意单位m/s
//const float ay_b=0.000098;
//const float az_b=0.000147;
//const float vx_b=0.00349065;//陀螺仪零偏 rad
//const float vy_b=0.00349065;
//const float vz_b=0.01745329;
//const float Gravity_Acc=9.8;//m/s
//unsigned long Time_IMU;
#define P_I 3.1415926535
typedef struct {
    double w, x, y, z;
} Quaternion;
typedef struct {
    float data[3][3];
} Matrix;
typedef struct {
    float data[6][6];
} Matrix_6_6;
typedef struct {
    float data[15][15];
} Matrix_15_15;
typedef struct {
    float data[6][15];
} Matrix_6_15;
typedef struct {
    float data[3][15];
} Matrix_3_15;
typedef struct {
    float data[15][6];
} Matrix_15_6;
typedef struct {
    float data[15][3];
} Matrix_15_3;
typedef struct {
    float data[6];
} RowVector_6;
typedef struct {
    float data[15];
} RowVector_15;
typedef struct {
    float data[3];
} RowVector;
//RowVector a_b;
//RowVector v_b;
//RowVector free_Acc;
typedef struct
{
	RowVector Gyro_Zero;
	RowVector Acc_Zero;
	RowVector Gyro_Nosie;
	RowVector Acc_Nosie;
	RowVector Eurl_Ins;
	Matrix Dcm;
	Quaternion Q;
	RowVector Speed_Ins;
	RowVector Position_Ins;
	RowVector Free_Acc_Ins;
	RowVector Wnen;
	RowVector Wnie;
	RowVector Wnin;
	RowVector fn;//比力
	double Wie;//地球自转角速度
	float r0;//半径
	float e;//曲率
	float Rm;
	float Rn;
	float g0;
	double Lati;
	double Longi;
	float alti;
	
}Ins_Parameter;
typedef struct
{
	RowVector_15 Xk;
	RowVector_15 Xkk_1;
	Matrix_15_15 Pk;
	Matrix_15_15 Pkk_1;
	Matrix_15_15 Qt;
	Matrix_15_15 Qk;
	Matrix_6_6 Rk;
	Matrix_6_15 Hk;
	Matrix_15_15 Ft;
	Matrix_15_15 Fk;
	Matrix_15_6 Kk;
	RowVector_6 Zk;
	
	//单一速度观测相关
	Matrix_3_15 Hk_;
	Matrix Rk_;
	Matrix_15_3 Kk_;
	RowVector Zk_;
	
	
	
}Kf_Parameter;
void getFusion_PS(IMUdata *Position,IMUdata *Speed);
extern unsigned long getRunTimeCounterValue(void);
extern osMessageQueueId_t EKF_IMU_QueHandle;
extern osMessageQueueId_t DVL_Speed_QueHandle;
extern osMessageQueueId_t DVL_Position_QueHandle;
extern osMessageQueueId_t EKF_Position_QueHandle;
extern osSemaphoreId_t DVL_Handle;
extern Ins_Parameter Ins;
extern Kf_Parameter Kf;
extern Axis3f Gyro_;
extern Axis3f Acc_;
extern float Updata_Time;
//RowVector matrix_vector_multiply(const Matrix* mat, const RowVector* vec);
//void Matrix_Attitude_Init(Matrix *Rotation_M,RowVector *Gyro_V,RowVector *Eurl_Current);
//RowVector Sub_vector(const RowVector* vec_1,const RowVector* vec_2);
//RowVector Attitude_Updata(void);
//void EKF_Init(RowVector *AttitudeValue,RowVector *SpeedValue,RowVector *Position);
//void Matrix_Speed_Init(Matrix *Rotation_M,RowVector *Acc_V,RowVector *Eurl_Current);












#endif