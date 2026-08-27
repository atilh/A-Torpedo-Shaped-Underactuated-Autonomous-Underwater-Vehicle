//#include <EKF.h>
#include <GPS.h>
#include <imu.h>
#include <math.h>
#include <stdlib.h>
#include <Math_Ekf.h>
#include "DVL.h"
IMUdata P_Fision;
IMUdata S_Fision;
float Updata_Time=0.0025;
float Ti;
Ins_Parameter Ins;
Kf_Parameter Kf;
Axis3f Gyro;
Axis3f Acc;
Axis3f Speed_;
Axis3f Position_;
Axis3f Init_Elur;
Matrix Init_Cnb;//放在水里，别动一会更新，固定姿态
Matrix_15_15 M15;

void Ins_Init(void)//纬度变化注意
{

//	GPRMC_Data Position_;
//	getGPSRawData(&Position_);//单位测试一下什么时候可以开始
	getEurlRawData(&Init_Elur);//验证欧拉角的顺序,单位,默认的欧拉角是东北天，航向角方向差异，转换，什么时候开始有个信号，经纬度到时候写上去
	Ins.Lati=(35.0+47.0/60.0)*P_I/180.0;//注意修改//需要指定初始的经纬度，纬度经度高度,弧度单位
	Ins.Longi=(120.0+1.0/60.0)*P_I/180.0;
	Ins.alti=10;//高度需要解算看手机
	Ins.Acc_Zero.data[0]=0.01*9.7803*1E-06;//待商榷
	Ins.Acc_Zero.data[1]=0.01*9.7803*1E-06;
	Ins.Acc_Zero.data[2]=0.01*9.7803*1E-06;
	Ins.Gyro_Zero.data[0]=8.0*P_I/180.0/3600;//待商榷
	Ins.Gyro_Zero.data[1]=8.0*P_I/180.0/3600;
	Ins.Gyro_Zero.data[2]=8.0*P_I/180.0/3600;
	Ins.Acc_Nosie.data[0]=1;
	Ins.Acc_Nosie.data[1]=1;//待商榷
	Ins.Acc_Nosie.data[2]=1;
	Ins.Gyro_Nosie.data[0]=0.007*P_I/180.0/60;
	Ins.Gyro_Nosie.data[1]=0.007*P_I/180.0/60;
	Ins.Gyro_Nosie.data[2]=0.007*P_I/180.0/60;
	Ins.Eurl_Ins.data[0]=Init_Elur.y*P_I/180.0;//弧度
	Ins.Eurl_Ins.data[1]=Init_Elur.x*P_I/180.0;
	Ins.Eurl_Ins.data[2]=Init_Elur.z*P_I/180.0;
	EurlToDcm(&(Ins.Dcm),&(Ins.Eurl_Ins));
	Init_Cnb=Ins.Dcm;
	Ins.Q=EurlToQuaternion(&(Ins.Eurl_Ins));
	Ins.Speed_Ins.data[0]=0;//都是NED坐标系
	Ins.Speed_Ins.data[1]=0;
	Ins.Speed_Ins.data[2]=0;
	Ins.Position_Ins.data[0]=0;//需要指定初始的经纬度，纬度经度高度，位置从DVL读取
	Ins.Position_Ins.data[1]=0;
	Ins.Position_Ins.data[2]=0;
	Ins.Wie=7.292115E-5;
	Ins.r0=6378137.0;
	Ins.e=0.0818191908426;
	Ins.Rm=Ins.r0*(1-pow(Ins.e,2.0))/pow((1-pow(Ins.e,2.0)*pow(sin(Ins.Lati),2.0)),3.0/2.0);
	Ins.Rn=Ins.r0/pow((1-pow(Ins.e,2.0)*pow(sin(Ins.Lati),2.0)),0.5);
	Ins.g0=9.780325*(1+0.00530240*pow(sin(Ins.Lati),2.0)-0.00000582*pow(sin(2*Ins.Lati),2.0));
	Ins.Free_Acc_Ins.data[0]=0;
	Ins.Free_Acc_Ins.data[1]=0;
	Ins.Free_Acc_Ins.data[2]=Ins.g0;//正负号注意
	Identity_matrix_15_15(&M15);
}
void KF_Init(void)
{
	int i=0;
	RowVector Erro_Begin_Attitude;
	RowVector Erro_Begin_Speed;
	RowVector Erro_Begin_Position;
	RowVector rk_Speed;
	RowVector rk_Position;
	Erro_Begin_Attitude.data[0]=0.0175;
	Erro_Begin_Attitude.data[1]=0.0175;
	Erro_Begin_Attitude.data[2]=0.5;
	Erro_Begin_Speed.data[0]=0.1;
	Erro_Begin_Speed.data[1]=0.1;
	Erro_Begin_Speed.data[2]=0.1;
	Erro_Begin_Position.data[0]=3.135E-7;
	Erro_Begin_Position.data[1]=3.135E-7;
	Erro_Begin_Position.data[2]=2;
	rk_Speed.data[0]=0.3;
	rk_Speed.data[1]=0.3;
	rk_Speed.data[2]=0.8;
	rk_Position.data[0]=2.35;//可调
	rk_Position.data[1]=2.35;
	rk_Position.data[2]=1.5;
	
	//Pk初始化
	for(;i<15;i++)
	{
		switch(i/3)
		{
			case 0:Kf.Pk.data[i][i]=pow(Erro_Begin_Attitude.data[i],2.0);break;
			case 1:Kf.Pk.data[i][i]=pow(Erro_Begin_Speed.data[i-3],2.0);break;
			case 2:Kf.Pk.data[i][i]=pow(Erro_Begin_Position.data[i-6],2.0);break;
			case 3:Kf.Pk.data[i][i]=pow(Ins.Gyro_Zero.data[i-9],2.0);break;
			case 4:Kf.Pk.data[i][i]=pow(Ins.Acc_Zero.data[i-12],2.0);break;
		}
	}
	//Qt的初始化
	for(i=0;i<6;i++)
	{
		switch(i/3)
		{
			case 0:Kf.Qt.data[i][i]=pow(Ins.Gyro_Nosie.data[i],2.0);break;//精度问题
			case 1:Kf.Qt.data[i][i]=pow(Ins.Acc_Nosie.data[i-3],2.0);break;
		}
	}
	Number_multiply_Matrix_15_15(Updata_Time,&Kf.Qt);
	Kf.Qk=Kf.Qt;
	//Rk的初始化
		for(i=0;i<6;i++)
	{
		switch(i/3)
		{
			case 0:Kf.Rk.data[i][i]=pow(rk_Speed.data[i],2.0);break;//精度问题
			case 1:Kf.Rk.data[i][i]=pow(rk_Position.data[i-3],2.0);break;
		}
	}
	//Hk的初始化
	   for(i=0;i<6;i++)
	{
		switch(i/3)
		{
			case 0:Kf.Hk.data[i][i+3]=1.0;break;//精度问题
			case 1:Kf.Hk.data[i][i+3]=1.0;break;
		}
	}	
	//单一速度观测的参数初始化
	//Hk_的初始化
		for(i=0;i<3;i++)
	{
		switch(i/3)
		{
			case 0:Kf.Hk_.data[i][i+3]=1.0;break;
		}
	}
	//Rk_的初始化
		for(i=0;i<3;i++)
	{
		switch(i/3)
		{
			case 0:Kf.Rk_.data[i][i]=pow(rk_Speed.data[i],2.0);break;//精度问题
		}
	}	
	

}
void Ins_Update(Ins_Parameter *Ins,Axis3f *Gyro_,Axis3f *Acc_)
{
//	Updata_Time=0.025;//400HZ可调
	int i;
	RowVector Gyro_d;//角增量
	RowVector Acc_d;//速度增量
	RowVector Wnen;
	RowVector Wnie;
	RowVector Wnin;
	RowVector Wbin;
	RowVector Wbin_d;
	RowVector Speed_;
	RowVector V_Scull_b;
	RowVector V_Scull_n;
	RowVector temporary;
	RowVector V_Update;
	Matrix Cbn;
	Matrix Cnb;
	double Wie_;
	double Lati_;//注意需要初始化
	double Longi_;
	float alti_;
	float Rm;
	float Rn;
	Wie_=Ins->Wie;
	Lati_=Ins->Lati;
	Longi_=Ins->Longi;
	alti_=Ins->alti;
	Rm=Ins->Rm;
	Rn=Ins->Rn;
	Speed_=Ins->Speed_Ins;
	Cnb=Ins->Dcm;
	Gyro_d.data[0]=Updata_Time*(Gyro_->y);//-Ins->Gyro_Zero.data[0]//单位小心，这里应该不要减去零漂*P_I/180.0
	Gyro_d.data[1]=Updata_Time*(Gyro_->x);//-Ins->Gyro_Zero.data[1]
	Gyro_d.data[2]=Updata_Time*(Gyro_->z);//-Ins->Gyro_Zero.data[2]
	Acc_d.data[0]=Updata_Time*(Acc_->y);//-Ins->Acc_Zero.data[0]//单位小心，这里应该不要减去零漂
	Acc_d.data[1]=Updata_Time*(Acc_->x);//-Ins->Acc_Zero.data[1]
	Acc_d.data[2]=Updata_Time*(Acc_->z);//-Ins->Acc_Zero.data[2]
	Wnie.data[0]=Wie_*cos(Lati_);
	Wnie.data[1]=0;
	Wnie.data[2]=-Wie_*sin(Lati_);
	Wnen.data[0]=Speed_.data[1]/Rn;
	Wnen.data[1]=-Speed_.data[0]/Rm;
	Wnen.data[2]=-Speed_.data[1]*tan(Lati_)/Rn;
	Cbn=transpose(&Cnb);
	Wnin=Add_vector(&Wnie,&Wnen);
	Wbin=matrix_vector_multiply(&Cbn,&Wnin);
	Wbin_d=Number_multiply_vector(Updata_Time,&Wbin);
	Gyro_d=Sub_vector(&Gyro_d,&Wbin_d);//体坐标系
	//速度更新方程
	V_Scull_b=cross_product(&Gyro_d,&Acc_d);
	V_Scull_b=Number_multiply_vector(0.5,&V_Scull_b);
	V_Scull_b=Add_vector(&V_Scull_b,&Acc_d);
	V_Scull_n=matrix_vector_multiply(&Cnb,&V_Scull_b);
	temporary=Number_multiply_vector(2,&Wnie);
	temporary=Add_vector(&temporary,&Wnen);
	temporary=cross_product(&temporary,&Ins->Speed_Ins);
	temporary=Sub_vector(&Ins->Free_Acc_Ins,&temporary);
	temporary=Number_multiply_vector(Updata_Time,&temporary);
	temporary=Add_vector(&temporary,&V_Scull_n);
	V_Update=Add_vector(&temporary,&Ins->Speed_Ins);
	Ins->Speed_Ins=V_Update;//速度推算结果
	//位置更新方程
//	Lati_+=Updata_Time*V_Update.data[0]/(Ins->Rm+alti_);//精度有问题？？？弧度单位
//	Longi_+=Updata_Time*V_Update.data[1]/(Ins->Rn+alti_)/cos(Lati_);
//	alti_-=Updata_Time*V_Update.data[2];
//	Ins->alti=alti_;//位置更新结果
//	Ins->Lati=Lati_;
//	Ins->Longi=Longi_;
	Ins->Position_Ins.data[0] += Updata_Time * V_Update.data[0];
	Ins->Position_Ins.data[1] += Updata_Time * V_Update.data[1];
	Ins->Position_Ins.data[2] += Updata_Time * V_Update.data[2];
	//惯导参数更新
//	Ins->Rm=Ins->r0*(1-pow(Ins->e,2.0))/pow((1-pow(Ins->e,2.0)*pow(sin(Ins->Lati),2.0)),3.0/2.0);
//	Ins->Rn=Ins->r0/pow((1-pow(Ins->e,2.0)*pow(sin(Ins->Lati),2.0)),0.5);
//	Ins->g0=9.780325*(1+0.00530240*pow(sin(Ins->Lati),2.0)-0.00000582*pow(sin(2*Ins->Lati),2.0));
//	Ins->Free_Acc_Ins.data[2]=Ins->g0;//正负号注意
	//姿态四元数更新
	Ins->Q=Quaternion_Update(&Gyro_d,Ins->Q,-1);
	Ins->Eurl_Ins=QuaternionToEulerAngles(Ins->Q);
	EurlToDcm(&Ins->Dcm,&Ins->Eurl_Ins);
	//保存滤波相关参数
	Ins->Wnen=Wnen;
	Ins->Wnie=Wnie;
	Ins->Wnin=Wnin;
	Acc_d.data[0]/=Updata_Time;
	Acc_d.data[1]/=Updata_Time;
	Acc_d.data[2]/=Updata_Time;
	Ins->fn=matrix_vector_multiply(&Cnb,&Acc_d);//比力更新
}
void Prediction_Update(void)//预测的两个方程
{
//	RowVector Wnen=Ins.Wnen;
//	RowVector Wnie=Ins.Wnie;
//	RowVector Wnin=Ins.Wnin;
	//RowVector fn=Ins.fn;
	//double Wie=Ins.Wie;
	//float Rm=Ins->Rm;
	//float Rn=Ins->Rn;
	//double Lati_=Ins->Lati;
	//float alti_=Ins.alti;
	//float Ve=Ins.Speed_Ins.data[1];
	//float Vn=Ins.Speed_Ins.data[0];
	//姿态误差更新方程的相关矩阵
	Matrix M_aa;
	Matrix M_av;
	Matrix M_ap;
	M_aa=_Askew(&Ins.Wnin,-1);//有问题吗？？
	M_av.data[0][0]=0;
	M_av.data[0][1]=1.0/(Ins.Rn);
	M_av.data[0][2]=0;
	M_av.data[1][0]=-1.0/(Ins.Rm);
	M_av.data[1][1]=0;
	M_av.data[1][2]=0;
	M_av.data[2][0]=0;
	M_av.data[2][1]=-tan(Ins.Lati)/(Ins.Rn);//00
	M_av.data[2][2]=0;//0
	memset(&M_ap,0,sizeof(Matrix));
//	M_ap.data[1][0]=Vn/(Rm*Rm);
//	M_ap.data[1][1]=Wnie.data[2];//-wu
//	M_ap.data[1][2]=-Ve/(Rn*Rn);//-ve/rn2顺序变换都要看乘的变量和结果是什么
//	M_ap.data[0][0]=Wnie.data[2];
//	M_ap.data[0][1]=0;
//	M_ap.data[0][2]=-Ve/(Rn*Rn);
//	M_ap.data[2][0]=-Wnie.data[0]-Ve*pow(1/cos(Lati_),2)/Rn;
//	M_ap.data[2][1]=0;//-wn-vesecl2/rn
//	M_ap.data[2][2]=Ve*tan(Lati_)/(Rn*Rn);
	//速度误差更新方程的相关矩阵
	Matrix M_va=_Askew(&Ins.fn,1);
	Matrix M_vv;
	Matrix M_vp;
	Matrix M_p1;
	Matrix M_Middle;
	Matrix M_Middle_1;
	RowVector R_Middle;
	R_Middle=Number_multiply_vector(2,&Ins.Wnie);
	R_Middle=Add_vector(&R_Middle,&Ins.Wnen);
	M_Middle=_Askew(&Ins.Speed_Ins,1);
	M_vv=multiply_3_3(&M_Middle,&M_av);
	M_Middle_1=_Askew(&R_Middle,-1);
	M_vv=addMatrices_3_3(&M_Middle_1,&M_vv);
	memset(&M_p1,0,sizeof(Matrix));
	M_p1.data[0][0]=Ins.Wnie.data[2];
	M_p1.data[2][0]=-Ins.Wnie.data[0];
	M_Middle_1=addMatrices_3_3(&M_ap,&M_p1);
	M_vp=multiply_3_3(&M_Middle,&M_Middle_1);
	//位置误差更新的相关矩阵
	Matrix M_pv;
	Matrix M_pp;
	memset(&M_pv,0,sizeof(Matrix));
//	M_pv.data[0][0]=1/(Rm+alti_);
//	M_pv.data[1][1]=1/(cos(Lati_)*(Rn+alti_));
//	M_pv.data[2][2]=-1;
	M_pv.data[0][0] = 1;
	M_pv.data[1][1] = 1;
	M_pv.data[2][2] = 1;
	memset(&M_pp,0,sizeof(Matrix));
//	M_pp.data[0][2]=-Vn/pow((Rm+alti_),2);
//	M_pp.data[1][0]=Ve*tan(Lati_)/(cos(Lati_)*(Rn+alti_));
//	M_pp.data[1][2]=-Ve/(cos(Lati_)*pow((Rn+alti_),2));
	//更新状态转移矩阵并离散化
//	Kf.Ft.data
	//Matrix_15_15 M_15;
	//Matrix_15_15 ft_;
    State_Matrix_Updata(&M_aa,&M_av,&M_ap,&M_va,&M_vv,&M_vp,&M_pv,&M_pp);
//	M_15=Identity_matrix_15_15();
//	ft_=Kf.Ft;
	Number_multiply_Matrix_15_15(Updata_Time,&Kf.Ft);
	addMatrices_15_15(&M15,&Kf.Ft,&Kf.Fk);//更新完成还剩下四个方程+
	Kf.Xk=matrix_vector_multiply_15_15(&Kf.Fk,&Kf.Xk);//状态更新
	//Matrix_15_15 Fk_T;
	Matrix_15_15 Middle_,Middle_1;//修改了
	multiply_15_15(&Kf.Fk,&Kf.Pk,&Middle_);
    transpose_15_15(&Kf.Fk);
	multiply_15_15(&Middle_,&Kf.Fk,&Middle_1);
	addMatrices_15_15(&Middle_1,&Kf.Qk,&Kf.Pk);//先验协方差更新完成
}
void Correct_Update(DVLdata *Speed ,DVLdata *Position)//需要条件，别忘记更新误差量测Zk;
{
	//计算量测信息减去DVL的数据,DVL的数据需要处理，旋转矩阵，需要一个恒定的旋转矩阵
	Matrix Cnb;
	RowVector Speed_;
	RowVector Position_;
	Cnb=Init_Cnb;
	Speed_.data[0]=Speed->DVL.x;
	Speed_.data[1]=Speed->DVL.y;//顺序
	Speed_.data[2]=Speed->DVL.z;
	Position_.data[0]=Position->DVL.x;
	Position_.data[1]=Position->DVL.y;
	Position_.data[2]=Position->DVL.z;
	Speed_=matrix_vector_multiply(&Cnb,&Speed_);//速度是体坐标系
	Position_=matrix_vector_multiply(&Cnb,&Position_);//固定的坐标系与北东地坐标系的关系
	Kf.Zk.data[0] = -Speed_.data[0] + Ins.Speed_Ins.data[0];
	Kf.Zk.data[1] = -Speed_.data[1] + Ins.Speed_Ins.data[1];
	Kf.Zk.data[2] = -Speed_.data[2] + Ins.Speed_Ins.data[2];
	Kf.Zk.data[3] = -Position_.data[0] + Ins.Position_Ins.data[0];//单位是米
	Kf.Zk.data[4] = -Position_.data[1] + Ins.Position_Ins.data[1];
	Kf.Zk.data[5] = -Position_.data[2] + Ins.Position_Ins.data[2];
	//更新卡尔曼增益
	Matrix_15_6 M;
	Matrix_15_6 Hk_T;
	Matrix_6_6 M1;
	Hk_T=transpose_Matrix_6_15(&Kf.Hk);
	M=multiply_15_15_15_6(&Kf.Pk,&Hk_T);
	M1=multiply_6_15_15_6(&Kf.Hk,&M);
	M1=addMatrices_6_6(&M1,&Kf.Rk);
	M1=matrixInverse(&M1);
	Kf.Kk=multiply_15_6_6_6(&M,&M1);
	//更新状态的估计值
	//1.更新量测值，后面写，需要建立队列判断
	RowVector_6 V;
	RowVector_15 V1;
	V=matrix_vector_multiply_6_15(&Kf.Hk,&Kf.Xk);
	V=Sub_vector_6(&Kf.Zk,&V);
	V1=matrix_vector_multiply_15_6(&Kf.Kk,&V);
	Kf.Xk=Add_vector_15(&V1,&Kf.Xk);//误差估计更新
	//更新协方差
	Matrix_15_15 Ma_;
	//Matrix_15_15 Mb_;
	Ma_=multiply_15_6_6_15(&Kf.Kk,&Kf.Hk);
	//Mb_=Identity_matrix_15_15();//
	Ma_=SubMatrices_15_15(&M15,&Ma_);
	Kf.Pk=multiply_15_15_(&Ma_,&Kf.Pk);
	//修正惯导信息
//	Ins.Lati-=Kf.Xk.data[6];
//	Ins.Longi-=Kf.Xk.data[7];
//	Ins.alti-=Kf.Xk.data[8];
	Ins.Position_Ins.data[0] -= Kf.Xk.data[6];
	Ins.Position_Ins.data[1] -= Kf.Xk.data[7];
	Ins.Position_Ins.data[2] -= Kf.Xk.data[8];
	Ins.Speed_Ins.data[0]-=Kf.Xk.data[3];
	Ins.Speed_Ins.data[1]-=Kf.Xk.data[4];;
	Ins.Speed_Ins.data[2]-=Kf.Xk.data[5];;
	RowVector W;//姿态角误差
	W.data[0]=Kf.Xk.data[0];//NED
	W.data[1]=Kf.Xk.data[1];
	W.data[2]=Kf.Xk.data[2];
	Ins.Q=Quaternion_Update(&W,Ins.Q,1);
	Ins.Eurl_Ins=QuaternionToEulerAngles(Ins.Q);
	EurlToDcm(&Ins.Dcm,&Ins.Eurl_Ins);
	//归零误差项
	for(int i=0;i<9;i++)
	{
		Kf.Xk.data[i]=0;
	}
}
void Correct_Update_single_Spe(DVLdata *Speed)
{
	Matrix Cnb;
	RowVector Speed_;
	Cnb=Ins.Dcm;
	Speed_.data[0]=Speed->DVL.x;
	Speed_.data[1]=Speed->DVL.y;//顺序
	Speed_.data[2]=Speed->DVL.z;
	Speed_=matrix_vector_multiply(&Cnb,&Speed_);//速度是体坐标系
	Kf.Zk_.data[0] = -Speed_.data[0] + Ins.Speed_Ins.data[0];
	Kf.Zk_.data[1] = -Speed_.data[1] + Ins.Speed_Ins.data[1];
	Kf.Zk_.data[2] = -Speed_.data[2] + Ins.Speed_Ins.data[2];
	//计算卡尔曼增益
	Matrix_15_3 M;
	Matrix_15_3 Hk_T;
	Matrix M1;
	Hk_T=transpose_Matrix_3_15(&Kf.Hk_);
	M=multiply_15_15_15_3(&Kf.Pk,&Hk_T);
	M1=multiply_3_15_15_3(&Kf.Hk_,&M);
	M1=addMatrices_3_3(&M1,&Kf.Rk_);
	M1=matrixInverse_3_3(&M1);
	Kf.Kk_=multiply_15_3_3_3(&M,&M1);	
	//后验估计更新
	RowVector V;
	RowVector_15 V1;
	V=matrix_vector_multiply_3_15(&Kf.Hk_,&Kf.Xk);
	V=Sub_vector(&Kf.Zk_,&V);
	V1=matrix_vector_multiply_15_3(&Kf.Kk_,&V);
	Kf.Xk=Add_vector_15(&V1,&Kf.Xk);
	//更新误差协方差
	Matrix_15_15 Ma_;
	Ma_=multiply_15_3_3_15(&Kf.Kk_,&Kf.Hk_);
	Ma_=SubMatrices_15_15(&M15,&Ma_);
	Kf.Pk=multiply_15_15_(&Ma_,&Kf.Pk);
	//修正状态
	Ins.Position_Ins.data[0] -= Kf.Xk.data[6];
	Ins.Position_Ins.data[1] -= Kf.Xk.data[7];
	Ins.Position_Ins.data[2] -= Kf.Xk.data[8];
	Ins.Speed_Ins.data[0]-=Kf.Xk.data[3];
	Ins.Speed_Ins.data[1]-=Kf.Xk.data[4];;
	Ins.Speed_Ins.data[2]-=Kf.Xk.data[5];;
	RowVector W;//姿态角误差
	W.data[0]=Kf.Xk.data[0];//NED
	W.data[1]=Kf.Xk.data[1];
	W.data[2]=Kf.Xk.data[2];
	Ins.Q=Quaternion_Update(&W,Ins.Q,1);
	Ins.Eurl_Ins=QuaternionToEulerAngles(Ins.Q);
	EurlToDcm(&Ins.Dcm,&Ins.Eurl_Ins);
	//归零误差项
	for(int i=0;i<9;i++)
	{
		Kf.Xk.data[i]=0;
	}
}
void EKF_UpdataTask(void *argument)
{
	
	  /* USER CODE BEGIN usblinktxTask */
	osStatus_t Status_Now=0;
	uint8_t count_=0;
	osDelay(2000);
	Ins_Init();
	KF_Init();
	
	unsigned long Tim;
    Tim=getRunTimeCounterValue();
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet( EKF_IMU_QueHandle, &Acc, 0, osWaitForever);//处理一下加速度数据
	  osMessageQueueGet( EKF_IMU_QueHandle, &Gyro, 0, 0);
	  Ins_Update(&Ins,&Gyro,&Acc);
	  Prediction_Update();//此函数有问题,检查一下对吗，另外的函数重复项检测，提高运行效率
	  Status_Now=osSemaphoreAcquire(DVL_Handle,0);
	  if(Status_Now==osOK)
	  { 
//		 Ti=((float)(getRunTimeCounterValue()-Tim))*0.05/1000.0;//测周期
//		 Tim=getRunTimeCounterValue();
		  DVLdata vel;
		  getDVLvelocityData(&vel);
		  Correct_Update_single_Spe(&vel);
	  }
	  IMUdata Position__,Speed__;
	  Position__.timestamp=getRunTimeCounterValue();
	  Position__.IMU.x=Ins.Position_Ins.data[0];
	  Position__.IMU.y=Ins.Position_Ins.data[1];
	  Position__.IMU.z=Ins.Position_Ins.data[2];
	  Speed__.timestamp=Position__.timestamp;
	  Speed__.IMU.x=Ins.Speed_Ins.data[0];
	  Speed__.IMU.y=Ins.Speed_Ins.data[1];
	  Speed__.IMU.z=Ins.Speed_Ins.data[2];
	  osMessageQueuePut(EKF_Position_QueHandle,&Position__,0,0);
	  osMessageQueuePut(EKF_Position_QueHandle,&Speed__,0,0);
//	  if(Status_Now==osOK&&osSemaphoreGetCount(DVL_Handle)==1)
//	  {
//		  osSemaphoreAcquire(DVL_Handle,0);
//		  osMessageQueueGet(DVL_Speed_QueHandle,&Speed_,0,0);
//		  osMessageQueueGet(DVL_Position_QueHandle,&Position_,0,0);
//		  Correct_Update(&Speed_ ,&Position_);
//	  }
	  
//     Status_Now=osSemaphoreAcquire(EKF_IMUHandle,osWaitForever);
//	  count_=osSemaphoreGetCount(DVLRxcpltHandle);
//	  if(Status_Now==osOK)
//	  {

//	  }
	  //osDelay(1);
  }
  /* USER CODE END usblinktxTask */
}
void EKF_FusionTask(void *argument)
{
	  for(;;)
  {
	  osMessageQueueGet( EKF_Position_QueHandle, &P_Fision, 0, osWaitForever);
	  osMessageQueueGet( EKF_Position_QueHandle, &S_Fision, 0, 0);
	  
  }
}
void getFusion_PS(IMUdata *Position,IMUdata *Speed)
{
	*Position=P_Fision;
	*Speed=S_Fision;
}









