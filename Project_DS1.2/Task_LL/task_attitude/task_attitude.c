#include "driver_mpu6050.h"
#include "kalman_filter.h"
#include "task_attitude.h"
#include "driver_motor.h"
#include "driver_hanging.h"
#include "inv_mpu.h"
#include <math.h>	


#define TOP_ANGLE 180    //最大角 用于位置归一化


//---------kalman variable--------//
// x = [theta,w_theta] pitch
kalman_filter_init_t Attitude_pitch_kalman_filter_para = {
	.xhat_data = {0, 0},
  //预测量相关
	.P_data = {2, 0, 0, 2},//协方差矩阵
  .A_data = {1, 10, 0, 1},//状态转移矩阵 第二项为dt
  .H_data = {1, 0, 0, 1},//传感器矩阵
  .Q_data = {1, 0, 0, 1},//高斯噪声矩阵
	//测量值相关
  .R_data = {15, 0, 0, 150}//不确定方差矩阵
};
kalman_filter_t Attitude_pitch_kalman_filter={0};
// x = [fai,w_fai] roll
kalman_filter_init_t Attitude_roll_kalman_filter_para = {
	.xhat_data = {0, 0},
  //预测量相关
	.P_data = {2, 0, 0, 2},//协方差矩阵
  .A_data = {1, 0.05, 0, 1},//状态转移矩阵 第二项为dt
  .H_data = {1, 0, 0, 1},//传感器矩阵
  .Q_data = {1, 0, 0, 1},//高斯噪声矩阵
	//测量值相关
  .R_data = {100, 0, 0, 12000}//不确定方差矩阵
};
kalman_filter_t Attitude_roll_kalman_filter={0};
//---------------------------------//

//-----significant variable--------//
extern MPU6050DATATYPE Mpu6050_Data; 
MPU6050_ANGLETYPE MPU6050_Angle; 
extern MotorStuct Motor_1,Motor_2;
extern PitchPIDStuct Pitch_Pid;
extern RollPIDStruct Roll_Pid;
extern DuojiStruct Duo[4]; 
float accel_pitch_theta,	//加速度pitch角度解算
			accel_roll_fai;			//加速度roll角度解算
float gyro_pitch_w,				//陀螺仪pitch角速度解算
			gyro_roll_w,        //陀螺仪roll角速度解算
			gyro_yaw_w;        //陀螺仪yaw角速度解算
uint8_t attitude_init_flag = 0;   //初始化标志位

float yaw_dt = 0.009;//yaw增角



float offset_pitch = 2.3,//偏角
			offset_roll = 2;
//---------------------------------//

//---jscope observe variable------//
int obse_pitch_theta,    //jscope pitch角度 
		obse_roll_fai;       //jscope roll角度
int obse_kal_pitch_theta,    //jscope pitch角度 
		obse_kal_roll_fai;       //jscope roll角度
//---------------------------------//

//----------------------------------姿态解算相关-------------------------------------//
void Task_attitude_sol(void)           //姿态解算函数
{
	if(!attitude_init_flag)
	{
		Attitude_kalman_init();
		attitude_init_flag = 1;
	}
	
	//MPU6050_DATA_Read();
	
	//mpu_dmp_get_data(&MPU6050_Angle.pitch,&MPU6050_Angle.roll,&MPU6050_Angle.yaw);
	
	accel_attitude_aol();              //角速度姿态解算
	gyro_attitude_aol();               //陀螺仪角速度解算
	
	Attitude_kalman_pitch_dispose();   //pitch卡尔曼滤波器数据处理
	Attitude_kalman_roll_dispose();    //roll卡尔曼滤波器数据处理
	
	float add_yaw;
	add_yaw = (fabs(gyro_yaw_w * yaw_dt) < 0.1) ? 0 : gyro_yaw_w * yaw_dt; 
	MPU6050_Angle.yaw += add_yaw; //yaw低通滤波器数据处理
	
	
	zero_sol();                        //零偏处理
	WM_locaupdate();
	Attitude_observe_jscope();
}

void accel_attitude_aol(void)          //角速度姿态解算
/*
		θ = arctan(-Ax / sqrt(Ay^2 + Az^2)) //pitch
		Ф = arctan(Ay / sqrt(Ax^2 + Az^2)) 	//roll
*/
{
	accel_pitch_theta = atan( - Mpu6050_Data.Accel_X / sqrt( pow(Mpu6050_Data.Accel_Y,2) + pow(Mpu6050_Data.Accel_Z,2)));
	accel_roll_fai = atan( Mpu6050_Data.Accel_Y / sqrt( pow(Mpu6050_Data.Accel_X,2) + pow(Mpu6050_Data.Accel_Z,2)));
}

void gyro_attitude_aol(void)           //陀螺仪角速度解算
/*
	trans = [ 1	, sin(fai)*tan(theta) , cos(fai)*tan(theta)
						0 ,       cos(fai)      ,       -sin(fai)
            0 , sin*fai)*sec(theta) , cos(phi)*sec(theta)]
  [ w_roll , w_pitch , w_yaw ] = trans*[ gx , gy , gz ]'
*/
{
	gyro_pitch_w = cos(accel_roll_fai) * Mpu6050_Data.Gyro_Y
									+(-sin(accel_roll_fai)) * Mpu6050_Data.Gyro_Z;
	gyro_roll_w = Mpu6050_Data.Gyro_X 
								+ sin(accel_roll_fai)*tan(accel_pitch_theta) * Mpu6050_Data.Gyro_Y 
								+ cos(accel_roll_fai)*tan(accel_pitch_theta) * Mpu6050_Data.Gyro_Z;
	gyro_yaw_w = sin(accel_roll_fai) / cos(accel_pitch_theta) * Mpu6050_Data.Gyro_Y
							 + cos(accel_roll_fai) / cos(accel_pitch_theta) * Mpu6050_Data.Gyro_Z;
}

void zero_sol(void)                   //零偏处理
{
	MPU6050_Angle.pitch += offset_pitch;
	MPU6050_Angle.roll += offset_roll;
}

float pitch_lowK = 0.035,
			turn_lowK = 0.01,
			roll_lowK = 0.01;
float last_pitchlocation,
			last_turnlocation,
			last_rollloacation;
void WM_locaupdate(void)           //驱动位置更新
{
	//电机 - pitch   归一化
	Pitch_Pid.Location.Location =  pitch_lowK * MPU6050_Angle.pitch / TOP_ANGLE + (1-pitch_lowK) * last_pitchlocation;
	Pitch_Pid.Turn.Location = MPU6050_Angle.yaw /(TOP_ANGLE*2.0f);
	Roll_Pid.Location.Location = roll_lowK * MPU6050_Angle.roll / TOP_ANGLE + (1-roll_lowK) * last_rollloacation;
	
	last_pitchlocation = Pitch_Pid.Location.Location;
	last_turnlocation = Pitch_Pid.Turn.Location;
	last_rollloacation = Roll_Pid.Location.Location;
}

//-----------------------------------卡尔曼相关-------------------------------------//
void Attitude_kalman_init(void)            //卡尔曼参数初始化
{
	mat_init(&Attitude_pitch_kalman_filter.Q,2,2, Attitude_pitch_kalman_filter_para.Q_data);
	mat_init(&Attitude_pitch_kalman_filter.R,2,2, Attitude_pitch_kalman_filter_para.R_data);
	mat_init(&Attitude_roll_kalman_filter.Q,2,2, Attitude_roll_kalman_filter_para.Q_data);
	mat_init(&Attitude_roll_kalman_filter.R,2,2, Attitude_roll_kalman_filter_para.R_data);
}


int Kalinit_flag_pitch=0;//用以消除卡尔曼在初值阶跃时的预测偏移
void Attitude_kalman_pitch_dispose(void)    //pitch卡尔曼滤波器数据处理
{
	//x = [theta,w_theta]
	if(Kalinit_flag_pitch == 0)
	{		
		Attitude_pitch_kalman_filter_para.xhat_data[0] = accel_pitch_theta;
		Attitude_pitch_kalman_filter_para.xhat_data[1] = gyro_pitch_w;
		kalman_filter_init(&Attitude_pitch_kalman_filter, &Attitude_pitch_kalman_filter_para);
	}
	if(Kalinit_flag_pitch < 50) Kalinit_flag_pitch ++; //在迭代50次后即0.05s后再使用滤波后数据
	float *Attitude_pitch_result = kalman_filter_calc(&Attitude_pitch_kalman_filter, accel_pitch_theta, gyro_pitch_w);
	if(Kalinit_flag_pitch < 50)
	{
		MPU6050_Angle.pitch = accel_pitch_theta / PI * 180 ;       	//转化为角度
	}
	else
	{
		MPU6050_Angle.pitch	=	Attitude_pitch_result[0] / PI * 180 ;  //转化为角度
	}
}
int Kalinit_flag_roll=0;//用以消除卡尔曼在初值阶跃时的预测偏移
void Attitude_kalman_roll_dispose(void)   //pitch卡尔曼滤波器数据处理
{
	//x = [fai,w_fai]
	if(Kalinit_flag_roll == 0)
	{		
		Attitude_roll_kalman_filter_para.xhat_data[0] = accel_roll_fai;
		Attitude_roll_kalman_filter_para.xhat_data[1] = gyro_roll_w;
		kalman_filter_init(&Attitude_roll_kalman_filter, &Attitude_roll_kalman_filter_para);
	}
	if(Kalinit_flag_roll < 50) Kalinit_flag_roll ++; //在迭代50次后即0.05s后再使用滤波后数据
	float *Attitude_roll_result = kalman_filter_calc(&Attitude_roll_kalman_filter, accel_roll_fai, gyro_roll_w);
	if(Kalinit_flag_roll < 50)
	{
		MPU6050_Angle.roll = accel_roll_fai / PI * 180;         	  //转化为角度
	}
	else
	{ 
		MPU6050_Angle.roll	=	Attitude_roll_result[0] / PI * 180;   //转化为角度
	}
}

//-----------------------------------观测相关-------------------------------------//
extern float AdcGetVal[4]; 
void Attitude_observe_jscope()              	//观测数据处理
{
	obse_pitch_theta = (int)(accel_pitch_theta / PI * 180 + 180)*50;
	obse_roll_fai = (int)(accel_roll_fai / PI * 180 + 180)*50;
	obse_kal_pitch_theta = (int)(MPU6050_Angle.pitch / PI * 180 + 180)*50;
	obse_kal_roll_fai = (int)(MPU6050_Angle.roll / PI * 180 + 180)*50;
//  AdcGetVal[0] = MPU6050_Angle.pitch;
//  AdcGetVal[1] = accel_pitch_theta / PI * 180;
//	AdcGetVal[2] = Pitch_Pid.Location.Location * TOP_ANGLE;
	
//	AdcGetVal[0] = MPU6050_Angle.yaw;
//	AdcGetVal[2] = Pitch_Pid.Turn.Location * TOP_ANGLE*2;
	
//	AdcGetVal[0] = MPU6050_Angle.roll;
//  AdcGetVal[1] = accel_roll_fai / PI * 180;
//	AdcGetVal[2] = Roll_Pid.Location.Location * TOP_ANGLE;
}