#include "driver_mpu6050.h"
#include "kalman_filter.h"
#include "task_attitude.h"
#include "driver_motor.h"
#include "driver_hanging.h"
#include "inv_mpu.h"
#include <math.h>	


#define TOP_ANGLE 180    //���� ����λ�ù�һ��


//---------kalman variable--------//
// x = [theta,w_theta] pitch
kalman_filter_init_t Attitude_pitch_kalman_filter_para = {
	.xhat_data = {0, 0},
  //Ԥ�������
	.P_data = {2, 0, 0, 2},//Э�������
  .A_data = {1, 10, 0, 1},//״̬ת�ƾ��� �ڶ���Ϊdt
  .H_data = {1, 0, 0, 1},//����������
  .Q_data = {1, 0, 0, 1},//��˹��������
	//����ֵ���
  .R_data = {15, 0, 0, 150}//��ȷ���������
};
kalman_filter_t Attitude_pitch_kalman_filter={0};
// x = [fai,w_fai] roll
kalman_filter_init_t Attitude_roll_kalman_filter_para = {
	.xhat_data = {0, 0},
  //Ԥ�������
	.P_data = {2, 0, 0, 2},//Э�������
  .A_data = {1, 0.05, 0, 1},//״̬ת�ƾ��� �ڶ���Ϊdt
  .H_data = {1, 0, 0, 1},//����������
  .Q_data = {1, 0, 0, 1},//��˹��������
	//����ֵ���
  .R_data = {100, 0, 0, 12000}//��ȷ���������
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
float accel_pitch_theta,	//���ٶ�pitch�ǶȽ���
			accel_roll_fai;			//���ٶ�roll�ǶȽ���
float gyro_pitch_w,				//������pitch���ٶȽ���
			gyro_roll_w,        //������roll���ٶȽ���
			gyro_yaw_w;        //������yaw���ٶȽ���
uint8_t attitude_init_flag = 0;   //��ʼ����־λ

float yaw_dt = 0.009;//yaw����



float offset_pitch = 2.3,//ƫ��
			offset_roll = 2;
//---------------------------------//

//---jscope observe variable------//
int obse_pitch_theta,    //jscope pitch�Ƕ� 
		obse_roll_fai;       //jscope roll�Ƕ�
int obse_kal_pitch_theta,    //jscope pitch�Ƕ� 
		obse_kal_roll_fai;       //jscope roll�Ƕ�
//---------------------------------//

//----------------------------------��̬�������-------------------------------------//
void Task_attitude_sol(void)           //��̬���㺯��
{
	if(!attitude_init_flag)
	{
		Attitude_kalman_init();
		attitude_init_flag = 1;
	}
	
	//MPU6050_DATA_Read();
	
	//mpu_dmp_get_data(&MPU6050_Angle.pitch,&MPU6050_Angle.roll,&MPU6050_Angle.yaw);
	
	accel_attitude_aol();              //���ٶ���̬����
	gyro_attitude_aol();               //�����ǽ��ٶȽ���
	
	Attitude_kalman_pitch_dispose();   //pitch�������˲������ݴ���
	Attitude_kalman_roll_dispose();    //roll�������˲������ݴ���
	
	float add_yaw;
	add_yaw = (fabs(gyro_yaw_w * yaw_dt) < 0.1) ? 0 : gyro_yaw_w * yaw_dt; 
	MPU6050_Angle.yaw += add_yaw; //yaw��ͨ�˲������ݴ���
	
	
	zero_sol();                        //��ƫ����
	WM_locaupdate();
	Attitude_observe_jscope();
}

void accel_attitude_aol(void)          //���ٶ���̬����
/*
		�� = arctan(-Ax / sqrt(Ay^2 + Az^2)) //pitch
		�� = arctan(Ay / sqrt(Ax^2 + Az^2)) 	//roll
*/
{
	accel_pitch_theta = atan( - Mpu6050_Data.Accel_X / sqrt( pow(Mpu6050_Data.Accel_Y,2) + pow(Mpu6050_Data.Accel_Z,2)));
	accel_roll_fai = atan( Mpu6050_Data.Accel_Y / sqrt( pow(Mpu6050_Data.Accel_X,2) + pow(Mpu6050_Data.Accel_Z,2)));
}

void gyro_attitude_aol(void)           //�����ǽ��ٶȽ���
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

void zero_sol(void)                   //��ƫ����
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
void WM_locaupdate(void)           //����λ�ø���
{
	//��� - pitch   ��һ��
	Pitch_Pid.Location.Location =  pitch_lowK * MPU6050_Angle.pitch / TOP_ANGLE + (1-pitch_lowK) * last_pitchlocation;
	Pitch_Pid.Turn.Location = MPU6050_Angle.yaw /(TOP_ANGLE*2.0f);
	Roll_Pid.Location.Location = roll_lowK * MPU6050_Angle.roll / TOP_ANGLE + (1-roll_lowK) * last_rollloacation;
	
	last_pitchlocation = Pitch_Pid.Location.Location;
	last_turnlocation = Pitch_Pid.Turn.Location;
	last_rollloacation = Roll_Pid.Location.Location;
}

//-----------------------------------���������-------------------------------------//
void Attitude_kalman_init(void)            //������������ʼ��
{
	mat_init(&Attitude_pitch_kalman_filter.Q,2,2, Attitude_pitch_kalman_filter_para.Q_data);
	mat_init(&Attitude_pitch_kalman_filter.R,2,2, Attitude_pitch_kalman_filter_para.R_data);
	mat_init(&Attitude_roll_kalman_filter.Q,2,2, Attitude_roll_kalman_filter_para.Q_data);
	mat_init(&Attitude_roll_kalman_filter.R,2,2, Attitude_roll_kalman_filter_para.R_data);
}


int Kalinit_flag_pitch=0;//���������������ڳ�ֵ��Ծʱ��Ԥ��ƫ��
void Attitude_kalman_pitch_dispose(void)    //pitch�������˲������ݴ���
{
	//x = [theta,w_theta]
	if(Kalinit_flag_pitch == 0)
	{		
		Attitude_pitch_kalman_filter_para.xhat_data[0] = accel_pitch_theta;
		Attitude_pitch_kalman_filter_para.xhat_data[1] = gyro_pitch_w;
		kalman_filter_init(&Attitude_pitch_kalman_filter, &Attitude_pitch_kalman_filter_para);
	}
	if(Kalinit_flag_pitch < 50) Kalinit_flag_pitch ++; //�ڵ���50�κ�0.05s����ʹ���˲�������
	float *Attitude_pitch_result = kalman_filter_calc(&Attitude_pitch_kalman_filter, accel_pitch_theta, gyro_pitch_w);
	if(Kalinit_flag_pitch < 50)
	{
		MPU6050_Angle.pitch = accel_pitch_theta / PI * 180 ;       	//ת��Ϊ�Ƕ�
	}
	else
	{
		MPU6050_Angle.pitch	=	Attitude_pitch_result[0] / PI * 180 ;  //ת��Ϊ�Ƕ�
	}
}
int Kalinit_flag_roll=0;//���������������ڳ�ֵ��Ծʱ��Ԥ��ƫ��
void Attitude_kalman_roll_dispose(void)   //pitch�������˲������ݴ���
{
	//x = [fai,w_fai]
	if(Kalinit_flag_roll == 0)
	{		
		Attitude_roll_kalman_filter_para.xhat_data[0] = accel_roll_fai;
		Attitude_roll_kalman_filter_para.xhat_data[1] = gyro_roll_w;
		kalman_filter_init(&Attitude_roll_kalman_filter, &Attitude_roll_kalman_filter_para);
	}
	if(Kalinit_flag_roll < 50) Kalinit_flag_roll ++; //�ڵ���50�κ�0.05s����ʹ���˲�������
	float *Attitude_roll_result = kalman_filter_calc(&Attitude_roll_kalman_filter, accel_roll_fai, gyro_roll_w);
	if(Kalinit_flag_roll < 50)
	{
		MPU6050_Angle.roll = accel_roll_fai / PI * 180;         	  //ת��Ϊ�Ƕ�
	}
	else
	{ 
		MPU6050_Angle.roll	=	Attitude_roll_result[0] / PI * 180;   //ת��Ϊ�Ƕ�
	}
}

//-----------------------------------�۲����-------------------------------------//
extern float AdcGetVal[4]; 
void Attitude_observe_jscope()              	//�۲����ݴ���
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