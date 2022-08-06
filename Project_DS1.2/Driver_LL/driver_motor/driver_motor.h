#ifndef __DRIVER_MOTOR_H_
#define __DRIVER_MOTOR_H_

#include "main.h" 
#include "tim.h"
#include "pid.h"
#include <math.h>

#define RR 30u    //������ٱ�
#define RELOADVALUE 65534    //�Զ�װ��ֵ

//����ṹ��

typedef struct					//����ٶ����ֵ
{
	float	SetSpeed;
	float Speed;
}SpeedStruct;						//�ٶȽṹ�嶨��

typedef struct					//���λ�����ֵ
{
	float	SetLocation;
	float Location;
}LocationStruct;				//λ�ýṹ�嶨��

typedef struct					//pwm�������ֵ
{
	float PWM_Pid_A;          //λ�û���pwm��ƽ�⣩
	float PWM_Pid_V;          //�ٶȻ���pwm���ٶȣ�
	float PWM_Pid_T;          //ת�򻷿�pwm������
	float PWM_k;              //���ط���ϵ��
	int veer;             		//ת�� 
	int PWM_Out;						  //pwm���
	int PWM_OutRaw;           //pwm�������ת��
}PWMrunStruct;					//�����ṹ�嶨��

typedef struct					//��ʱ�����ֵ
{
		TIM_HandleTypeDef htim;
		uint32_t Channel[2];
}TimMotorStruct;					//��ʱ���ṹ�嶨��

typedef struct          //����ܲ���
{
	uint8_t con_flag;     //�ܿر�־
	int turn_veer;
	SpeedStruct	Speed;
	TimMotorStruct tim;
	PWMrunStruct PWMrun;
}MotorStuct;            //����ṹ�嶨��

typedef struct             //pitch_PID�����ܲ���
{
	LocationStruct Location;     
	PID	PIDLocation;             //λ�û���ƽ�⻷��
	SpeedStruct	Speed;
	PID	PIDSpeed;                //�ٶȻ�
	LocationStruct Turn;
	PID	PIDTurn;                 //ת��
}PitchPIDStuct;            ///pitch_PID���ƽṹ�嶨��

typedef struct					//����������ֵ
{
	uint8_t con_flag;     //�ܿر�־
	float	con_speed;
	float con_location;
	float con_turn;
	float con_k;
}ControlStruct;					//���ƽṹ�嶨��


void Motor_Init(void);

//�������������
int TimX_encoder_cont(TIM_TypeDef * TIMx);  //����������
void TimX_encoder_solu(TIM_TypeDef * TIMx);	//���������ݴ���													
int getTIMx_DetaCnt(TIM_TypeDef * TIMx); 		//��ȡ��λʱ��������仯ֵ

//�ٶȿ��������
void MotorV_kalman_init(void);            //������������ʼ��
void MotorV_kalman_dispose(void);   //����ٶȿ������˲������ݴ���

//pid�������
void Control_Datainput(MotorStuct *motor,PitchPIDStuct *pitch_pid,ControlStruct con);  //������������
void Control_PID(MotorStuct *motor1,MotorStuct *motor2,PitchPIDStuct *pitch_pid);         //����pid����
void Control_speedloop(PitchPIDStuct *pitch_pid);          //�����ٶȻ�
void Control_localoop(PitchPIDStuct *pitch_pid); 	         //����λ�û�
void Control_turnloop(PitchPIDStuct *pitch_pid); 	         //����ת��
//����������
void Motor_Run(MotorStuct *motor,PitchPIDStuct *pitch_pid);           //����������

//J-scope �۲�
void Motorspeed_observe_jscope(void);               	//J-scope �۲����ݴ���
#endif