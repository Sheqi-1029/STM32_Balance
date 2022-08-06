#ifndef __DRIVER_HANGING_H_
#define __DRIVER_HANGING_H_
#include "main.h"
#include "pid.h"
#include "tim.h"
typedef struct					//����Ƕ����ֵ
{
	float Zero_offset;  //��ֵ
	float	SetLocation;
	float Location;
}LocaAngStruct;					//�ǶȽṹ�嶨��

typedef struct					//pwm�������ֵ
{
	float PWM_Pid_A;    //pid��pwm����
	int veer;           //��ת�� 
	int PWM_Out;				//pwm���
	int PWM_basic_l;    //�ͻ�׼λ
	int PWM_basic_h;		//�߻�׼λ
}PWMDuoStruct;					//�����ṹ�嶨��

typedef struct					//��ʱ�����ֵ
{
		TIM_HandleTypeDef htim;
		uint32_t Channel;
}TimDuoStruct;					//��ʱ���ṹ�嶨��

typedef struct          //����ܲ���
{
	uint8_t con_flag;   //�ܿر�־ 0����λ�̶� 1����λ�̶� 2����λ���� 3����λ����
	PWMDuoStruct PWMrun;
	TimDuoStruct Tim;
}DuojiStruct;            //����ṹ�嶨��

typedef struct             //roll_PID�����ܲ���
{
	LocaAngStruct Location;
	PID	PIDLocation;
}RollPIDStruct;
typedef struct					//����������ֵ
{
	uint8_t con_flag;   //�ܿر�־
	float con_location; //-90~90
}Con_DuoStruct;					//���ƽṹ�嶨��

void Hanging_Init(void);

//pid�������
void Hanging_Datainput(DuojiStruct Duo[],RollPIDStruct *roll,Con_DuoStruct con);  //������������
void Hanging_PID(RollPIDStruct *roll);         //����pid����
void Hanging_localoop(RollPIDStruct *roll); 	  //����λ�û�

//����������
void Hanging_Run(DuojiStruct *duo,RollPIDStruct *roll);           //����������
void Duo_Xianfu(DuojiStruct *duo);
#endif