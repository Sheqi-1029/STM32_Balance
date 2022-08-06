#ifndef __TASK_MOTOR_H_
#define __TASK_MOTOR_H_

#include "main.h"
#include "driver_motor.h"


//�ܿ����
void Task_motor(void);           //����ܿ�
void MotorDriver(void);          //�ײ���������
void Motor_init_enable(void);    //��ʼ��ʹ��    

//ģʽ�������
void Motor_Lost_Run(void);			 //�ؿ�ģʽ
void Motor_Park_Run(void);       //ƽ��ֹͣģʽ
void Motor_RunRun(void);         //�˶�����ģʽ
void Motor_PIDset(void);         //PID��������

void Motor_Safe(void);           //�㵹����
#endif