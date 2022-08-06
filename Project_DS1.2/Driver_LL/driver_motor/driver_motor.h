#ifndef __DRIVER_MOTOR_H_
#define __DRIVER_MOTOR_H_

#include "main.h" 
#include "tim.h"
#include "pid.h"
#include <math.h>

#define RR 30u    //电机减速比
#define RELOADVALUE 65534    //自动装载值

//电机结构体

typedef struct					//电机速度相关值
{
	float	SetSpeed;
	float Speed;
}SpeedStruct;						//速度结构体定义

typedef struct					//电机位置相关值
{
	float	SetLocation;
	float Location;
}LocationStruct;				//位置结构体定义

typedef struct					//pwm驱动相关值
{
	float PWM_Pid_A;          //位置环控pwm（平衡）
	float PWM_Pid_V;          //速度环控pwm（速度）
	float PWM_Pid_T;          //转向环控pwm（方向）
	float PWM_k;              //力矩分配系数
	int veer;             		//转向 
	int PWM_Out;						  //pwm输出
	int PWM_OutRaw;           //pwm输出绝对转化
}PWMrunStruct;					//驱动结构体定义

typedef struct					//定时器相关值
{
		TIM_HandleTypeDef htim;
		uint32_t Channel[2];
}TimMotorStruct;					//定时器结构体定义

typedef struct          //电机总参量
{
	uint8_t con_flag;     //受控标志
	int turn_veer;
	SpeedStruct	Speed;
	TimMotorStruct tim;
	PWMrunStruct PWMrun;
}MotorStuct;            //电机结构体定义

typedef struct             //pitch_PID控制总参量
{
	LocationStruct Location;     
	PID	PIDLocation;             //位置环（平衡环）
	SpeedStruct	Speed;
	PID	PIDSpeed;                //速度环
	LocationStruct Turn;
	PID	PIDTurn;                 //转向环
}PitchPIDStuct;            ///pitch_PID控制结构体定义

typedef struct					//电机控制相关值
{
	uint8_t con_flag;     //受控标志
	float	con_speed;
	float con_location;
	float con_turn;
	float con_k;
}ControlStruct;					//控制结构体定义


void Motor_Init(void);

//编码器计数相关
int TimX_encoder_cont(TIM_TypeDef * TIMx);  //编码器计数
void TimX_encoder_solu(TIM_TypeDef * TIMx);	//编码器数据处理													
int getTIMx_DetaCnt(TIM_TypeDef * TIMx); 		//读取单位时间内脉冲变化值

//速度卡尔曼相关
void MotorV_kalman_init(void);            //卡尔曼参数初始化
void MotorV_kalman_dispose(void);   //电机速度卡尔曼滤波器数据处理

//pid计算相关
void Control_Datainput(MotorStuct *motor,PitchPIDStuct *pitch_pid,ControlStruct con);  //控制数据输入
void Control_PID(MotorStuct *motor1,MotorStuct *motor2,PitchPIDStuct *pitch_pid);         //控制pid计算
void Control_speedloop(PitchPIDStuct *pitch_pid);          //控制速度环
void Control_localoop(PitchPIDStuct *pitch_pid); 	         //控制位置环
void Control_turnloop(PitchPIDStuct *pitch_pid); 	         //控制转向环
//电机驱动相关
void Motor_Run(MotorStuct *motor,PitchPIDStuct *pitch_pid);           //电机驱动相关

//J-scope 观测
void Motorspeed_observe_jscope(void);               	//J-scope 观测数据处理
#endif