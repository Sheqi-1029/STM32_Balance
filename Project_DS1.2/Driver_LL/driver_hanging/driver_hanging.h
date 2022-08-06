#ifndef __DRIVER_HANGING_H_
#define __DRIVER_HANGING_H_
#include "main.h"
#include "pid.h"
#include "tim.h"
typedef struct					//舵机角度相关值
{
	float Zero_offset;  //基值
	float	SetLocation;
	float Location;
}LocaAngStruct;					//角度结构体定义

typedef struct					//pwm驱动相关值
{
	float PWM_Pid_A;    //pid控pwm增量
	int veer;           //正转向 
	int PWM_Out;				//pwm输出
	int PWM_basic_l;    //低基准位
	int PWM_basic_h;		//高基准位
}PWMDuoStruct;					//驱动结构体定义

typedef struct					//定时器相关值
{
		TIM_HandleTypeDef htim;
		uint32_t Channel;
}TimDuoStruct;					//定时器结构体定义

typedef struct          //舵机总参量
{
	uint8_t con_flag;   //受控标志 0：低位固定 1：高位固定 2：低位悬挂 3：高位悬挂
	PWMDuoStruct PWMrun;
	TimDuoStruct Tim;
}DuojiStruct;            //舵机结构体定义

typedef struct             //roll_PID控制总参量
{
	LocaAngStruct Location;
	PID	PIDLocation;
}RollPIDStruct;
typedef struct					//舵机控制相关值
{
	uint8_t con_flag;   //受控标志
	float con_location; //-90~90
}Con_DuoStruct;					//控制结构体定义

void Hanging_Init(void);

//pid计算相关
void Hanging_Datainput(DuojiStruct Duo[],RollPIDStruct *roll,Con_DuoStruct con);  //控制数据输入
void Hanging_PID(RollPIDStruct *roll);         //控制pid计算
void Hanging_localoop(RollPIDStruct *roll); 	  //控制位置环

//舵机驱动相关
void Hanging_Run(DuojiStruct *duo,RollPIDStruct *roll);           //舵机驱动相关
void Duo_Xianfu(DuojiStruct *duo);
#endif