#ifndef __TASK_MOTOR_H_
#define __TASK_MOTOR_H_

#include "main.h"
#include "driver_motor.h"


//总控相关
void Task_motor(void);           //电机总控
void MotorDriver(void);          //底层驱动连接
void Motor_init_enable(void);    //初始化使能    

//模式驱动相关
void Motor_Lost_Run(void);			 //关控模式
void Motor_Park_Run(void);       //平衡停止模式
void Motor_RunRun(void);         //运动控制模式
void Motor_PIDset(void);         //PID参数设置

void Motor_Safe(void);           //倾倒保护
#endif