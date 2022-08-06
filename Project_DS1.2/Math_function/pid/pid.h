#ifndef __PID_H__
#define __PID_H__
#include "main.h"
typedef struct 
{
	float  Ref;   				// 目标参考输入Input: Reference input
	float  Fdb;   				// 现实反馈输入Input: Feedback input
	float  Err;						// 误差Variable: Error
	
	float  Kp;						// 比例项系数Parameter: Proportional gain
	float  Ki;			   		// 积分项系数Parameter: Integral gain
	float  Kd; 		        // 微分项系数Parameter: Derivative gain
	
	float  Up;						// 比例输出Variable: Proportional output
	float  Ui;						// 积分输出Variable: Integral output
	float  Ud;						// 微分输出Variable: Derivative output
	float  OutPreSat; 		// 预处理输出Variable: Pre-saturated output
	float  OutMax;		    // 输出最大值Parameter: Maximum output
	float  OutMin;	    	// 输出最小值Parameter: Minimum output
	float  Out;   				// 处理后输出Output: PID output
	float  SatErr;				// 饱和误差Variable: Saturated difference
	float  Kc;		     		// 抗饱和积分系数Parameter: Integral correction gain
	float  Up1;		   	    // 上次输出History: Previous proportional output
	void  (*calc)();	  	// Pointer to calculation function
	void  (*clear)();
	
	//变结构PID Kp 的参数
	float Ap;           //电机静止时的kp值（防止kp过大而抖动）
	float Bp;						//电机以最大Kp运行时的Kp值（在Err过大时保证快速响应）
	float Cp;						//Kp增大的速率

} PID;

void PidCalc(PID *u);
void PidClear(PID *u);

#endif
