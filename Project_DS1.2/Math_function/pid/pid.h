#ifndef __PID_H__
#define __PID_H__
#include "main.h"
typedef struct 
{
	float  Ref;   				// Ŀ��ο�����Input: Reference input
	float  Fdb;   				// ��ʵ��������Input: Feedback input
	float  Err;						// ���Variable: Error
	
	float  Kp;						// ������ϵ��Parameter: Proportional gain
	float  Ki;			   		// ������ϵ��Parameter: Integral gain
	float  Kd; 		        // ΢����ϵ��Parameter: Derivative gain
	
	float  Up;						// �������Variable: Proportional output
	float  Ui;						// �������Variable: Integral output
	float  Ud;						// ΢�����Variable: Derivative output
	float  OutPreSat; 		// Ԥ�������Variable: Pre-saturated output
	float  OutMax;		    // ������ֵParameter: Maximum output
	float  OutMin;	    	// �����СֵParameter: Minimum output
	float  Out;   				// ��������Output: PID output
	float  SatErr;				// �������Variable: Saturated difference
	float  Kc;		     		// �����ͻ���ϵ��Parameter: Integral correction gain
	float  Up1;		   	    // �ϴ����History: Previous proportional output
	void  (*calc)();	  	// Pointer to calculation function
	void  (*clear)();
	
	//��ṹPID Kp �Ĳ���
	float Ap;           //�����ֹʱ��kpֵ����ֹkp�����������
	float Bp;						//��������Kp����ʱ��Kpֵ����Err����ʱ��֤������Ӧ��
	float Cp;						//Kp���������

} PID;

void PidCalc(PID *u);
void PidClear(PID *u);

#endif
