#include "pid.h"
#include "math.h"

void PidCalc(PID *u)
{	
		//��ṹKp//
		u->Kp = u->Ap + u->Bp * (1 - exp(-u->Cp * fabs(u->Err))); 
    //�������
    u->Err = u->Ref - u->Fdb;
		//����������
    u->Up = u->Kp * u->Err;
		//����������
    u->Ui = u->Ui + u->Ki * u->Up + u->Kc * u->SatErr;
		//�������������
	  if(u->Ui>0.5 && u->Err<0)//ת��֮�����������
	  {
		  u->Ui=0;
	  }
	  if(u->Ui<-0.5 && u->Err>0)
		{
			u->Ui=0;
		}
		//����΢�����
    u->Ud = u->Kd*(u->Up - u->Up1);
		//�������
    u->OutPreSat = u->Up + u->Ui + u->Ud;     
    
		//�޷�
    if (u->OutPreSat > u->OutMax)                   
      u->Out =  u->OutMax;
    else if (u->OutPreSat < u->OutMin)
      u->Out =  u->OutMin;  
    else
      u->Out = u->OutPreSat;
		
    //����
    u->SatErr = u->Out - u->OutPreSat;
    //����
    u->Up1 = u->Up;
}


void PidClear(PID *v)
{
		v->Ui = 0;
		v->Ud = 0;
}
