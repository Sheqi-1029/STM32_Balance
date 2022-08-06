#include "pid.h"
#include "math.h"

void PidCalc(PID *u)
{	
		//变结构Kp//
		u->Kp = u->Ap + u->Bp * (1 - exp(-u->Cp * fabs(u->Err))); 
    //计算误差
    u->Err = u->Ref - u->Fdb;
		//计算比例输出
    u->Up = u->Kp * u->Err;
		//计算积分输出
    u->Ui = u->Ui + u->Ki * u->Up + u->Kc * u->SatErr;
		//清算积分项消弱
	  if(u->Ui>0.5 && u->Err<0)//转向之后积分项清零
	  {
		  u->Ui=0;
	  }
	  if(u->Ui<-0.5 && u->Err>0)
		{
			u->Ui=0;
		}
		//计算微分输出
    u->Ud = u->Kd*(u->Up - u->Up1);
		//计算输出
    u->OutPreSat = u->Up + u->Ui + u->Ud;     
    
		//限幅
    if (u->OutPreSat > u->OutMax)                   
      u->Out =  u->OutMax;
    else if (u->OutPreSat < u->OutMin)
      u->Out =  u->OutMin;  
    else
      u->Out = u->OutPreSat;
		
    //积分
    u->SatErr = u->Out - u->OutPreSat;
    //留存
    u->Up1 = u->Up;
}


void PidClear(PID *v)
{
		v->Ui = 0;
		v->Ud = 0;
}
