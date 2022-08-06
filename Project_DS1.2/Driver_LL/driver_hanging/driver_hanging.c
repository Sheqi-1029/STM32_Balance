#include "driver_hanging.h"

// 0.5ms ~ 2.5ms T=5ms D：0.1~0.5
#define CONT_PERIOD     2000
#define PWM_TOP_DUO     0.1*CONT_PERIOD
#define PWM_BOTTOM_DUO  0.5*CONT_PERIOD
#define ANGLELIMIT_PWM1  500  
#define ANGLELIMIT_PWM2  700

//Roll pid variable
#define ROLL_LOCATION_KP (0)
#define ROLL_LOCATION_KI (0)   
#define ROLL_LOCATION_KD (0)
#define ROLL_LOCATION_KC (0)
#define ROLL_LOCATION_AP (-3)
#define ROLL_LOCATION_BP (0)
#define ROLL_LOCATION_CP (0)

DuojiStruct Duo[4]; //后往前看 0：前左 1：后左 2：后右 3：前右
RollPIDStruct Roll_Pid;
//pwm值增加 舵机顺时针转
int LowBasic_PWM[4] = {550,650,530,650};    //低位基准值
int HighBasic_PWM[4] = {600,600,580,600};   //高位基准值
int Duoveer[4] = {1,-1,-1,1};               //舵机极性
void Hanging_Init(void)
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	//ROLL-pid参数初始化
	Roll_Pid.PIDLocation.OutMax=1;
	Roll_Pid.PIDLocation.OutMin=-1;
	Roll_Pid.PIDLocation.calc=&PidCalc;
	Roll_Pid.PIDLocation.clear=&PidClear;
	Roll_Pid.PIDLocation.clear(&Roll_Pid.PIDLocation);
	
	Roll_Pid.PIDLocation.Kp	= ROLL_LOCATION_KP;
	Roll_Pid.PIDLocation.Ki	= ROLL_LOCATION_KI;
	Roll_Pid.PIDLocation.Kd	= ROLL_LOCATION_KD;
	Roll_Pid.PIDLocation.Kc	= ROLL_LOCATION_KC;
	Roll_Pid.PIDLocation.Ap	=	ROLL_LOCATION_AP;
	Roll_Pid.PIDLocation.Bp	=	ROLL_LOCATION_BP;
	Roll_Pid.PIDLocation.Cp	=	ROLL_LOCATION_CP;
	
	//舵机参数初始化
	for(int i=0;i<4;i++)
	{
		Duo[i].con_flag = 0;
		Duo[i].Tim.htim = htim3;
		Duo[i].Tim.Channel = TIM_CHANNEL_1 + (TIM_CHANNEL_2-TIM_CHANNEL_1) * i;
		Duo[i].PWMrun.PWM_basic_l = LowBasic_PWM[i];
		Duo[i].PWMrun.PWM_basic_h = HighBasic_PWM[i];
		Duo[i].PWMrun.veer = Duoveer[i];
		
		__HAL_TIM_SET_COMPARE(&Duo[i].Tim.htim,Duo[i].Tim.Channel,Duo[i].PWMrun.PWM_basic_h);
	}
}

//---------------------------------pid计算相关------------------------------------//
void Hanging_Datainput(DuojiStruct Duo[],RollPIDStruct *roll,Con_DuoStruct con)  //控制数据输入
{
	for (int i=0;i<4;i++) Duo[i].con_flag = con.con_flag;
	roll->Location.SetLocation = con.con_location;
}
void Hanging_PID(RollPIDStruct *roll)         //控制pid计算
{
	Hanging_localoop(roll);
}
void Hanging_localoop(RollPIDStruct *roll) 	  //控制位置环
{
	roll->PIDLocation.Ref = roll->Location.SetLocation;
	roll->PIDLocation.Fdb = roll->Location.Location;
	roll->PIDLocation.calc(&roll->PIDLocation);
}

//---------------------------------舵机驱动相关------------------------------------//
void Hanging_Run(DuojiStruct *duo,RollPIDStruct *roll)           //舵机驱动相关
{
		//pwm输出
		duo->PWMrun.PWM_Pid_A = duo->PWMrun.veer * roll->PIDLocation.Out * (PWM_TOP_DUO-PWM_BOTTOM_DUO)/2;
		//限幅
		duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out <= ANGLELIMIT_PWM1)?ANGLELIMIT_PWM1:duo->PWMrun.PWM_Out;
		duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out >= ANGLELIMIT_PWM2)?ANGLELIMIT_PWM2:duo->PWMrun.PWM_Out;
		
		if( duo->con_flag == 0) //低位基准 不动
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_basic_l);
		if( duo->con_flag == 1) //高位基准 不动
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_basic_h);
		if( duo->con_flag == 2) //低位基准 + roll  限小幅
		{
			duo->PWMrun.PWM_Out = (int)(duo->PWMrun.PWM_basic_l + duo->PWMrun.PWM_Pid_A);
			Duo_Xianfu(duo);
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_Out);
		}
		if( duo->con_flag == 3) //高位基准 + roll  限大幅
		{
			duo->PWMrun.PWM_Out = (int)(duo->PWMrun.PWM_basic_h + duo->PWMrun.PWM_Pid_A);
			Duo_Xianfu(duo);
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_Out);
		}
		
}

void Duo_Xianfu(DuojiStruct *duo)
{
	//左前&右后 
		if(duo == &Duo[0] || duo == &Duo[2]) // 低位基准pwm小，高为基准pwm大
		{
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out <= duo->PWMrun.PWM_basic_l)?duo->PWMrun.PWM_basic_l:duo->PWMrun.PWM_Out;
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out >= duo->PWMrun.PWM_basic_h)?duo->PWMrun.PWM_basic_h:duo->PWMrun.PWM_Out;
		}
		if(duo == &Duo[1] || duo == &Duo[3]) // 低位基准pwm大，高为基准pwm小
		{
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out >= duo->PWMrun.PWM_basic_l)?duo->PWMrun.PWM_basic_l:duo->PWMrun.PWM_Out;
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out <= duo->PWMrun.PWM_basic_h)?duo->PWMrun.PWM_basic_h:duo->PWMrun.PWM_Out;
		}
}