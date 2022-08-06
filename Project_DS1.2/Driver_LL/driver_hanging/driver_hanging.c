#include "driver_hanging.h"

// 0.5ms ~ 2.5ms T=5ms D��0.1~0.5
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

DuojiStruct Duo[4]; //����ǰ�� 0��ǰ�� 1������ 2������ 3��ǰ��
RollPIDStruct Roll_Pid;
//pwmֵ���� ���˳ʱ��ת
int LowBasic_PWM[4] = {550,650,530,650};    //��λ��׼ֵ
int HighBasic_PWM[4] = {600,600,580,600};   //��λ��׼ֵ
int Duoveer[4] = {1,-1,-1,1};               //�������
void Hanging_Init(void)
{
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	//ROLL-pid������ʼ��
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
	
	//���������ʼ��
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

//---------------------------------pid�������------------------------------------//
void Hanging_Datainput(DuojiStruct Duo[],RollPIDStruct *roll,Con_DuoStruct con)  //������������
{
	for (int i=0;i<4;i++) Duo[i].con_flag = con.con_flag;
	roll->Location.SetLocation = con.con_location;
}
void Hanging_PID(RollPIDStruct *roll)         //����pid����
{
	Hanging_localoop(roll);
}
void Hanging_localoop(RollPIDStruct *roll) 	  //����λ�û�
{
	roll->PIDLocation.Ref = roll->Location.SetLocation;
	roll->PIDLocation.Fdb = roll->Location.Location;
	roll->PIDLocation.calc(&roll->PIDLocation);
}

//---------------------------------����������------------------------------------//
void Hanging_Run(DuojiStruct *duo,RollPIDStruct *roll)           //����������
{
		//pwm���
		duo->PWMrun.PWM_Pid_A = duo->PWMrun.veer * roll->PIDLocation.Out * (PWM_TOP_DUO-PWM_BOTTOM_DUO)/2;
		//�޷�
		duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out <= ANGLELIMIT_PWM1)?ANGLELIMIT_PWM1:duo->PWMrun.PWM_Out;
		duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out >= ANGLELIMIT_PWM2)?ANGLELIMIT_PWM2:duo->PWMrun.PWM_Out;
		
		if( duo->con_flag == 0) //��λ��׼ ����
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_basic_l);
		if( duo->con_flag == 1) //��λ��׼ ����
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_basic_h);
		if( duo->con_flag == 2) //��λ��׼ + roll  ��С��
		{
			duo->PWMrun.PWM_Out = (int)(duo->PWMrun.PWM_basic_l + duo->PWMrun.PWM_Pid_A);
			Duo_Xianfu(duo);
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_Out);
		}
		if( duo->con_flag == 3) //��λ��׼ + roll  �޴��
		{
			duo->PWMrun.PWM_Out = (int)(duo->PWMrun.PWM_basic_h + duo->PWMrun.PWM_Pid_A);
			Duo_Xianfu(duo);
			__HAL_TIM_SET_COMPARE(&duo->Tim.htim,duo->Tim.Channel,duo->PWMrun.PWM_Out);
		}
		
}

void Duo_Xianfu(DuojiStruct *duo)
{
	//��ǰ&�Һ� 
		if(duo == &Duo[0] || duo == &Duo[2]) // ��λ��׼pwmС����Ϊ��׼pwm��
		{
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out <= duo->PWMrun.PWM_basic_l)?duo->PWMrun.PWM_basic_l:duo->PWMrun.PWM_Out;
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out >= duo->PWMrun.PWM_basic_h)?duo->PWMrun.PWM_basic_h:duo->PWMrun.PWM_Out;
		}
		if(duo == &Duo[1] || duo == &Duo[3]) // ��λ��׼pwm�󣬸�Ϊ��׼pwmС
		{
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out >= duo->PWMrun.PWM_basic_l)?duo->PWMrun.PWM_basic_l:duo->PWMrun.PWM_Out;
			duo->PWMrun.PWM_Out = ( duo->PWMrun.PWM_Out <= duo->PWMrun.PWM_basic_h)?duo->PWMrun.PWM_basic_h:duo->PWMrun.PWM_Out;
		}
}