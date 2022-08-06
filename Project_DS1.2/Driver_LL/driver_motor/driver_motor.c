#include "driver_motor.h"
#include "kalman_filter.h"
#include <stdlib.h>

#define SPEEDLOOP_TIAOSHI 0 //�����ٶȻ�����
#define LOCALOOP_TIAOSHI 1  //λ�û�����

#define TIM_MOT1 TIM2  //PITCHʹ�ö�ʱ��
#define TIM_MOT2 TIM4  //mot2ʹ�ö�ʱ��

#define TOP_SPEED 2200.0f  //����� �����ٶȹ�һ��     
#define LIMIT_TOP_PWM 2000*0.95   //�޸�pwm�٣��Զ���װ�أ�
//#define LIMIT_BOTTOM_PWM 580 //����pwm�٣��Զ���װ�أ�
//#define START_PWM 450  //����pwm�٣��Զ���װ�أ�

//pitch_pid pid variable
#define PITCH_LOCATION_KP (0)
#define PITCH_LOCATION_KI (0)   
#define PITCH_LOCATION_KD (7.4)//15*0.6
#define PITCH_LOCATION_KC (0)
#define PITCH_LOCATION_AP (5.6)//10*0.6
#define PITCH_LOCATION_BP (0)
#define PITCH_LOCATION_CP (0)
#define PITCH_SPEED_KP (0)							

#define PITCH_SPEED_KD (0)
#define PITCH_SPEED_KC (0)   
#define PITCH_SPEED_AP (-0.38)//-0.38								
#define PITCH_SPEED_BP (0)								
#define PITCH_SPEED_CP (0)
#define PITCH_SPEED_KI (0)

#define PITCH_TURN_KP (0)							
#define PITCH_TURN_KI (0)
#define PITCH_TURN_KD (0)
#define PITCH_TURN_KC (0)   
#define PITCH_TURN_AP (-7)								
#define PITCH_TURN_BP (-8)								
#define PITCH_TURN_CP (0.1)


int LIMIT_BOTTOM_PWM=580; //����pwm�٣��Զ���װ�أ�
int START_PWM=450;
//---------kalman variable--------//
// x = [v_PITCH,v_mot2] PITCH MOT2 �ٶ�
kalman_filter_init_t Attitude_motorspeed_filter_para = {
	.xhat_data = {0, 0},
  //Ԥ�������
	.P_data = {2, 0, 0, 2},//Э�������
  .A_data = {1, 0, 0, 1},//״̬ת�ƾ���
  .H_data = {1, 0, 0, 1},//����������
  .Q_data = {1, 0, 0, 1},//��˹��������
	//����ֵ���
  .R_data = {20, 0, 0, 20}//��ȷ���������
};
kalman_filter_t Attitude_motorspeed_kalman_filter={0};
//--------------------------------//

MotorStuct Motor_1,Motor_2;
PitchPIDStuct Pitch_Pid;
void Motor_Init(void)
{
	//������������ʼ��
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);      //������������ʱ��
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);      //������������ʱ��
	__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim6);                       //������ʱ���ж�

	//��������ʼ��
	MotorV_kalman_init();
	
	//���pid������ʼ��
	Pitch_Pid.PIDLocation.OutMax=1;
	Pitch_Pid.PIDLocation.OutMin=-1;
	Pitch_Pid.PIDLocation.calc=&PidCalc;
	Pitch_Pid.PIDLocation.clear=&PidClear;
	Pitch_Pid.PIDLocation.clear(&Pitch_Pid.PIDLocation);
	
	Pitch_Pid.PIDLocation.Kp	= PITCH_LOCATION_KP;
	Pitch_Pid.PIDLocation.Ki	= PITCH_LOCATION_KI;
	Pitch_Pid.PIDLocation.Kd	= PITCH_LOCATION_KD;
	Pitch_Pid.PIDLocation.Kc	= PITCH_LOCATION_KC;
	Pitch_Pid.PIDLocation.Ap	=	PITCH_LOCATION_AP;
	Pitch_Pid.PIDLocation.Bp	=	PITCH_LOCATION_BP;
	Pitch_Pid.PIDLocation.Cp	=	PITCH_LOCATION_CP;
	
	
	Pitch_Pid.PIDSpeed.OutMax=1;
	Pitch_Pid.PIDSpeed.OutMin=-1;
	Pitch_Pid.PIDSpeed.calc=&PidCalc;
	Pitch_Pid.PIDSpeed.clear=&PidClear;
	Pitch_Pid.PIDSpeed.clear(&Pitch_Pid.PIDSpeed);
	
	Pitch_Pid.PIDSpeed.Kp	=	PITCH_SPEED_KP;
	Pitch_Pid.PIDSpeed.Ki	=	PITCH_SPEED_KI;
	Pitch_Pid.PIDSpeed.Kd	=	PITCH_SPEED_KD;
	Pitch_Pid.PIDSpeed.Kc	=	PITCH_SPEED_KC;
	Pitch_Pid.PIDSpeed.Ap	=	PITCH_SPEED_AP;
	Pitch_Pid.PIDSpeed.Bp	=	PITCH_SPEED_BP;
	Pitch_Pid.PIDSpeed.Cp	=	PITCH_SPEED_CP;
	
	Pitch_Pid.PIDTurn.OutMax=1;
	Pitch_Pid.PIDTurn.OutMin=-1;
	Pitch_Pid.PIDTurn.calc=&PidCalc;
	Pitch_Pid.PIDTurn.clear=&PidClear;
	Pitch_Pid.PIDTurn.clear(&Pitch_Pid.PIDTurn);
	
	Pitch_Pid.PIDTurn.Kp	=	PITCH_TURN_KP;
	Pitch_Pid.PIDTurn.Ki	=	PITCH_TURN_KI;
	Pitch_Pid.PIDTurn.Kd	=	PITCH_TURN_KD;
	Pitch_Pid.PIDTurn.Kc	=	PITCH_TURN_KC;
	Pitch_Pid.PIDTurn.Ap	=	PITCH_TURN_AP;
	Pitch_Pid.PIDTurn.Bp	=	PITCH_TURN_BP;
	Pitch_Pid.PIDTurn.Cp	=	PITCH_TURN_CP;
	
	//pwm������ʼ��
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	
	Motor_1.con_flag = 0;
	Motor_2.con_flag = 0;
	
	Motor_1.turn_veer = 1;
	Motor_2.turn_veer = -1;
	
	Motor_1.tim.htim = htim1;
	Motor_2.tim.htim = htim1;
	
	Motor_1.tim.Channel[0] = TIM_CHANNEL_1;
	Motor_1.tim.Channel[1] = TIM_CHANNEL_2;
	Motor_2.tim.Channel[0] = TIM_CHANNEL_3;
	Motor_2.tim.Channel[1] = TIM_CHANNEL_4;
	
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
}


//---------------------------------�������������------------------------------------//
int TimX_encoder_cont(TIM_TypeDef * TIMx) // ��������������
{
		int Pluse_cont;
		Pluse_cont = getTIMx_DetaCnt(TIMx) - RELOADVALUE/2;	    //��������ת
		return Pluse_cont;
}
float Pluse_Calspeed1,Pluse_Calspeed2;
void TimX_encoder_solu(TIM_TypeDef * TIMx)		//���������ݴ���	
{		               									
		float calspeed = (float)TimX_encoder_cont(TIMx);
		if(TIMx == TIM_MOT1)	Pluse_Calspeed1 = calspeed*50.0f;
		if(TIMx == TIM_MOT2)	Pluse_Calspeed2 = calspeed*50.0f;	
		MotorV_kalman_dispose();
		Motorspeed_observe_jscope();	
}	


int getTIMx_DetaCnt(TIM_TypeDef * TIMx) //��ȡ��λʱ��������仯ֵ
{
	int cnt;
	cnt = TIMx->CNT;
	TIMx->CNT = RELOADVALUE/2;
	return cnt;
}

int obse_PITCH_v,    //jscope motor1 speed 
		obse_mot2_v;       //jscope motor2 speed 
int obse_kal_PITCH_v,
		obse_kal_mot2_v;
extern float AdcGetVal[3]; 
void Motorspeed_observe_jscope(void)     //J-scope �۲����ݴ���
{
	obse_PITCH_v = (int)Pluse_Calspeed1;
	obse_mot2_v = (int)Pluse_Calspeed2;
	obse_kal_PITCH_v = (int)Motor_1.Speed.Speed*TOP_SPEED;
	obse_kal_mot2_v = (int)Motor_2.Speed.Speed*TOP_SPEED;
	//AdcGetVal[0] = Motor_1.Speed.SetSpeed;
	//AdcGetVal[1] = Motor_1.Speed.Speed;
	//AdcGetVal[0] = Pitch_Pid.Location.SetLocation;
	//AdcGetVal[1] = Pitch_Pid.Location.Location;
}

//-----------------------------------�ٶȿ��������-------------------------------------//
void MotorV_kalman_init(void)            //������������ʼ��
{
	mat_init(&Attitude_motorspeed_kalman_filter.Q,2,2, Attitude_motorspeed_filter_para.Q_data);
	mat_init(&Attitude_motorspeed_kalman_filter.R,2,2, Attitude_motorspeed_filter_para.R_data);
}


int Kalinit_flag_motorv=0;//���������������ڳ�ֵ��Ծʱ��Ԥ��ƫ��
void MotorV_kalman_dispose(void)    //����ٶȿ������˲������ݴ���
{
	//x = [v_PITCH,v_mot2]
	if(Kalinit_flag_motorv == 0)
	{		
		Attitude_motorspeed_filter_para.xhat_data[0] = Pluse_Calspeed1;
		Attitude_motorspeed_filter_para.xhat_data[1] = Pluse_Calspeed2;
		kalman_filter_init(&Attitude_motorspeed_kalman_filter, &Attitude_motorspeed_filter_para);
	}
	if(Kalinit_flag_motorv < 50) Kalinit_flag_motorv ++; //�ڵ���50�κ�0.05s����ʹ���˲�������
	float *Attitude_motorv_result = kalman_filter_calc(&Attitude_motorspeed_kalman_filter, Pluse_Calspeed1, Pluse_Calspeed2);
	if(Kalinit_flag_motorv < 50)
	{
		Motor_1.Speed.Speed = Pluse_Calspeed1/TOP_SPEED;  
		Motor_2.Speed.Speed = Pluse_Calspeed2/TOP_SPEED;
		Pitch_Pid.Speed.Speed = (Motor_1.Speed.Speed + Motor_2.Speed.Speed)/2;
	}
	else
	{
		Motor_1.Speed.Speed	=	Attitude_motorv_result[0]/TOP_SPEED;
		Motor_2.Speed.Speed = Attitude_motorv_result[1]/TOP_SPEED;
		Pitch_Pid.Speed.Speed = (Motor_1.Speed.Speed + Motor_2.Speed.Speed)/2;
	}
}

//---------------------------------pid�������------------------------------------//
void Control_Datainput(MotorStuct *motor,PitchPIDStuct *pitch_pid,ControlStruct con)  //������������
{
	#if SPEEDLOOP_TIAOSHI
		motor->con_flag = con.con_flag;
		motor->PWMrun.PWM_k = con.con_k;                       //���ط���ϵ��
		motor->Speed.SetSpeed = con.con_speed;
	#endif
	
	#if LOCALOOP_TIAOSHI
		motor->con_flag = con.con_flag;
		motor->PWMrun.PWM_k = con.con_k;                       //���ط���ϵ��
		pitch_pid->Location.SetLocation = con.con_location;
		pitch_pid->Speed.SetSpeed = con.con_speed;
		pitch_pid->Turn.SetLocation = con.con_turn;
	#endif
}
void Control_PID(MotorStuct *motor1,MotorStuct *motor2,PitchPIDStuct *pitch_pid)         //����pid����
{
	#if SPEEDLOOP_TIAOSHI
		Control_speedloop(motor1);
		Control_speedloop(motor2);
	#endif
	#if LOCALOOP_TIAOSHI
		Control_speedloop(pitch_pid);
		Control_localoop(pitch_pid);
		Control_turnloop(pitch_pid);
	#endif
}
void Control_speedloop(PitchPIDStuct *pitch_pid)   //�����ٶȻ�
{
	pitch_pid->PIDSpeed.Ref = pitch_pid->Speed.SetSpeed;
	pitch_pid->PIDSpeed.Fdb = pitch_pid->Speed.Speed;
	pitch_pid->PIDSpeed.calc(&pitch_pid->PIDSpeed);
}
void Control_localoop(PitchPIDStuct *pitch_pid) 	 //����λ�û�
{
	pitch_pid->PIDLocation.Ref = pitch_pid->PIDSpeed.Out;
	pitch_pid->PIDLocation.Fdb = pitch_pid->Location.Location;
	pitch_pid->PIDLocation.calc(&pitch_pid->PIDLocation);
}
void Control_turnloop(PitchPIDStuct *pitch_pid) 	 //����ת��
{
	pitch_pid->PIDTurn.Ref = pitch_pid->Turn.SetLocation;
	pitch_pid->PIDTurn.Fdb = pitch_pid->Turn.Location;
	pitch_pid->PIDTurn.calc(&pitch_pid->PIDTurn);
}
void Motor_Run(MotorStuct *motor,PitchPIDStuct *pitch_pid)           //����������
{
	//������� 
	motor->PWMrun.PWM_Pid_A = pitch_pid->PIDLocation.Out * LIMIT_TOP_PWM;
	motor->PWMrun.PWM_Pid_T = pitch_pid->PIDTurn.Out * LIMIT_TOP_PWM * motor->turn_veer;
	motor->PWMrun.PWM_Out = motor->PWMrun.PWM_Pid_A + motor->PWMrun.PWM_Pid_T; 
	
	//���ѹ��
	double a=9/6728.f,
				 b=-16/29.f,
				 c=(double)START_PWM;
	double res;
	if((motor->PWMrun.PWM_Out>=0)&&(motor->PWMrun.PWM_Out<LIMIT_BOTTOM_PWM))
	{
		res = a*pow((double)motor->PWMrun.PWM_Out,2)+ b*(double)(motor->PWMrun.PWM_Out) + c;
	 	motor->PWMrun.PWM_Out = (int)res;
	}
	else if((motor->PWMrun.PWM_Out<0)&&(motor->PWMrun.PWM_Out>-LIMIT_BOTTOM_PWM))
	{
		res = -a*pow((double)motor->PWMrun.PWM_Out,2)+ b*(double)(motor->PWMrun.PWM_Out) - c;
		motor->PWMrun.PWM_Out = (int)res;
	}
	
	//�޸߷�
	if (motor->PWMrun.PWM_Out>=0)
	{
		motor->PWMrun.PWM_Out = ( motor->PWMrun.PWM_Out >= LIMIT_TOP_PWM) ? LIMIT_TOP_PWM : motor->PWMrun.PWM_Out;
	}
	else if(motor->PWMrun.PWM_Out<0)
	{
		motor->PWMrun.PWM_Out = ( motor->PWMrun.PWM_Out <= -LIMIT_TOP_PWM)? -LIMIT_TOP_PWM : motor->PWMrun.PWM_Out;
	}
	//ת��
	motor->PWMrun.veer = (motor->PWMrun.PWM_Out == 0) ? 0 : abs(motor->PWMrun.PWM_Out)/motor->PWMrun.PWM_Out;
	//����ֵ��
	motor->PWMrun.PWM_OutRaw = abs(motor->PWMrun.PWM_Out);
	
	if(!motor->con_flag)
	{
		__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[0],0);
		__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[1],0);

	}
	else	
	{			
		switch(motor->PWMrun.veer)
		{
			case(0):
			{
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[0],LIMIT_BOTTOM_PWM);
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[1],LIMIT_BOTTOM_PWM);
				break;
			}
			case(1):
			{	
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[0],motor->PWMrun.PWM_OutRaw);
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[1],0);
				break;
			}
			case(-1):
			{	
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[0],0);
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[1],motor->PWMrun.PWM_OutRaw);
				break;
			}
			default:
			{
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[0],LIMIT_BOTTOM_PWM);
				__HAL_TIM_SET_COMPARE(&motor->tim.htim,motor->tim.Channel[1],LIMIT_BOTTOM_PWM);
				break;
			}
		}
	}

}