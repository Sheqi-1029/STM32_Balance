#include "task_motor.h"
#include "task_hanging.h"

//-----significant variable--------//
uint32_t MotorPWM[4] = {0,0,0,0};
ControlStruct MotorControl_1,MotorControl_2,Motor_CON;
extern MotorStuct Motor_1,Motor_2;
extern PitchPIDStuct Pitch_Pid;
uint8_t motor_init_flag = 0;            //��ʼ����־λ
uint8_t motor_fall_flag = 0;            //�㵹��־λ

uint8_t RX;//����

extern int mode_flag; //���ģʽ
//---------------------------------//

//------- observe variable---------//




//---------------------------------//

//�����߼�
/*   e = ��λ
     a = ǰ�� b = ���� c = ��ת d = ��ת
     f = ���� g = ���ϰ�
*/
//---------------------------------�ܿ����------------------------------------//
void Task_motor(void)
{
	//Motor_Safe();
//	Motor_Lost_Run();
	
	Motor_PIDset();
	if(RX=='e'||RX== 0 )       //��λ       
		Motor_Park_Run();
	else             //��������
		Motor_RunRun();
	
	MotorDriver();
	
}

void MotorDriver(void)                  //�ײ���������
{
	MotorControl_1.con_flag = Motor_CON.con_flag;
	MotorControl_1.con_k = Motor_CON.con_k;
	MotorControl_1.con_speed = Motor_CON.con_speed;
	MotorControl_1.con_location = Motor_CON.con_location;
	MotorControl_1.con_turn = Motor_CON.con_turn;
	
	MotorControl_2.con_flag = Motor_CON.con_flag;
	MotorControl_2.con_k = 1 - Motor_CON.con_k;
	MotorControl_2.con_speed = Motor_CON.con_speed;
	MotorControl_2.con_location = Motor_CON.con_location;
	MotorControl_2.con_turn = Motor_CON.con_turn;
	
	Control_Datainput(&Motor_1,&Pitch_Pid,MotorControl_1);
	Control_Datainput(&Motor_2,&Pitch_Pid,MotorControl_2);
	Control_PID(&Motor_1,&Motor_2,&Pitch_Pid);
	Motor_Run(&Motor_1,&Pitch_Pid);
	Motor_Run(&Motor_2,&Pitch_Pid);
}

void Motor_init_enable(void)   					//��ʼ��ʹ��  
{
	Motor_CON.con_flag = 1;
	Motor_CON.con_k = 0.5; 
	Motor_CON.con_speed = 0.02;
	Motor_CON.con_location = 0;
}
//---------------------------------ģʽ�������------------------------------------//
void Motor_Lost_Run(void)            //�ؿ�ģʽ
{
	motor_init_flag = 0;
	Motor_CON.con_flag = 0;
}	
void Motor_Park_Run(void)            //ƽ��ֹͣģʽ
{
	if(!motor_init_flag)
	{
		Motor_CON.con_turn = Pitch_Pid.Turn.Location;
		motor_init_flag = 1;
	}
	
	Pitch_Pid.PIDLocation.Ap	=	6.0;
	Pitch_Pid.PIDLocation.Kd	= 7.8;
	
	Motor_init_enable();
}
void Motor_RunRun(void)       //�˶�����ģʽ
{
	if(!motor_init_flag)
	{
		Motor_init_enable();
		Motor_CON.con_turn = Pitch_Pid.Turn.Location;
		motor_init_flag = 1;
	}
		
	if((mode_flag == 0)||(mode_flag == 2))   //��λ��׼
	{
		//ǰ��
		if(RX == 'a')
		{
			Motor_CON.con_speed = -0.08;
			Pitch_Pid.PIDLocation.Ap	=	5.0;
			Pitch_Pid.PIDLocation.Kd	= 7.3;
			Pitch_Pid.PIDSpeed.Ap	=	-0.36f;
		}
		//����
		else if(RX == 'b')
		{
			Motor_CON.con_speed = 0.12;
			Pitch_Pid.PIDLocation.Ap	=	5.0;
			Pitch_Pid.PIDLocation.Kd	= 7.3;
			Pitch_Pid.PIDSpeed.Ap	=	-0.36f;
		}
		//��ת
		else if(RX == 'c')
		{
			Motor_CON.con_speed = 0;
			Motor_CON.con_turn += 0.001;
			Pitch_Pid.PIDLocation.Ap	=	5.2;
			Pitch_Pid.PIDLocation.Kd	= 7.2;
		}
		//��ת
		else if(RX == 'd')
		{
			Motor_CON.con_speed = 0;
			Motor_CON.con_turn -= 0.001;
			Pitch_Pid.PIDLocation.Ap	=	5.2;
			Pitch_Pid.PIDLocation.Kd	= 7.2;
		}
	}
	else if((mode_flag == 1)||(mode_flag == 3))   //��λ��׼
	{
		//ǰ��
		if(RX == 'a')
		{
			Motor_CON.con_speed = -0.06;
			Pitch_Pid.PIDLocation.Ap	=	4.7;
			Pitch_Pid.PIDLocation.Kd	= 7.0;
			Pitch_Pid.PIDSpeed.Ap	=	-0.34f;
		}
		//����
		else if(RX == 'b')
		{
			Motor_CON.con_speed = 0.1;
			Pitch_Pid.PIDLocation.Ap	=	4.7;
			Pitch_Pid.PIDLocation.Kd	= 7.0;
			Pitch_Pid.PIDSpeed.Ap	=	-0.34f;
		}
		//��ת
		else if(RX == 'c')
		{
			Motor_CON.con_speed = 0;
			Motor_CON.con_turn += 0.001;
			Pitch_Pid.PIDLocation.Ap	=	4.8;
			Pitch_Pid.PIDLocation.Kd	= 7.0;
		}
		//��ת
		else if(RX == 'd')
		{
			Motor_CON.con_speed = 0;
			Motor_CON.con_turn -= 0.001;
			Pitch_Pid.PIDLocation.Ap	=	4.8;
			Pitch_Pid.PIDLocation.Kd	= 7.0;
		}
	}
}
void Motor_PIDset(void)//PID��������
{
	if(mode_flag == 0)
	{
		Pitch_Pid.PIDLocation.Ap	=	5.2f;//5.55
		Pitch_Pid.PIDLocation.Kd	= 7.3f;//7.4
		Pitch_Pid.PIDSpeed.Ap	=	-0.37f;//-0.38
	}		
	if(mode_flag == 2)
	{
		Pitch_Pid.PIDLocation.Ap	=	4.8f;//5.55
		Pitch_Pid.PIDLocation.Kd	= 7.0f;//7.4
		Pitch_Pid.PIDSpeed.Ap	=	-0.35f;//-0.38
	}		
	if((mode_flag == 1)||(mode_flag == 3))
	{
		Pitch_Pid.PIDLocation.Ap	=	4.8f;
		Pitch_Pid.PIDLocation.Kd	= 7.0f;
		Pitch_Pid.PIDSpeed.Ap	=	-0.34f;
	}		
	
}

void Motor_Safe(void)//�㵹����
{
	if((Pitch_Pid.Location.Location > 0.4)||(Pitch_Pid.Location.Location < -0.4))
		motor_fall_flag = 0;
	else motor_fall_flag = 1;
}
