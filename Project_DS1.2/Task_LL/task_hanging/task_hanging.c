#include "task_hanging.h"
#include "driver_hanging.h"
#include "usart.h"
//-----significant variable--------//
extern DuojiStruct Duo[4]; //����ǰ�� 0��ǰ�� 1������ 2������ 3��ǰ��
extern RollPIDStruct Roll_Pid;
int mode_flag=0;           //ģʽ��־λ 0:��λ  1:��λ  2����λrollƽ��  3����λrollƽ��
int yuyin_flag = 0;
Con_DuoStruct Duo_con[4],Duo_CON;
extern uint8_t RX;//���ڿ���
uint8_t rx;//��������
//---------------------------------//

void Task_hanging(void)           //��������ܿ�
{
	if (huart3.Instance->DR == 104)
			RX='f';
	
	if(RX=='e'||RX== 0)       //��λ       
		mode_flag = 0;
	else if(RX=='f')           
		mode_flag = 1;
	else if(RX=='g')
		mode_flag = 2;		

	Hang_init_enable();
	HangingDriver();

}	

void HangingDriver(void)        	   //�ײ���������
{
	for(int i=0;i<4;i++) 
	{
		Duo_con[i].con_flag =  Duo_CON.con_flag;
		Duo_con[i].con_location = Duo_CON.con_location;
		Hanging_Datainput(&Duo[i],&Roll_Pid,Duo_con[i]);
	}
	Hanging_PID(&Roll_Pid);
	for(int i=0;i<4;i++) Hanging_Run(&Duo[i],&Roll_Pid);
	
}

void Hang_init_enable(void)        //��ʼ��ʹ��
{
	Duo_CON.con_flag = mode_flag;
	Duo_CON.con_location = 0;
}	
