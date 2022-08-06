#include "task_hanging.h"
#include "driver_hanging.h"
#include "usart.h"
//-----significant variable--------//
extern DuojiStruct Duo[4]; //后往前看 0：前左 1：后左 2：后右 3：前右
extern RollPIDStruct Roll_Pid;
int mode_flag=0;           //模式标志位 0:低位  1:高位  2：低位roll平衡  3：高位roll平衡
int yuyin_flag = 0;
Con_DuoStruct Duo_con[4],Duo_CON;
extern uint8_t RX;//串口控制
uint8_t rx;//语音控制
//---------------------------------//

void Task_hanging(void)           //舵机悬挂总控
{
	if (huart3.Instance->DR == 104)
			RX='f';
	
	if(RX=='e'||RX== 0)       //复位       
		mode_flag = 0;
	else if(RX=='f')           
		mode_flag = 1;
	else if(RX=='g')
		mode_flag = 2;		

	Hang_init_enable();
	HangingDriver();

}	

void HangingDriver(void)        	   //底层驱动连接
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

void Hang_init_enable(void)        //初始化使能
{
	Duo_CON.con_flag = mode_flag;
	Duo_CON.con_location = 0;
}	
