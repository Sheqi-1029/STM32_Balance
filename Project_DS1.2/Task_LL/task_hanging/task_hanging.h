#ifndef __TASK_HANGING_H_
#define __TASK_HANGING_H_


//�ܿ����
void Task_hanging(void);           //��������ܿ�
void HangingDriver(void);         	   //�ײ���������
void Hang_init_enable(void);        //��ʼ��ʹ��    

//�������
void Hang_Low(void);
void Hang_High(void);


//ģʽ�������
void Hang_Lost_Run(void); 			 //�ؿ�ģʽ
void Hang_Park_Run(void);        //ƽ��ֹͣģʽ
void Hang_Bluetooth_Run(void);   //��������ģʽ
void Hang_Yuyin_Run(void);       //��������ģʽ




#endif