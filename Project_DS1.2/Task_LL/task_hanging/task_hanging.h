#ifndef __TASK_HANGING_H_
#define __TASK_HANGING_H_


//总控相关
void Task_hanging(void);           //舵机悬挂总控
void HangingDriver(void);         	   //底层驱动连接
void Hang_init_enable(void);        //初始化使能    

//运行相关
void Hang_Low(void);
void Hang_High(void);


//模式驱动相关
void Hang_Lost_Run(void); 			 //关控模式
void Hang_Park_Run(void);        //平衡停止模式
void Hang_Bluetooth_Run(void);   //蓝牙控制模式
void Hang_Yuyin_Run(void);       //语音控制模式




#endif