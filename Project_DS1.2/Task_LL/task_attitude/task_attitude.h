#ifndef __TASK_MPU6050_H_
#define __TASK_MPU6050_H_

typedef struct{
	// 欧拉角度
	float pitch;
	float yaw;
	float roll;
}MPU6050_ANGLETYPE;

extern MPU6050_ANGLETYPE MPU6050_Angle; 

//姿态解算相关
void Task_attitude_sol(void);             //姿态解算函数
void accel_attitude_aol(void);            //角速度姿态解算
void gyro_attitude_aol(void);             //陀螺仪角速度解算
void zero_sol(void);                      //零偏处理
void WM_locaupdate(void);              //驱动位置更新

//卡尔曼相关
void Attitude_kalman_init(void);            //卡尔曼参数初始化
void Attitude_kalman_pitch_dispose(void);   //pitch卡尔曼滤波器数据处理
void Attitude_kalman_roll_dispose(void);    //roll卡尔曼滤波器数据处理

//J-scope 观测
void Attitude_observe_jscope();               	//J-scope 观测数据处理

#endif