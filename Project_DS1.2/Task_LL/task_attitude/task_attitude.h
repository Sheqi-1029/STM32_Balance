#ifndef __TASK_MPU6050_H_
#define __TASK_MPU6050_H_

typedef struct{
	// ŷ���Ƕ�
	float pitch;
	float yaw;
	float roll;
}MPU6050_ANGLETYPE;

extern MPU6050_ANGLETYPE MPU6050_Angle; 

//��̬�������
void Task_attitude_sol(void);             //��̬���㺯��
void accel_attitude_aol(void);            //���ٶ���̬����
void gyro_attitude_aol(void);             //�����ǽ��ٶȽ���
void zero_sol(void);                      //��ƫ����
void WM_locaupdate(void);              //����λ�ø���

//���������
void Attitude_kalman_init(void);            //������������ʼ��
void Attitude_kalman_pitch_dispose(void);   //pitch�������˲������ݴ���
void Attitude_kalman_roll_dispose(void);    //roll�������˲������ݴ���

//J-scope �۲�
void Attitude_observe_jscope();               	//J-scope �۲����ݴ���

#endif