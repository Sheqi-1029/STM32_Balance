#include "driver_mpu6050.h"
#include "driver_i2cunlock.h"

#define ReBus 0

static int16_t Mpu6050Addr = 0xD0;
MPU6050DATATYPE Mpu6050_Data;
int8_t Accel_flag,Gyro_flag,Temp_flag;

int8_t Sensor_I2C1_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t DataLen,uint8_t *oData)
{
	return HAL_I2C_Mem_Read(&hi2c1,DevAddr,MemAddr,I2C_MEMADD_SIZE_8BIT,oData,DataLen,200);
}//��������װ

int8_t Sensor_I2C1_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t DataLen,uint8_t *iData)
{
	return HAL_I2C_Mem_Write(&hi2c1,DevAddr,MemAddr,I2C_MEMADD_SIZE_8BIT,iData,DataLen,200);
}//д������װ

int16_t Sensor_I2C1_Serch(void)//̽ѰĿ���ַ
{
	for(uint8_t i = 1; i < 255; i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c1, i, 1, 200) == HAL_OK)
		{
			Mpu6050Addr = i;
			return i;
		}
	}
	return 0xD0;
}


int8_t MPU6050_Init(int16_t Addr)//��ʼ������
{
	HAL_Delay(500);
	uint8_t check;
	HAL_I2C_Mem_Read(&hi2c1,Addr,WHO_AM_I,I2C_MEMADD_SIZE_8BIT,&check,1,200);
	if(check == 0x68) // ȷ���豸�� ��ַ�Ĵ���
	{	
		check = 0x00;
		Sensor_I2C1_Write(Addr,PWR_MGMT_1, 1,&check); 	    // ����
		check = 0x07;	
		Sensor_I2C1_Write(Addr,SMPLRT_DIV, 1,&check);	    // 1Khz������
		check = 0x00;
		Sensor_I2C1_Write(Addr,ACCEL_CONFIG, 1,&check);	 	// ���ٶ�����
		check = 0x00;
		Sensor_I2C1_Write(Addr,GYRO_CONFIG, 1,&check);		// ��������
		return 0;
	}
	return -1;
}

void MPU6050_Read_Accel(void)//���ٶȶ�ȡ
{
	uint8_t Read_Buf[6];
	
	// �Ĵ��������Ǽ��ٶ�X�� - ���ٶ�X�� - ���ٶ�Y��λ - ���ٶ�Y��λ - ���ٶ�Z��λ - ���ٶȶ�Z��λ
	Accel_flag = Sensor_I2C1_Read(Mpu6050Addr, ACCEL_XOUT_H, 6, Read_Buf); 
	
	Mpu6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Accel_X = Mpu6050_Data.Accel_X / 16384.0f;
	Mpu6050_Data.Accel_Y = Mpu6050_Data.Accel_Y / 16384.0f;
	Mpu6050_Data.Accel_Z = Mpu6050_Data.Accel_Z / 16384.0f;
	
}
void MPU6050_Read_Gyro(void)//�����ǣ����ٶȣ���ȡ
{
	uint8_t Read_Buf[6];
	
	// �Ĵ��������ǽǶ�X�� - �Ƕ�X�� - �Ƕ�Y��λ - �Ƕ�Y��λ - �Ƕ�Z��λ - �Ƕ�Z��λ
	Gyro_flag = Sensor_I2C1_Read(Mpu6050Addr, GYRO_XOUT_H, 6, Read_Buf); 
	
	Mpu6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Gyro_X = Mpu6050_Data.Gyro_X / 131.0f;
	Mpu6050_Data.Gyro_Y = Mpu6050_Data.Gyro_Y / 131.0f;
	Mpu6050_Data.Gyro_Z = Mpu6050_Data.Gyro_Z / 131.0f;
	
}
void MPU6050_Read_Temp(void)//�¶ȶ�ȡ
{
    uint8_t Read_Buf[2];
	
	Temp_flag = Sensor_I2C1_Read(Mpu6050Addr, TEMP_OUT_H, 2, Read_Buf); 
	
	Mpu6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050_Data.Temp = 36.53f + (Mpu6050_Data.Temp / 340.0f);
}

void MPU6050_DATA_Read(void) //�����ܶ�ȡ
{
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
	MPU6050_Read_Temp();
	
	#if ReBus
		I2CResetBus();
	#endif
}	







