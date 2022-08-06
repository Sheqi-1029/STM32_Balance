#include "driver_mpu6050.h"
#include "driver_i2cunlock.h"

#define ReBus 0

static int16_t Mpu6050Addr = 0xD0;
MPU6050DATATYPE Mpu6050_Data;
int8_t Accel_flag,Gyro_flag,Temp_flag;

int8_t Sensor_I2C1_Read(uint16_t DevAddr, uint16_t MemAddr, uint8_t DataLen,uint8_t *oData)
{
	return HAL_I2C_Mem_Read(&hi2c1,DevAddr,MemAddr,I2C_MEMADD_SIZE_8BIT,oData,DataLen,200);
}//读操作包装

int8_t Sensor_I2C1_Write(uint16_t DevAddr, uint16_t MemAddr, uint8_t DataLen,uint8_t *iData)
{
	return HAL_I2C_Mem_Write(&hi2c1,DevAddr,MemAddr,I2C_MEMADD_SIZE_8BIT,iData,DataLen,200);
}//写操作包装

int16_t Sensor_I2C1_Serch(void)//探寻目标地址
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


int8_t MPU6050_Init(int16_t Addr)//初始化操作
{
	HAL_Delay(500);
	uint8_t check;
	HAL_I2C_Mem_Read(&hi2c1,Addr,WHO_AM_I,I2C_MEMADD_SIZE_8BIT,&check,1,200);
	if(check == 0x68) // 确认设备用 地址寄存器
	{	
		check = 0x00;
		Sensor_I2C1_Write(Addr,PWR_MGMT_1, 1,&check); 	    // 唤醒
		check = 0x07;	
		Sensor_I2C1_Write(Addr,SMPLRT_DIV, 1,&check);	    // 1Khz的速率
		check = 0x00;
		Sensor_I2C1_Write(Addr,ACCEL_CONFIG, 1,&check);	 	// 加速度配置
		check = 0x00;
		Sensor_I2C1_Write(Addr,GYRO_CONFIG, 1,&check);		// 陀螺配置
		return 0;
	}
	return -1;
}

void MPU6050_Read_Accel(void)//加速度读取
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是加速度X高 - 加速度X低 - 加速度Y高位 - 加速度Y低位 - 加速度Z高位 - 加速度度Z低位
	Accel_flag = Sensor_I2C1_Read(Mpu6050Addr, ACCEL_XOUT_H, 6, Read_Buf); 
	
	Mpu6050_Data.Accel_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Accel_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Accel_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Accel_X = Mpu6050_Data.Accel_X / 16384.0f;
	Mpu6050_Data.Accel_Y = Mpu6050_Data.Accel_Y / 16384.0f;
	Mpu6050_Data.Accel_Z = Mpu6050_Data.Accel_Z / 16384.0f;
	
}
void MPU6050_Read_Gyro(void)//陀螺仪（角速度）读取
{
	uint8_t Read_Buf[6];
	
	// 寄存器依次是角度X高 - 角度X低 - 角度Y高位 - 角度Y低位 - 角度Z高位 - 角度Z低位
	Gyro_flag = Sensor_I2C1_Read(Mpu6050Addr, GYRO_XOUT_H, 6, Read_Buf); 
	
	Mpu6050_Data.Gyro_X = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	Mpu6050_Data.Gyro_Y = (int16_t)(Read_Buf[2] << 8 | Read_Buf[3]);
	Mpu6050_Data.Gyro_Z = (int16_t)(Read_Buf[4] << 8 | Read_Buf[5]);
	
	Mpu6050_Data.Gyro_X = Mpu6050_Data.Gyro_X / 131.0f;
	Mpu6050_Data.Gyro_Y = Mpu6050_Data.Gyro_Y / 131.0f;
	Mpu6050_Data.Gyro_Z = Mpu6050_Data.Gyro_Z / 131.0f;
	
}
void MPU6050_Read_Temp(void)//温度读取
{
    uint8_t Read_Buf[2];
	
	Temp_flag = Sensor_I2C1_Read(Mpu6050Addr, TEMP_OUT_H, 2, Read_Buf); 
	
	Mpu6050_Data.Temp = (int16_t)(Read_Buf[0] << 8 | Read_Buf[1]);
	
	Mpu6050_Data.Temp = 36.53f + (Mpu6050_Data.Temp / 340.0f);
}

void MPU6050_DATA_Read(void) //数据总读取
{
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
	MPU6050_Read_Temp();
	
	#if ReBus
		I2CResetBus();
	#endif
}	







