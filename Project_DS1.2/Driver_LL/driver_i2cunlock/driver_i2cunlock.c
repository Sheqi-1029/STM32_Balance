#include "driver_i2cunlock.h"

static void User_I2C2_GeneralPurposeOutput_Init(I2C_HandleTypeDef* i2cHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if(i2cHandle->Instance==I2C2)
	{
		/*   PB10     ------> I2C2_SCL; PB11     ------> I2C2_SDA */
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}


static void User_I2C2_AlternateFunction_Init(I2C_HandleTypeDef* i2cHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if(i2cHandle->Instance==I2C2)
	{
		/*   PB10     ------> I2C2_SCL; PB11     ------> I2C2_SDA */
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	}
}

HAL_StatusTypeDef I2CResetBus(void)
{
	hi2c1.ErrorCode = HAL_I2C_ERROR_AF;
	/* 1. Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register */
	__HAL_I2C_DISABLE(&hi2c1);
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
	
	/* 2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR) */
	User_I2C2_GeneralPurposeOutput_Init(&hi2c1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);
	
	/* 3. Check SCL and SDA High level in GPIOx_IDR */
	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET)||(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_SET))
	{

		return HAL_ERROR;
	}
	
	/* 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	 * 5. Check SDA Low level in GPIOx_IDR.
	 * 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR)
	 * 7. Check SCL Low level in GPIOx_IDR.
	 * */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_Delay(1);
	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_RESET)||(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_RESET))
	{

		return HAL_ERROR;
	}
	
	/*
	 * 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	 * 9. Check SCL High level in GPIOx_IDR.
	 * 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	 * 11. Check SDA High level in GPIOx_IDR.
	 */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);
	HAL_Delay(1);
	if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) != GPIO_PIN_SET)||(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) != GPIO_PIN_SET))
	{

		return HAL_ERROR;
	}
	
	/* 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain. */
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
	User_I2C2_AlternateFunction_Init(&hi2c1);
	
	/* 13. Set SWRST bit in I2Cx_CR1 register. */
	hi2c1.Instance->CR1 |=  I2C_CR1_SWRST;
	HAL_Delay(2);
	/* 14. Clear SWRST bit in I2Cx_CR1 register. */
	hi2c1.Instance->CR1 &=  ~I2C_CR1_SWRST;
	HAL_Delay(2);
	/* 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register */
	MX_I2C1_Init();
	__HAL_I2C_ENABLE(&hi2c1);
	HAL_Delay(2);

	hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
	hi2c1.State = HAL_I2C_STATE_READY;
	hi2c1.PreviousState = (uint32_t)HAL_I2C_MODE_NONE;
	hi2c1.Mode = HAL_I2C_MODE_NONE;
	return HAL_OK;
}
