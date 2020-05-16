/**
***********************************************************************************************************************
* @File		touch source file
*
* @Brief	use gpio simulate i2c, all-powerful i2c driver
*
* @ChangeLogs:
*	Version		Date			Author				Notes
*	V0.1		2020-05-12		skull
*	...
*
* COPYRIGHT (C) 2020 - 2030, skull(skullboyer@qq.com)
*
***********************************************************************************************************************
*/
#include "touch_ft5x06.h"
#include "i2c/bsp_i2c_gpio.h"


Ft5x06Manager g_TouchManger;

/* the touch info of i2c bus */
i2c_hw_s i2c_tp =
{
	GPIOD,
	GPIO_PORTD,
	GPIO_PIN_5,
	GPIOD,
	GPIO_PORTD,
	GPIO_PIN_4
};


static void Ft5x06_GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin = TP_RST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TP_RST_PORT, &GPIO_InitStruct);

    /*Configure GPIO pin : TP_INT_Pin */
    GPIO_InitStruct.Pin = TP_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(TP_INT_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    
    i2c_init(i2c_tp);
    g_TouchManger.Enable = 1;    
    
    HAL_GPIO_WritePin(TP_RST_PORT, TP_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(20);    
    HAL_GPIO_WritePin(TP_RST_PORT, TP_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(50);
    
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

uint8_t Ft5x06_IsTouch(void)
{
    if (HAL_GPIO_ReadPin(TP_INT_PORT, TP_INT_PIN) == GPIO_PIN_SET) {
        return 1;
    }
    return 0;
}

static uint8_t Ft5x06_ReadReg(uint16_t addr, uint8_t *buf, uint8_t len)
{
    return i2c_read_package(i2c_tp, FT5X06_I2C_ADDR, addr, buf, len);
}

static uint8_t Ft5x06_WriteReg(uint16_t addr, uint8_t *buf, uint8_t len)
{
    return i2c_write_package(i2c_tp, FT5X06_I2C_ADDR, addr, buf, len);
}


void Ft5x06_TouchProcess(void)
{  
    uint8_t buf[CFG_POINT_READ_BUF];
	uint8_t i;
	uint16_t x, y;

	if (g_TouchManger.Enable == 0) {
		return;
	}
	
	if (Ft5x06_IsTouch() == 0) {
		return;
	}
	/* TODO: */
//	Ft5x06_ReadReg(2, buf, 1); // TD_STATUS
//    /* 判断是否有触摸数据 */	
//	if ((buf[0] & 0x07) == 0) {		
//		return;
//	}
	
	/* 有触摸，读取完整的数据，这里读取了一次 */
	Ft5x06_ReadReg(TS_DEVICE_MODE, buf, CFG_POINT_READ_BUF);
	for (i = 0; i < FT5X06_TOUCH_POINTS; i++) {
		uint8_t pointid;
		
		pointid = (buf[5 + 6*i]) >> 4;
		if (pointid >= 0x0f) {
			break;
		} else {
        	g_TouchManger.X[i] = (int16_t)(buf[3 + 6*i] & 0x0F)<<8 | (int16_t)buf[4 + 6*i];
        	g_TouchManger.Y[i] = (int16_t)(buf[5 + 6*i] & 0x0F)<<8 | (int16_t)buf[6 + 6*i];
        	g_TouchManger.Event[i] = buf[0x3 + 6*i] >> 6;
        	g_TouchManger.id[i] = (buf[5 + 6*i])>>4;
    	}
    }

	/* 检测按下 */
	if ((g_TouchManger.ChipID == 0x55)||(g_TouchManger.ChipID == 0xa3))       /* 4.3寸 480 * 272 */
	{
		x = g_TouchManger.Y[0];
		y = g_TouchManger.X[0];	
		
		/* 判断值域 */
		if (x > 479)
		{
			x = 479;
		}
		
		if (y > 271)
		{
			y = 271;
		}			
	}
	else if (g_TouchManger.ChipID == 0x0A)	/* 5.0寸 800 * 480 */
	{
		x = g_TouchManger.X[0];
		y = g_TouchManger.Y[0];	
		
		/* 判断值域 */
		if (x > 799)
		{
			x = 799;
		}			
		if (y > 479)
		{
			y = 479;
		}			
	}
    else if (g_TouchManger.ChipID == 0x54)	/* 7.0寸 1024 * 600 */
	{
		x = g_TouchManger.X[0];
		y = g_TouchManger.Y[0];	
		
		/* 判断值域 */
		if (x > 1023)
		{
			x = 1023;
		}			
		if (y > 599)
		{
			y = 599;
		}			
	}
	else	/* id == 0x06 表示7寸电容屏（FT芯片） */
	{
		x = g_TouchManger.Y[0];
		y = g_TouchManger.X[0];	
		
		/* 判断值域 */
		if (x > 799)
		{
			x = 799;
		}			
		if (y > 479)
		{
			y = 479;
		}			
	}	
}

uint8_t FT5X06_ReadID(void)
{
	uint8_t id;
    uint8_t ret;
	
	ret = Ft5x06_ReadReg(FTS_REG_CHIP_ID, &id, 1);
    if (ret == 1) {
        return 0xFF;
    }
	
	g_TouchManger.ChipID = id;		/* 保存id */
	return id;
}

uint8_t touchBuf[2];
static void Ft5x06_ConfigParam(void)
{
    
    touchBuf[0] = 0;
    Ft5x06_WriteReg(0x00, touchBuf, 1);
    Ft5x06_WriteReg(0xA4, touchBuf, 1);
    touchBuf[0] = 22;
    Ft5x06_WriteReg(0x80, touchBuf, 1);
    touchBuf[0] = 22;
    Ft5x06_WriteReg(0x88, touchBuf, 1);
    Ft5x06_ReadReg(0xA1, touchBuf, 2);
}    

void TouchPanelInit(void)
{
    Ft5x06_GpioInit();
    Ft5x06_ConfigParam();
}

void GetTouchInfo(void)
{    
    uint8_t count = 0;
    while (count < 5) {
        if (i2c_check_device(i2c_tp, FT5X06_I2C_ADDR) == 0) {
            FT5X06_ReadID();
        }
        HAL_Delay(10);
        count++;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case GPIO_PIN_6:
            Ft5x06_TouchProcess();
            break;
        default:
            break;
    }
}

/****************************************************^_^\__END__/^_^**************************************************/
