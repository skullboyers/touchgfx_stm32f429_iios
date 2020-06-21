/**
**************************************************************************************************
* @File		i2c.c
*
* @Brief	use gpio simulate i2c, all-powerful i2c driver
*
* @ChangeLogs:
*	Version		Date			Author				Notes
*	Ver0.1		2018-04-02		skullboyer
*	...
*
* COPYRIGHT (C) 2018-2028, skullboyer(skullboyer@qq.com)
*
**************************************************************************************************
*/
#include "i2c/bsp_i2c_gpio.h"
//#include "stm32f4xx_hal_rcc.h"

/**
 * @brief	i2c总线硬件配置
 * @{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{
 */
 


/* the second group info of i2c bus */
i2c_hw_s i2c_eeprom =
{
	GPIOC,
	GPIO_PORTC,
	GPIO_PIN_2,
	GPIOF,
	GPIO_PORTF,
	GPIO_PIN_12
};

/**
 * @}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}
 */


static void i2c_stop(i2c_hw_s i2c_sw);

void OpenGpioPortClock(uint8_t portNum)
{
    switch (portNum) {
        case GPIO_PORTA:
            __HAL_RCC_GPIOA_CLK_ENABLE();
            break;
        case GPIO_PORTB:
            __HAL_RCC_GPIOB_CLK_ENABLE();
            break;
        case GPIO_PORTC:
            __HAL_RCC_GPIOC_CLK_ENABLE();
            break;
        case GPIO_PORTD:
            __HAL_RCC_GPIOD_CLK_ENABLE();
            break;
        case GPIO_PORTE:
            __HAL_RCC_GPIOE_CLK_ENABLE();
            break;
        case GPIO_PORTF:
            __HAL_RCC_GPIOF_CLK_ENABLE();
            break;
        case GPIO_PORTG:
            __HAL_RCC_GPIOG_CLK_ENABLE();
            break;
        case GPIO_PORTH:
            __HAL_RCC_GPIOH_CLK_ENABLE();
            break;
        case GPIO_PORTI:
            __HAL_RCC_GPIOI_CLK_ENABLE();
            break;
        default:
            break;
    }
}
/**
**************************************************************************************************
*	@Brief		i2c硬件初始化	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*	@RetVal		void
*	@Notes		每使用一条i2c总线便调用该函数初始化该条总线的硬件信息
**************************************************************************************************
*/
void i2c_init(i2c_hw_s i2c_sw)
{
	GPIO_InitTypeDef gpioStruct;
	
	/* open io port clock */
	OpenGpioPortClock(i2c_sw.portClk);
	OpenGpioPortClock(i2c_sw.portData);
	
	/* config work mode */
	gpioStruct.Mode = GPIO_MODE_OUTPUT_OD;
	gpioStruct.Pull = GPIO_PULLUP;
	gpioStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	
	gpioStruct.Pin = i2c_sw.pin_data;
	HAL_GPIO_Init(i2c_sw.gpio_data, &gpioStruct);
	
	gpioStruct.Pin = i2c_sw.pin_clk;
	HAL_GPIO_Init(i2c_sw.gpio_clk, &gpioStruct);
	
	/* give a stop signal, reset all device on i2c-bus to standvy mode */
	i2c_stop(i2c_sw);
}

/**
**************************************************************************************************
*	@Brief		i2c时钟控制	
*	@Param		void
*	@RetVal		void
*	@Notes
**************************************************************************************************
*/
static void i2c_delay(void)
{
	uint8_t t =0;
	/* plese refer to armfly-V6 i2c driver */
	/*
		use an oscilloscope to observe the duration
	*/
	for (t = 0; t < 30; t++);
	
}

/**
**************************************************************************************************
*	@Brief		简单的毫秒延时	
*	@Param		void
*	@RetVal		void
*	@Notes		原理为：认为实现一条语句耗时1个system-clock
**************************************************************************************************
*/
#if 0
void delay_ms(uint16_t time)
{
	uint16_t i, j, k;
	
	for(i =0; i <time; i++)
	{
		for(j =0; j <1000; j++)
		{
			for(k =0; k <180; k++)
			{
			}
		}
	}
}
#endif

/**
**************************************************************************************************
*	@Brief		总线启动信号	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*	@RetVal		void
*	@Notes
**************************************************************************************************
*/
static void i2c_start(i2c_hw_s i2c_sw)
{
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_SET);
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_SET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_RESET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_RESET);
	i2c_delay();
}

/**
**************************************************************************************************
*	@Brief		总线停止信号	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*	@RetVal		void
*	@Notes
**************************************************************************************************
*/
static void i2c_stop(i2c_hw_s i2c_sw)
{
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_SET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_SET);
	i2c_delay();
}

/**
**************************************************************************************************
*	@Brief		等待 ACK	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*	@RetVal		0: OK, 1: ERROR
*	@Notes
**************************************************************************************************
*/
static uint8_t i2c_wait_ack(i2c_hw_s i2c_sw)
{
	uint8_t error_time =0;
	
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_SET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_SET);
	i2c_delay();
	
	while(HAL_GPIO_ReadPin(i2c_sw.gpio_data, i2c_sw.pin_data))
	{
		if(250 < error_time++)
		{
			i2c_stop(i2c_sw);
			return 1;
		}
	}
	
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_RESET);
	i2c_delay();
	
	return 0;
}

/**
**************************************************************************************************
*	@Brief		发送 ACK	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*	@RetVal		void
*	@Notes
**************************************************************************************************
*/
static void i2c_ack(i2c_hw_s i2c_sw)
{
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_RESET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_SET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_RESET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_SET);
}

/**
**************************************************************************************************
*	@Brief		发送 nACK	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*	@RetVal		void
*	@Notes
**************************************************************************************************
*/
static void i2c_nack(i2c_hw_s i2c_sw)
{
	HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data,GPIO_PIN_SET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_SET);
	i2c_delay();
	HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_RESET);
	i2c_delay();
}

/**
**************************************************************************************************
*	@Brief		写一字节数据	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*				@data  : 是否发送ACK标志
*	@RetVal		void
*	@Notes
**************************************************************************************************
*/
static void i2c_send_byte(i2c_hw_s i2c_sw, uint8_t data)
{
	uint8_t i =0;

	for (i = 0; i < 8; i++)
	{
		if(data & 0x80)
		{
			HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_RESET);
		}
		
		i2c_delay();
		HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_SET);
		i2c_delay();
		HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_RESET);
		
		if(7 == i)
		{
			HAL_GPIO_WritePin(i2c_sw.gpio_data, i2c_sw.pin_data, GPIO_PIN_SET);	/* release data bus */
		}
		
		data <<= 1;
		i2c_delay();
	}
}

/**
**************************************************************************************************
*	@Brief		读一字节数据	
*	@Param	
*				@i2c_sw: i2c总线硬件信息
*				@ack   : 是否发送ACK标志
*	@RetVal		读到的数据
*	@Notes
**************************************************************************************************
*/
static uint8_t i2c_read_byte(i2c_hw_s i2c_sw, ACK_enum ack)
{
	uint8_t i =0;
	uint8_t data =0;
	
	for(i = 0; i < 8; i++)
	{
		data <<= 1;
		HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_SET);
		i2c_delay();
		
		if(HAL_GPIO_ReadPin(i2c_sw.gpio_data, i2c_sw.pin_data))
		{
			data++;
		}
		
		HAL_GPIO_WritePin(i2c_sw.gpio_clk, i2c_sw.pin_clk, GPIO_PIN_RESET);
		i2c_delay();
	}
    
    if(ack == nACK)
	{
		i2c_nack(i2c_sw);
	}
	else
	{
		i2c_ack(i2c_sw);
	}
	
	return data;
}

/**
**************************************************************************************************
*	@Brief		检查器件能否正确访问	
*	@Param	
*				@i2c_sw : i2c总线硬件信息
*				@regAddr: 器件地址
*	@RetVal		0: OK, 1: ERROR
*	@Notes		可任意操作一读写寄存器，对比写入与读出的数据是否一致
**************************************************************************************************
*/
uint8_t i2c_check_device(i2c_hw_s i2c_sw, uint8_t regAddr)
{
	uint8_t res = 0;
	
	/* detect data line and clock line was released, i2c-bus don't use(*own understand*) */
	if (HAL_GPIO_ReadPin(i2c_sw.gpio_data, i2c_sw.pin_data) && HAL_GPIO_ReadPin(i2c_sw.gpio_clk, i2c_sw.pin_clk))
	{
		i2c_start(i2c_sw);
		
		i2c_send_byte(i2c_sw, regAddr);
		res = i2c_wait_ack(i2c_sw);
		
		i2c_stop(i2c_sw);
	}
	
	return res;		/* i2c-bus abnormal */
}

/**
**************************************************************************************************
*	@Brief		写入指定器件的指定寄存器的指定长度数据	
*	@Param	
*				@i2c_sw : i2c总线硬件信息
*				@addr   : 器件地址
*				@regAddr: 寄存器地址
*				@buf    : 数据缓存器
*				@len    : 要写入的数据长度
*	@RetVal		0: OK, others: ERROR
*	@Notes
**************************************************************************************************
*/
uint8_t i2c_write_package(i2c_hw_s i2c_sw, uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len)
{
    i2c_start(i2c_sw);
    i2c_send_byte(i2c_sw, devAddr);	/* device address + write command */
    if (1 == i2c_wait_ack(i2c_sw)) {
        i2c_stop(i2c_sw);
        return 1;
    }

    i2c_send_byte(i2c_sw, regAddr);
    i2c_wait_ack(i2c_sw);

    while (len--) {
        i2c_send_byte(i2c_sw, *buf++);
        if (1 == i2c_wait_ack(i2c_sw)) {
            i2c_stop(i2c_sw);
            return 1;
        }
    }

    i2c_stop(i2c_sw);

    return 0;
}

/**
**************************************************************************************************
*	@Brief		读取指定器件的指定寄存器的指定长度数据	
*	@Param	
*				@i2c_sw : i2c总线硬件信息
*				@addr   : 器件地址
*				@regAddr: 寄存器地址
*				@buf    : 数据缓存器
*				@len    : 要读取的数据长度
*	@RetVal		0: OK, others: ERROR
*	@Notes
**************************************************************************************************
*/
uint8_t i2c_read_package(i2c_hw_s i2c_sw, uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len)
{
    i2c_start(i2c_sw);

    i2c_send_byte(i2c_sw, devAddr); // (devAddr<<1) & ~(1<<0)); /* device address + write command */
    if (1 == i2c_wait_ack(i2c_sw)) {
        i2c_stop(i2c_sw);
        return 1;
    }

    i2c_send_byte(i2c_sw, regAddr);
    i2c_wait_ack(i2c_sw);

    i2c_start(i2c_sw);
    i2c_send_byte(i2c_sw, devAddr + 1); // (devAddr<<1) | 1<<0);
    i2c_wait_ack(i2c_sw);

    while (len--) {
        if (0 == len) {
            *buf = i2c_read_byte(i2c_sw, nACK);
        } else {
            *buf++ = i2c_read_byte(i2c_sw, ACK);
        }
    }

    i2c_stop(i2c_sw);

    return 0;
}




/******************************************^_^\__END__/^_^****************************************/


