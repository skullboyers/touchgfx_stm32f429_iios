/**
**************************************************************************************************
* @File		i2c.h
*
* @Brief	some definitions of using macros and variables for source files, and function 
*			declarations
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
#ifndef __I2C_GPIO_H__
#define __I2C_GPIO_H__

#include "stm32f4xx_hal.h"
#include "stdint.h"
//#include "stm32f4xx_hal_gpio.h"



typedef enum{nACK =0, ACK =1} ACK_enum;

typedef enum {
    GPIO_PORTA = 1,
    GPIO_PORTB,
    GPIO_PORTC,
    GPIO_PORTD,
    GPIO_PORTE,
    GPIO_PORTF,
    GPIO_PORTG,
    GPIO_PORTH,
    GPIO_PORTI,
} PortNumber;
    

/* i2c pin configure */
typedef struct
{
	GPIO_TypeDef *gpio_data;	/* i2c总线的SDA管脚所属的IO端口 */
	PortNumber portData;		/* i2c总线的SDA管脚所属的IO端口时钟 */
	uint16_t pin_data;			/* i2c总线的SDA管脚 */
	
	GPIO_TypeDef *gpio_clk;
	PortNumber portClk;
	uint16_t pin_clk;	
}i2c_hw_s;

/* variable declaration */
extern i2c_hw_s i2c_tp;
extern i2c_hw_s i2c_eeprom;


/* function declaration */
void i2c_init(i2c_hw_s i2c_sw);
uint8_t i2c_check_device(i2c_hw_s i2c_sw, uint8_t address);
uint8_t i2c_write_package(i2c_hw_s i2c_sw, uint8_t DevAddr, uint8_t regAddr, uint8_t *buf, uint16_t len);
uint8_t i2c_read_package(i2c_hw_s i2c_sw, uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint16_t len);

//void delay_ms(uint16_t);
	
#endif

/******************************************^_^\__END__/^_^****************************************/

