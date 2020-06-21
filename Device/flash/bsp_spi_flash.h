#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"
//#include "stm32f4xx_hal_gpio.h"
//#include "stm32f4xx_hal_spi.h"
//#include "stm32f4xx_hal_rcc.h"

/* Private typedef -----------------------------------------------------------*/
//#define  sFLASH_ID                       0xEF3015     //W25X16
//#define  sFLASH_ID                       0xEF4015	    //W25Q16
//#define  sFLASH_ID                        0XEF4017     //W25Q64
#define  sFLASH_ID                       0XEF4018     //W25Q128


//#define SPI_FLASH_PageSize            4096
#define SPI_FLASH_PageSize              256
#define SPI_FLASH_PerWritePageSize      256
    
#define EXFLASH_BASEADDR                0x90000000
#define EXFLASH_ENDADDR                 0x91000000

/* Private define ------------------------------------------------------------*/
/*命令定义-开头*******************************/
#define W25X_WriteEnable                0x06
#define W25X_WriteDisable               0x04
#define W25X_ReadStatusReg              0x05
#define W25X_WriteStatusReg	            0x01
#define W25X_ReadData                   0x03
#define W25X_FastReadData               0x0B
#define W25X_FastReadDual               0x3B
#define W25X_PageProgram                0x02
#define W25X_BlockErase                 0xD8
#define W25X_SectorErase                0x20
#define W25X_ChipErase                  0xC7
#define W25X_PowerDown                  0xB9
#define W25X_ReleasePowerDown           0xAB
#define W25X_DeviceID                   0xAB
#define W25X_ManufactDeviceID           0x90
#define W25X_JedecDeviceID              0x9F

#define WIP_Flag                        0x01 /* Write In Progress (WIP) flag */
#define Dummy_Byte                      0xFF
/*命令定义-结尾*******************************/


/*SPI接口定义-开头****************************/
#define FLASH_SPI                           SPI5

#define FLASH_SPI_SCK_PIN                   GPIO_PIN_7
#define FLASH_SPI_SCK_GPIO_PORT             GPIOF

#define FLASH_SPI_MISO_PIN                  GPIO_PIN_8
#define FLASH_SPI_MISO_GPIO_PORT            GPIOF

#define FLASH_SPI_MOSI_PIN                  GPIO_PIN_9
#define FLASH_SPI_MOSI_GPIO_PORT            GPIOF

#define FLASH_CS_PIN                        GPIO_PIN_6
#define FLASH_CS_GPIO_PORT                  GPIOF

#define SPI_FLASH_CS_LOW()      {FLASH_CS_GPIO_PORT->BSRRH=FLASH_CS_PIN;}
#define SPI_FLASH_CS_HIGH()     {FLASH_CS_GPIO_PORT->BSRRL=FLASH_CS_PIN;}
/*SPI接口定义-结尾****************************/

/*等待超时时间*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

/*信息输出*/
#define FLASH_DEBUG_ON         1

#define FLASH_INFO(fmt,arg...)           printf("<<-FLASH-INFO->> "fmt"\n",##arg)
#define FLASH_ERROR(fmt,arg...)          printf("<<-FLASH-ERROR->> "fmt"\n",##arg)
#define FLASH_DEBUG(fmt,arg...)          do{\
                                          if(FLASH_DEBUG_ON)\
                                          printf("<<-FLASH-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)



void SPI_FLASH_Init(void);
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t SPI_FLASH_ReadID(void);
uint32_t SPI_FLASH_ReadDeviceID(void);
uint32_t SPI_FLASH_ReadManufactDeviceID(void);
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);


uint8_t SPI_FLASH_ReadByte(void);
uint8_t SPI_FLASH_SendByte(uint8_t byte);
uint16_t SPI_FLASH_SendHalfWord(uint16_t HalfWord);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);

void SpiReadFlash(uint8_t* buffer, uint32_t readAddr, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_FLASH_H */

