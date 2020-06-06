 /**
  ******************************************************************************
  * @file    bsp_spi_flash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   spi flash 底层应用函数bsp 
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火STM32 F429 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
#include ".\flash\bsp_spi_flash.h"
#include <string.h>

#define W25Q_SPI_CS_Pin GPIO_PIN_6
#define W25Q_SPI_CS_GPIO_Port GPIOF


uint32_t flashId;
uint32_t deviceId;
uint32_t manufactId;

static __IO uint32_t SPITimeout = SPIT_LONG_TIMEOUT;
extern SPI_HandleTypeDef hspi5;

static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);
extern void MX_SPI5_Init(void);


 /**
  * @brief  SPI_FLASH初始化
  * @param  无
  * @retval 无
  */
    uint8_t readData[50] = {0};
void SPI_FLASH_Init(void)
{
//    uint8_t checkBuffer[] = {0x1A, 0x2B, 0x5E, 0x9F};
    
    MX_SPI5_Init();    
    SPI_Flash_WAKEUP();
    manufactId = SPI_FLASH_ReadManufactDeviceID();
    deviceId = SPI_FLASH_ReadDeviceID();
    flashId = SPI_FLASH_ReadID();
//    SPI_FLASH_SectorErase(0x0);
////    SPI_FLASH_PageWrite(checkBuffer, 0x0, sizeof(checkBuffer));
//    SPI_FLASH_BufferWrite(checkBuffer, 0x0, sizeof(checkBuffer));
//    memset(checkBuffer, 0, sizeof(checkBuffer));
//    SPI_FLASH_BufferRead(checkBuffer, 0x0, sizeof(checkBuffer));
    SPI_FLASH_BufferRead(readData, 0x0, sizeof(readData));
}

 /**
  * @brief  擦除FLASH扇区
  * @param  SectorAddr：要擦除的扇区地址
  * @retval 无
  */
void SPI_FLASH_SectorErase(uint32_t SectorAddr)
{
    uint8_t txData[4];
    
    txData[0] = W25X_SectorErase;
    txData[1] = (SectorAddr & 0xFF0000) >> 16;
    txData[2] = (SectorAddr & 0xFF00) >> 8;
    txData[3] = SectorAddr & 0xFF;

    SPI_FLASH_WriteEnable();
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

//     HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000000);
     HAL_SPI_TransmitReceive(&hspi5, txData, NULL, sizeof(txData), 1000000);

    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);

    SPI_FLASH_WaitForWriteEnd();
}


 /**
  * @brief  擦除FLASH扇区，整片擦除
  * @param  无
  * @retval 无
  */
void SPI_FLASH_BulkErase(void)
{
    /* 发送FLASH写使能命令 */
    SPI_FLASH_WriteEnable();
    
    /* 整块 Erase */
    /* 选择FLASH: CS低电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    /* 发送整块擦除指令*/
    SPI_FLASH_SendByte(W25X_ChipErase);
    /* 停止信号 FLASH: CS 高电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
    
    /* 等待擦除完毕*/
    SPI_FLASH_WaitForWriteEnd();
}



 /**
  * @brief  对FLASH按页写入数据，调用本函数写入数据前需要先擦除扇区
  * @param	pBuffer，要写入数据的指针
  * @param WriteAddr，写入地址
  * @param  NumByteToWrite，写入数据长度，必须小于等于SPI_FLASH_PerWritePageSize
  * @retval 无
  */
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t txData[4];
 
    txData[0] = W25X_PageProgram;
    txData[1] = (WriteAddr & 0xFF0000) >> 16;
    txData[2] = (WriteAddr & 0xFF00) >> 8;
    txData[3] = WriteAddr & 0xFF;
    
    /* 发送FLASH写使能命令 */
    SPI_FLASH_WriteEnable();
    
    /* 选择FLASH: CS低电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
//    /* 写页写指令*/
//    SPI_FLASH_SendByte(W25X_PageProgram);
//    /*发送写地址的高位*/
//    SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
//    /*发送写地址的中位*/
//    SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
//    /*发送写地址数据传输的低位*/
//    SPI_FLASH_SendByte(WriteAddr & 0xFF);
    
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 100);
    
    if (NumByteToWrite > SPI_FLASH_PerWritePageSize) {
        NumByteToWrite = SPI_FLASH_PerWritePageSize;
//        FLASH_ERROR("SPI_FLASH_PageWrite too large!");
    }
    
//    /* 写入数据*/
//    while (NumByteToWrite--)
//    {
//        /* 发送当前要写入的字节数据 */
//        SPI_FLASH_SendByte(*pBuffer);
//        /* 指向下一字节数据 */
//        pBuffer++;
//    }
    
    HAL_SPI_Transmit(&hspi5, pBuffer, NumByteToWrite, 100);
    
    /* 停止信号 FLASH: CS 高电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
    
    /* 等待写入完毕*/
    SPI_FLASH_WaitForWriteEnd();
}


 /**
  * @brief  对FLASH写入数据，调用本函数写入数据前需要先擦除扇区
  * @param	pBuffer，要写入数据的指针
  * @param  WriteAddr，写入地址
  * @param  NumByteToWrite，写入数据长度
  * @retval 无
  */
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	
	/*mod运算求余，若writeAddr是SPI_FLASH_PageSize整数倍，运算结果Addr值为0*/
    Addr = WriteAddr % SPI_FLASH_PageSize;
	
	/*差count个数据值，刚好可以对齐到页地址*/
    count = SPI_FLASH_PageSize - Addr;	
	/*计算出要写多少整数页*/
    NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
	/*mod运算求余，计算出剩余不满一页的字节数*/
    NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

    /* Addr=0,则WriteAddr 刚好按页对齐 aligned  */
    if (Addr == 0) {
        /* NumByteToWrite < SPI_FLASH_PageSize */
        if (NumOfPage == 0) {
            SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
        } else {
            /* NumByteToWrite > SPI_FLASH_PageSize */
            /*先把整数页都写了*/
            while (NumOfPage--) {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
                WriteAddr +=  SPI_FLASH_PageSize;
                pBuffer += SPI_FLASH_PageSize;
            }
                
            /*若有多余的不满一页的数据，把它写完*/
            SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        }
    } else {
        /* 若地址与 SPI_FLASH_PageSize 不对齐  */
        /* NumByteToWrite < SPI_FLASH_PageSize */
        if (NumOfPage == 0)  {
            /*当前页剩余的count个位置比NumOfSingle小，写不完*/
            if (NumOfSingle > count) {
                temp = NumOfSingle - count;
                        
                /*先写满当前页*/
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
                WriteAddr +=  count;
                pBuffer += count;
                        
                /*再写剩余的数据*/
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
            } else {
                /*当前页剩余的count个位置能写完NumOfSingle个数据*/                
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
            }
        } else {
            /* NumByteToWrite > SPI_FLASH_PageSize */
            /*地址不对齐多出的count分开处理，不加入这个运算*/
            NumByteToWrite -= count;
            NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;
            NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;
        
            SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
            WriteAddr += count;
            pBuffer += count;
                
            /*把整数页都写了*/
            while (NumOfPage--) {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
                WriteAddr +=  SPI_FLASH_PageSize;
                pBuffer += SPI_FLASH_PageSize;
            }
                /*若有多余的不满一页的数据，把它写完*/
            if (NumOfSingle != 0) {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    }
}

/**
 * @brief    read data from flash
 * @note     data size change 32bit
 * @issue    SZIE_16_BIT = 0xFFFF 时，读取数据异常，analog clock widget 显示有3条脏数据图像，3条对应读取数据量为
 *           SZIE_16_BIT三次，猜测是spi读接口无法读取最大0xFFFF的数据量
 */
#define SZIE_16_BIT            0x7FFF
void SpiReadFlash(uint8_t* buffer, uint32_t readAddr, uint32_t size)
{
    for (uint16_t i = 0; i < (size / SZIE_16_BIT); i++) {
        SPI_FLASH_BufferRead(buffer, readAddr, SZIE_16_BIT);
        buffer += SZIE_16_BIT;
        readAddr += SZIE_16_BIT;
    }

    uint16_t residue = size % SZIE_16_BIT;
    if (residue > 0) {
        SPI_FLASH_BufferRead(buffer, readAddr, residue);
    }
}

void _SpiReadFlash(uint8_t* buffer, uint32_t readAddr, uint32_t size)
{
    uint8_t txData[4];

    txData[0] = W25X_ReadData;
    txData[1] = (readAddr & 0xFF0000) >> 16;
    txData[2] = (readAddr & 0xFF00) >> 8;
    txData[3] = readAddr & 0xFF;

    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 100);

    for (uint16_t i = 0; i < (size / SZIE_16_BIT); i++) {
        HAL_SPI_Receive(&hspi5, buffer, SZIE_16_BIT, 100);
        buffer += SZIE_16_BIT;
//        readAddr += SZIE_16_BIT;
    }

    uint16_t residue = size % SZIE_16_BIT;
    if (residue > 0) {
        HAL_SPI_Receive(&hspi5, buffer, residue, 100);
    }

    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}

 /**
  * @brief  读取FLASH数据
  * @param 	pBuffer，存储读出数据的指针
  * @param   ReadAddr，读取地址
  * @param   NumByteToRead，读取数据长度
  * @retval 无
  */
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint8_t txData[4];

    txData[0] = W25X_ReadData;
    txData[1] = (ReadAddr & 0xFF0000) >> 16;
    txData[2] = (ReadAddr & 0xFF00) >> 8;
    txData[3] = ReadAddr & 0xFF;
    
    /* 选择FLASH: CS低电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    
//    /* 发送 读 指令 */
//    SPI_FLASH_SendByte(W25X_ReadData);
//    
//    /* 发送 读 地址高位 */
//    SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
//    /* 发送 读 地址中位 */
//    SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
//    /* 发送 读 地址低位 */
//    SPI_FLASH_SendByte(ReadAddr & 0xFF);
    
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 100);
    
//    /* 读取数据 */
//    while (NumByteToRead--) {
//        /* 读取一个字节*/
//        *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
//        /* 指向下一个字节缓冲区 */
//        pBuffer++;
//    }
//    uint8_t* txDummy = ( uint8_t* )pvPortMalloc(NumByteToRead);
//    memset(txDummy, Dummy_Byte, NumByteToRead);
//    HAL_SPI_TransmitReceive(&hspi5, txDummy, pBuffer, NumByteToRead, 100);
//    vPortFree(txDummy);
    HAL_SPI_Receive(&hspi5, pBuffer, NumByteToRead, 100);
    
    /* 停止信号 FLASH: CS 高电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}

 /**
  * @brief  读取FLASH ID
  * @param 	无
  * @retval FLASH ID
  */
uint32_t SPI_FLASH_ReadID(void)
{
    uint32_t retVal;
    uint8_t txData[] = {W25X_JedecDeviceID};
    uint8_t rxData[4] = {0};
    
    /* 开始通讯：CS低电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    
    /* rxData[0]:存放发送地址时读回的数据 */
#if 1
    HAL_SPI_TransmitReceive(&hspi5, txData, rxData, sizeof(rxData), 1000);
    retVal = rxData[1]<<16 | rxData[2]<<8 | rxData[3];
#else
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);
    HAL_SPI_Receive(&hspi5, rxData, sizeof(rxData) - 1, 1000);
    retVal = rxData[0]<<16 | rxData[1]<<8 | rxData[2];
#endif
    /* 停止通讯：CS高电平 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);

    return retVal;
}

 /**
  * @brief  读取FLASH Device ID
  * @param 	无
  * @retval FLASH Device ID
  */
uint32_t SPI_FLASH_ReadDeviceID(void)
{
    uint32_t retVal = 0;
    uint8_t txData[] = {W25X_DeviceID};
    uint8_t rxData[5] = {0};
    
    /* Select the FLASH: Chip Select low */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    /* rxData[0]:存放发送地址时读回的数据 */
    HAL_SPI_TransmitReceive(&hspi5, txData, rxData, sizeof(rxData), 1000);
    retVal = rxData[4];
    
    /* Deselect the FLASH: Chip Select high */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
    
    return retVal;
}

/**
 * @brief  读取FLASH Device ID
 * @param 	无
 * @retval FLASH Device ID
 */
uint32_t SPI_FLASH_ReadManufactDeviceID(void)
{
	uint32_t retVal = 0;
    uint8_t txData[] = {W25X_ManufactDeviceID};
    uint8_t rxData[6] = {0};
	
	/* Select the FLASH: Chip Select low */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);	
    /* rxData[0]:存放发送地址时读回的数据 */
    HAL_SPI_TransmitReceive(&hspi5, txData, rxData, sizeof(rxData), 1000);
    retVal = rxData[4]<<8 | rxData[5];
	/* Deselect the FLASH: Chip Select high */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
	
	return retVal;
}
/*******************************************************************************
* Function Name  : SPI_FLASH_StartReadSequence
* Description    : Initiates a read data byte (READ) sequence from the Flash.
*                  This is done by driving the /CS line low to select the device,
*                  then the READ instruction is transmitted followed by 3 bytes
*                  address. This function exit and keep the /CS line low, so the
*                  Flash still being selected. With this technique the whole
*                  content of the Flash is read with a single READ instruction.
* Input          : - ReadAddr : FLASH's internal address to read from.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr)
{
  /* Select the FLASH: Chip Select low */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

  /* Send "Read from Memory " instruction */
  SPI_FLASH_SendByte(W25X_ReadData);

  /* Send the 24-bit address of the address to read from -----------------------*/
  /* Send ReadAddr high nibble address byte */
  SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte */
  SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte */
  SPI_FLASH_SendByte(ReadAddr & 0xFF);
}


 /**
  * @brief  使用SPI读取一个字节的数据
  * @param  无
  * @retval 返回接收到的数据
  */
uint8_t SPI_FLASH_ReadByte(void)
{
    return (SPI_FLASH_SendByte(Dummy_Byte));
}


 /**
  * @brief  使用SPI发送一个字节的数据
  * @param  byte：要发送的数据
  * @retval 返回接收到的数据
  */
uint8_t SPI_FLASH_SendByte(uint8_t byte)
{
    uint8_t retval;
    
    HAL_SPI_Transmit(&hspi5, &byte, sizeof(byte), 100);
    HAL_SPI_Receive(&hspi5, &retval, sizeof(retval), 100);
    return retval;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_SendHalfWord
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
uint16_t SPI_FLASH_SendHalfWord(uint16_t HalfWord)
{
  SPITimeout = SPIT_FLAG_TIMEOUT;

  /* Loop while DR register in not emplty */
  while (!(HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_READY))
  {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(2);
   }

  /* Send Half Word through the FLASH_SPI peripheral */
   HAL_SPI_Transmit(&hspi5, (uint8_t* )&HalfWord, sizeof(HalfWord), 10);

  SPITimeout = SPIT_FLAG_TIMEOUT;

  /* Wait to receive a Half Word */
  while (!(HAL_SPI_GetState(&hspi5) == HAL_SPI_STATE_READY))
   {
    if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(3);
   }
  /* Return the Half Word read from the SPI bus */
   uint16_t retval;
   HAL_SPI_Receive(&hspi5, (uint8_t* )&retval, sizeof(retval), 10);
  return retval;
}


 /**
  * @brief  向FLASH发送 写使能 命令
  * @param  none
  * @retval none
  */
void SPI_FLASH_WriteEnable(void)
{
    uint8_t txData[] = {W25X_WriteEnable};
    /* 通讯开始：CS低 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    
    /* 发送写使能命令*/
//    SPI_FLASH_SendByte(W25X_WriteEnable);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);
    
    /*通讯结束：CS高 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}

 /**
  * @brief  等待WIP(BUSY)标志被置0，即等待到FLASH内部数据写入完毕
  * @param  none
  * @retval none
  */
void SPI_FLASH_WaitForWriteEnd(void)
{
  uint8_t FLASH_Status = 0;
    uint8_t txData[] = {W25X_ReadStatusReg};

  /* 选择 FLASH: CS 低 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

  /* 发送 读状态寄存器 命令 */
//  SPI_FLASH_SendByte(W25X_ReadStatusReg);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);

  SPITimeout = SPIT_FLAG_TIMEOUT;
  /* 若FLASH忙碌，则等待 */
  do
  {
    /* 读取FLASH芯片的状态寄存器 */
//    FLASH_Status = SPI_FLASH_SendByte(Dummy_Byte);	 
    HAL_SPI_Receive(&hspi5, &FLASH_Status, sizeof(FLASH_Status), 1000);
    {
      if((SPITimeout--) == 0) 
      {
        SPI_TIMEOUT_UserCallback(4);
        return;
      }
    } 
  }
  while ((FLASH_Status & WIP_Flag) == SET); /* 正在写入标志 */

  /* 停止信号  FLASH: CS 高 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}


//进入掉电模式
void SPI_Flash_PowerDown(void)   
{
    uint8_t txData[] = {W25X_PowerDown};
  /* 选择 FLASH: CS 低 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

  /* 发送 掉电 命令 */
//  SPI_FLASH_SendByte(W25X_PowerDown);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);

  /* 停止信号  FLASH: CS 高 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}   

//唤醒
void SPI_Flash_WAKEUP(void)   
{
    uint8_t txData[] = {W25X_ReleasePowerDown};
  /*选择 FLASH: CS 低 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

  /* 发上 上电 命令 */
//  SPI_FLASH_SendByte(W25X_ReleasePowerDown);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);

  /* 停止信号 FLASH: CS 高 */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET); // 等待TRES1
}   


/**
  * @brief  等待超时回调函数
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* 等待超时后的处理,输出错误信息 */
//  FLASH_ERROR("SPI 等待超时!errorCode = %d",errorCode);
  return 0;
}
   
/*********************************************END OF FILE**********************/
