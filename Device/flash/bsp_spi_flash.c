 /**
  ******************************************************************************
  * @file    bsp_spi_flash.c
  * @author  fire
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   spi flash �ײ�Ӧ�ú���bsp 
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:����STM32 F429 ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
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
  * @brief  SPI_FLASH��ʼ��
  * @param  ��
  * @retval ��
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
  * @brief  ����FLASH����
  * @param  SectorAddr��Ҫ������������ַ
  * @retval ��
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
  * @brief  ����FLASH��������Ƭ����
  * @param  ��
  * @retval ��
  */
void SPI_FLASH_BulkErase(void)
{
    /* ����FLASHдʹ������ */
    SPI_FLASH_WriteEnable();
    
    /* ���� Erase */
    /* ѡ��FLASH: CS�͵�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    /* �����������ָ��*/
    SPI_FLASH_SendByte(W25X_ChipErase);
    /* ֹͣ�ź� FLASH: CS �ߵ�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
    
    /* �ȴ��������*/
    SPI_FLASH_WaitForWriteEnd();
}



 /**
  * @brief  ��FLASH��ҳд�����ݣ����ñ�����д������ǰ��Ҫ�Ȳ�������
  * @param	pBuffer��Ҫд�����ݵ�ָ��
  * @param WriteAddr��д���ַ
  * @param  NumByteToWrite��д�����ݳ��ȣ�����С�ڵ���SPI_FLASH_PerWritePageSize
  * @retval ��
  */
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t txData[4];
 
    txData[0] = W25X_PageProgram;
    txData[1] = (WriteAddr & 0xFF0000) >> 16;
    txData[2] = (WriteAddr & 0xFF00) >> 8;
    txData[3] = WriteAddr & 0xFF;
    
    /* ����FLASHдʹ������ */
    SPI_FLASH_WriteEnable();
    
    /* ѡ��FLASH: CS�͵�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
//    /* дҳдָ��*/
//    SPI_FLASH_SendByte(W25X_PageProgram);
//    /*����д��ַ�ĸ�λ*/
//    SPI_FLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
//    /*����д��ַ����λ*/
//    SPI_FLASH_SendByte((WriteAddr & 0xFF00) >> 8);
//    /*����д��ַ���ݴ���ĵ�λ*/
//    SPI_FLASH_SendByte(WriteAddr & 0xFF);
    
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 100);
    
    if (NumByteToWrite > SPI_FLASH_PerWritePageSize) {
        NumByteToWrite = SPI_FLASH_PerWritePageSize;
//        FLASH_ERROR("SPI_FLASH_PageWrite too large!");
    }
    
//    /* д������*/
//    while (NumByteToWrite--)
//    {
//        /* ���͵�ǰҪд����ֽ����� */
//        SPI_FLASH_SendByte(*pBuffer);
//        /* ָ����һ�ֽ����� */
//        pBuffer++;
//    }
    
    HAL_SPI_Transmit(&hspi5, pBuffer, NumByteToWrite, 100);
    
    /* ֹͣ�ź� FLASH: CS �ߵ�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
    
    /* �ȴ�д�����*/
    SPI_FLASH_WaitForWriteEnd();
}


 /**
  * @brief  ��FLASHд�����ݣ����ñ�����д������ǰ��Ҫ�Ȳ�������
  * @param	pBuffer��Ҫд�����ݵ�ָ��
  * @param  WriteAddr��д���ַ
  * @param  NumByteToWrite��д�����ݳ���
  * @retval ��
  */
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
    uint8_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;
	
	/*mod�������࣬��writeAddr��SPI_FLASH_PageSize��������������AddrֵΪ0*/
    Addr = WriteAddr % SPI_FLASH_PageSize;
	
	/*��count������ֵ���պÿ��Զ��뵽ҳ��ַ*/
    count = SPI_FLASH_PageSize - Addr;	
	/*�����Ҫд��������ҳ*/
    NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
	/*mod�������࣬�����ʣ�಻��һҳ���ֽ���*/
    NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

    /* Addr=0,��WriteAddr �պð�ҳ���� aligned  */
    if (Addr == 0) {
        /* NumByteToWrite < SPI_FLASH_PageSize */
        if (NumOfPage == 0) {
            SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
        } else {
            /* NumByteToWrite > SPI_FLASH_PageSize */
            /*�Ȱ�����ҳ��д��*/
            while (NumOfPage--) {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
                WriteAddr +=  SPI_FLASH_PageSize;
                pBuffer += SPI_FLASH_PageSize;
            }
                
            /*���ж���Ĳ���һҳ�����ݣ�����д��*/
            SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        }
    } else {
        /* ����ַ�� SPI_FLASH_PageSize ������  */
        /* NumByteToWrite < SPI_FLASH_PageSize */
        if (NumOfPage == 0)  {
            /*��ǰҳʣ���count��λ�ñ�NumOfSingleС��д����*/
            if (NumOfSingle > count) {
                temp = NumOfSingle - count;
                        
                /*��д����ǰҳ*/
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
                WriteAddr +=  count;
                pBuffer += count;
                        
                /*��дʣ�������*/
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, temp);
            } else {
                /*��ǰҳʣ���count��λ����д��NumOfSingle������*/                
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumByteToWrite);
            }
        } else {
            /* NumByteToWrite > SPI_FLASH_PageSize */
            /*��ַ����������count�ֿ������������������*/
            NumByteToWrite -= count;
            NumOfPage = NumByteToWrite / SPI_FLASH_PageSize;
            NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;
        
            SPI_FLASH_PageWrite(pBuffer, WriteAddr, count);
            WriteAddr += count;
            pBuffer += count;
                
            /*������ҳ��д��*/
            while (NumOfPage--) {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, SPI_FLASH_PageSize);
                WriteAddr +=  SPI_FLASH_PageSize;
                pBuffer += SPI_FLASH_PageSize;
            }
                /*���ж���Ĳ���һҳ�����ݣ�����д��*/
            if (NumOfSingle != 0) {
                SPI_FLASH_PageWrite(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    }
}

/**
 * @brief    read data from flash
 * @note     data size change 32bit
 * @issue    SZIE_16_BIT = 0xFFFF ʱ����ȡ�����쳣��analog clock widget ��ʾ��3��������ͼ��3����Ӧ��ȡ������Ϊ
 *           SZIE_16_BIT���Σ��²���spi���ӿ��޷���ȡ���0xFFFF��������
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
  * @brief  ��ȡFLASH����
  * @param 	pBuffer���洢�������ݵ�ָ��
  * @param   ReadAddr����ȡ��ַ
  * @param   NumByteToRead����ȡ���ݳ���
  * @retval ��
  */
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
    uint8_t txData[4];

    txData[0] = W25X_ReadData;
    txData[1] = (ReadAddr & 0xFF0000) >> 16;
    txData[2] = (ReadAddr & 0xFF00) >> 8;
    txData[3] = ReadAddr & 0xFF;
    
    /* ѡ��FLASH: CS�͵�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    
//    /* ���� �� ָ�� */
//    SPI_FLASH_SendByte(W25X_ReadData);
//    
//    /* ���� �� ��ַ��λ */
//    SPI_FLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
//    /* ���� �� ��ַ��λ */
//    SPI_FLASH_SendByte((ReadAddr& 0xFF00) >> 8);
//    /* ���� �� ��ַ��λ */
//    SPI_FLASH_SendByte(ReadAddr & 0xFF);
    
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 100);
    
//    /* ��ȡ���� */
//    while (NumByteToRead--) {
//        /* ��ȡһ���ֽ�*/
//        *pBuffer = SPI_FLASH_SendByte(Dummy_Byte);
//        /* ָ����һ���ֽڻ����� */
//        pBuffer++;
//    }
//    uint8_t* txDummy = ( uint8_t* )pvPortMalloc(NumByteToRead);
//    memset(txDummy, Dummy_Byte, NumByteToRead);
//    HAL_SPI_TransmitReceive(&hspi5, txDummy, pBuffer, NumByteToRead, 100);
//    vPortFree(txDummy);
    HAL_SPI_Receive(&hspi5, pBuffer, NumByteToRead, 100);
    
    /* ֹͣ�ź� FLASH: CS �ߵ�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}

 /**
  * @brief  ��ȡFLASH ID
  * @param 	��
  * @retval FLASH ID
  */
uint32_t SPI_FLASH_ReadID(void)
{
    uint32_t retVal;
    uint8_t txData[] = {W25X_JedecDeviceID};
    uint8_t rxData[4] = {0};
    
    /* ��ʼͨѶ��CS�͵�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    
    /* rxData[0]:��ŷ��͵�ַʱ���ص����� */
#if 1
    HAL_SPI_TransmitReceive(&hspi5, txData, rxData, sizeof(rxData), 1000);
    retVal = rxData[1]<<16 | rxData[2]<<8 | rxData[3];
#else
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);
    HAL_SPI_Receive(&hspi5, rxData, sizeof(rxData) - 1, 1000);
    retVal = rxData[0]<<16 | rxData[1]<<8 | rxData[2];
#endif
    /* ֹͣͨѶ��CS�ߵ�ƽ */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);

    return retVal;
}

 /**
  * @brief  ��ȡFLASH Device ID
  * @param 	��
  * @retval FLASH Device ID
  */
uint32_t SPI_FLASH_ReadDeviceID(void)
{
    uint32_t retVal = 0;
    uint8_t txData[] = {W25X_DeviceID};
    uint8_t rxData[5] = {0};
    
    /* Select the FLASH: Chip Select low */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    /* rxData[0]:��ŷ��͵�ַʱ���ص����� */
    HAL_SPI_TransmitReceive(&hspi5, txData, rxData, sizeof(rxData), 1000);
    retVal = rxData[4];
    
    /* Deselect the FLASH: Chip Select high */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
    
    return retVal;
}

/**
 * @brief  ��ȡFLASH Device ID
 * @param 	��
 * @retval FLASH Device ID
 */
uint32_t SPI_FLASH_ReadManufactDeviceID(void)
{
	uint32_t retVal = 0;
    uint8_t txData[] = {W25X_ManufactDeviceID};
    uint8_t rxData[6] = {0};
	
	/* Select the FLASH: Chip Select low */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);	
    /* rxData[0]:��ŷ��͵�ַʱ���ص����� */
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
  * @brief  ʹ��SPI��ȡһ���ֽڵ�����
  * @param  ��
  * @retval ���ؽ��յ�������
  */
uint8_t SPI_FLASH_ReadByte(void)
{
    return (SPI_FLASH_SendByte(Dummy_Byte));
}


 /**
  * @brief  ʹ��SPI����һ���ֽڵ�����
  * @param  byte��Ҫ���͵�����
  * @retval ���ؽ��յ�������
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
  * @brief  ��FLASH���� дʹ�� ����
  * @param  none
  * @retval none
  */
void SPI_FLASH_WriteEnable(void)
{
    uint8_t txData[] = {W25X_WriteEnable};
    /* ͨѶ��ʼ��CS�� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);
    
    /* ����дʹ������*/
//    SPI_FLASH_SendByte(W25X_WriteEnable);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);
    
    /*ͨѶ������CS�� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}

 /**
  * @brief  �ȴ�WIP(BUSY)��־����0�����ȴ���FLASH�ڲ�����д�����
  * @param  none
  * @retval none
  */
void SPI_FLASH_WaitForWriteEnd(void)
{
  uint8_t FLASH_Status = 0;
    uint8_t txData[] = {W25X_ReadStatusReg};

  /* ѡ�� FLASH: CS �� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

  /* ���� ��״̬�Ĵ��� ���� */
//  SPI_FLASH_SendByte(W25X_ReadStatusReg);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);

  SPITimeout = SPIT_FLAG_TIMEOUT;
  /* ��FLASHæµ����ȴ� */
  do
  {
    /* ��ȡFLASHоƬ��״̬�Ĵ��� */
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
  while ((FLASH_Status & WIP_Flag) == SET); /* ����д���־ */

  /* ֹͣ�ź�  FLASH: CS �� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}


//�������ģʽ
void SPI_Flash_PowerDown(void)   
{
    uint8_t txData[] = {W25X_PowerDown};
  /* ѡ�� FLASH: CS �� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

  /* ���� ���� ���� */
//  SPI_FLASH_SendByte(W25X_PowerDown);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);

  /* ֹͣ�ź�  FLASH: CS �� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET);
}   

//����
void SPI_Flash_WAKEUP(void)   
{
    uint8_t txData[] = {W25X_ReleasePowerDown};
  /*ѡ�� FLASH: CS �� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_RESET);

  /* ���� �ϵ� ���� */
//  SPI_FLASH_SendByte(W25X_ReleasePowerDown);
    HAL_SPI_Transmit(&hspi5, txData, sizeof(txData), 1000);

  /* ֹͣ�ź� FLASH: CS �� */
    HAL_GPIO_WritePin(W25Q_SPI_CS_GPIO_Port, W25Q_SPI_CS_Pin, GPIO_PIN_SET); // �ȴ�TRES1
}   


/**
  * @brief  �ȴ���ʱ�ص�����
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
  /* �ȴ���ʱ��Ĵ���,���������Ϣ */
//  FLASH_ERROR("SPI �ȴ���ʱ!errorCode = %d",errorCode);
  return 0;
}
   
/*********************************************END OF FILE**********************/
