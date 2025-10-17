/***************************************************************************************
*文件名称   nandflash_sram.h  
*作者  		Three Gorges Nengshida Electric Co.,Ltd
*版本 		NSD_STM32F2_Driver_1.0
*修订日期   2013-08-22
*备注  		SRAM和Nand_Flash
****************************************************************************************
*使用说明:																		   *
*SRAM的使用:																		  *
*SRAM型号为IS61WV25616BLL，可以储存256K个字，数据位宽度:16位，地址范围0～0x3FF         *
*SRAM初始化函数void SRAM_Init(void)													  *
*SRAM写函数																			   *
void SRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
*SRAM读函数																				*
void SRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead)  *
*Nand_Flash的使用:															*
*Nand_Flash型号为K9F2G08U0M,可以储存2G个bits, 1 Device = (2K)B x 64Pages x 2048 Blocks  *
 =2048Mbits; 1 Block = (2K)B x 64 Pages= (128K) Bytes; 1 Page = (2K)Bytes                *
*Nand_Flash初始化函数void NandFlash_Init(void)									*
*读取Nand_Flash自身ID函数void NandFlash_ReadID(NAND_IDTypeDef* NAND_ID)           *
*Nand_Flash写页函数
 uint32_t NAND_WriteSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToWrite)
*Nand_Flash读页函数
 uint32_t NAND_ReadSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToRead)
*Nand_Flash块擦除函数 uint32_t NAND_EraseBlock(NAND_ADDRESS Address);

注意：在对Nand_Flash进行写操作时，必须先对相应的块进行擦除。           
***************************************************************************************/

/* Define to prevent recursive inclusion ---------------------------------------------*/
#ifndef __NANDFLASH_SRAM_H
#define __NANDFLASH_SRAM_H
#include <stm32f4xx.h>

typedef struct
{
 	uint8_t Maker_ID;
  	uint8_t Device_ID;
  	uint8_t Third_ID;
  	uint8_t Fourth_ID;
}NAND_IDTypeDef;

typedef struct 
{
  	uint16_t Zone;
  	uint16_t Block;
  	uint16_t Page;
} NAND_ADDRESS;

/***************************************************************************************
*函数名：SRAM_Init																	 *
*功能：SRAM初始化函数																 *
***************************************************************************************/
void SRAM_Init(void);

/***************************************************************************************
*函数名：SRAM_WriteBuffer															*
*功能：向指定地址写入指定长度的字到SRAM中，数据位数为16位							*	
*参数：	pBuffer 数据缓冲区首地址													*
		WriteAddr 写入RAM的地址														*
		NumHalfwordToWrite 写入数据的长度											*
***************************************************************************************/
void SRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite);

/***************************************************************************************
*函数名：SRAM_ReadBuffer														   	*
*功能：从SRAM中读取数据															  	*
*参数：pBuffer 数据缓冲区首地址												  		*
	   ReadAddr	读取的RAM地址													  	*
	   NumHalfwordToRead 读取的数据长度											  	*
***************************************************************************************/
void SRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead);

/***************************************************************************************
*函数名：NandFlash_Init														   *
*功能：Nand_Flash的初始化函数												   *
***************************************************************************************/
void NandFlash_Init(void);

/***************************************************************************************
*函数名：NandFlash_ReadID														*
*功能：读取NAND 存储的ID														 *
*参数：NAND_ID 保存制造商和设备ID的NAND_IDTypeDef结构体指针。					 *
**********s****************************************************************************/
void NandFlash_ReadID(NAND_IDTypeDef* NAND_ID);

/***************************************************************************************
*函数名：NAND_ReadSmallPage														 *
*功能：读取一个或多个2048字节页大小的内容。										 *
*参数：pBuffer 存储数据的缓冲区的指针											 *
  	   Address 第一页地址														 *
  	   NumPageToWrite 需要读取的页数											 *
*返回值： 新的NAND操作的状态。此参数是：										 *
  			- NAND_TIMEOUT_ERROR：前面的操作时发生超时错误						 *
			- NAND_READY：当内存准备下一次操作和增加新操作的操作地址。参数是：	 *
			- NAND_VALID_ADDRESS：当新的地址是有效的地址						 *
			- NAND_INVALID_ADDRESS：当新的地址是无效的地址						 *
***************************************************************************************/
uint32_t NAND_ReadSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToRead);

/***************************************************************************************
*函数名：NAND_WriteSmallPage													 *
*功能：写一个或多个2048字节页大小。												 *
*输入参数：pBuffer 要写入数据的缓冲区的指针									 		 *
  	   Address 第一页地址													  *
  	   NumPageToWrite 需要写的页数									  			*
*返回值： 新的NAND操作的状态。此参数是：									 *
  		- NAND_TIMEOUT_ERROR：前面的操作时发生超时错误						  *
		- NAND_READY：当内存准备下一次操作和增加新操作的操作地址。参数是：	  *
		- NAND_VALID_ADDRESS：当新的地址是有效的地址						  *
		- NAND_INVALID_ADDRESS：当新的地址是无效的地址						  *
***************************************************************************************/
uint32_t NAND_WriteSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToWrite);

/***************************************************************************************
*函数名：NAND_EraseBlock														   *
*功能：完全擦除一个NAND Flash块											   	*
*输入参数： Address：任何要擦除的块地址												*
*返回值：新的NAND操作的状态。此参数是：										   *
  			- NAND_TIMEOUT_ERROR：前面的操作时发生超时错误						   *
			- NAND_READY：当内存准备下一次操作和增加新操作的操作地址。			   *
***************************************************************************************/
uint32_t NAND_EraseBlock(NAND_ADDRESS Address);

#endif 


/******* (C) COPYRIGHT 2013 Three Gorges Nengshida Electric Co.,Ltd *****END OF FILE***/

