/* Includes ------------------------------------------------------------------*/
#include "nandflash_sram.h"


#define CMD_AREA                   (uint32_t)(1<<16)  		// A16 = CLE high 
#define ADDR_AREA                  (uint32_t)(1<<17)  		// A17 = ALE high 

#define DATA_AREA                  ((uint32_t)0x00000000) 

/* FSMC NAND memory command */
//#define Bank_NAND_ADDR			   ((uint32_t)0x70000000) 
#define	NAND_CMD_AREA_A            ((uint8_t)0x00)
#define	NAND_CMD_AREA_B            ((uint8_t)0x01)
#define NAND_CMD_AREA_C            ((uint8_t)0x50)
#define NAND_CMD_AREA_TRUE1        ((uint8_t)0x30)

#define NAND_CMD_WRITE0            ((uint8_t)0x80)
#define NAND_CMD_WRITE_TRUE1       ((uint8_t)0x10)
	
#define NAND_CMD_ERASE0            ((uint8_t)0x60)
#define NAND_CMD_ERASE1            ((uint8_t)0xD0)  

#define NAND_CMD_READID            ((uint8_t)0x90)	
#define NAND_CMD_STATUS            ((uint8_t)0x70)
#define NAND_CMD_LOCK_STATUS       ((uint8_t)0x7A)
#define NAND_CMD_RESET             ((uint8_t)0xFF)

/* NAND memory status */
#define NAND_VALID_ADDRESS         ((uint32_t)0x00000100)
#define NAND_INVALID_ADDRESS       ((uint32_t)0x00000200)
#define NAND_TIMEOUT_ERROR         ((uint32_t)0x00000400)
#define NAND_BUSY                  ((uint32_t)0x00000000)
#define NAND_ERROR                 ((uint32_t)0x00000001)
#define NAND_READY                 ((uint32_t)0x00000040)

/* FSMC NAND memory parameters */
#define NAND_PAGE_SIZE             ((uint16_t)0x0800) 				// 2048 bytes per page w/o Spare Area 
#define NAND_BLOCK_SIZE            ((uint16_t)0x0040) 				// 64x2048 bytes pages per block 
#define NAND_ZONE_SIZE             ((uint16_t)0x0800) 				// 2048 Block per zone 
#define NAND_SPARE_AREA_SIZE       ((uint16_t)0x0040) 				// last 64 bytes as spare area 
#define NAND_MAX_ZONE              ((uint16_t)0x0001) 				// 1 zones of 2048 block 

//FSMC的NAND内存地址计算
#define ADDR_1st_CYCLE(ADDR)       (uint8_t) 0x00               		/* 1st addressing cycle */
#define ADDR_2nd_CYCLE(ADDR)       (uint8_t) 0x00      					/* 2nd addressing cycle */
#define ADDR_3rd_CYCLE(ADDR)       (uint8_t)((ADDR)& 0x0000FF)    		/* 3rd addressing cycle */
#define ADDR_4th_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0x00FF00) >> 8) 	/* 4th addressing cycle */
#define ADDR_5fh_CYCLE(ADDR)       (uint8_t)(((ADDR)& 0x010000) >> 16)	/* 5th addressing cycle */

#define Bank1_SRAM1_ADDR  ((uint32_t)0x68000000)

#define FSMC_Bank_NAND     FSMC_Bank2_NAND
#define Bank_NAND_ADDR     Bank2_NAND_ADDR
#define Bank2_NAND_ADDR    ((uint32_t)0x70000000)

#define ROW_ADDRESS (Address.Page + (Address.Block + (Address.Zone * NAND_ZONE_SIZE)) * NAND_BLOCK_SIZE)

void PC13_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
}


/*******************************************************************************
*NandFlash_Init
*FSMC的配置和GPIO接口与NAND存储器。
*这个函数必须被调用在任何读/写操作上的NAND闪存之前。
*******************************************************************************/
void NandFlash_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	FSMC_NAND_PCCARDTimingInitTypeDef  p;
	FSMC_NANDInitTypeDef FSMC_NANDInitStructure;

	// Enable GPIOs clock 
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
                           RCC_AHB1Periph_GPIOG, ENABLE);
  	// Enable FSMC clock 
  	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 
  
	//-- GPIOs Configuration ------------------------------------------------------
	/*
	 +---------------------+---------------------+
	 +          NandFlash pins assignment        +
	 +---------------------+---------------------+
	 | PD14  <-> FSMC_D0   | PD11 <-> FSMC_A16   |
	 | PD15  <-> FSMC_D1   | PD12 <-> FSMC_A17   |
	 | PD0   <-> FSMC_D2   | PD4  <-> FSMC_NOE   |
	 | PD1   <-> FSMC_D3   | PD5  <-> FSMC_NWE   |
	 | PE7   <-> FSMC_D4   | PD7  <-> FSMC_NCE2  |
	 | PE8   <-> FSMC_D5   | PD6  <-> FSMC_NWAIT |
	 | PE9   <-> FSMC_D6   |                     |
	 | PE10  <-> FSMC_D7   |				     |
	 +---------------------+---------------------+	 */
	//GPIOD configuration 
 	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);	  	//D2
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);	  	//D3
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);	  	//NOE
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);	  	//NWE
//	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_FSMC);	  	//NWAIT
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);	  	//NCE2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);  	//A16
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);	//A17
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);  	//D0
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);	//D1


  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                               	  /*GPIO_Pin_6 |*/ GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12|
								  GPIO_Pin_14 | GPIO_Pin_15;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	 // GPIOE configuration 
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);  //D4
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);  //D5
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);  //D6
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC); //D7
  
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7  | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
   	GPIO_Init(GPIOE, &GPIO_InitStructure);

	 // NWAIT NAND引脚配置
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6 , GPIO_AF_FSMC);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

	//FSMC 总线 存储器参数配置
	p.FSMC_SetupTime = 0x00;         //建立时间
	p.FSMC_WaitSetupTime = 0x03;     //等待时间
	p.FSMC_HoldSetupTime = 0x02;     //保持时间
	p.FSMC_HiZSetupTime = 0x00;      //高阻建立时间
	FSMC_NANDInitStructure.FSMC_Bank = FSMC_Bank2_NAND; 					//使用FSMC BANK2
	FSMC_NANDInitStructure.FSMC_Waitfeature = FSMC_Waitfeature_Disable; 	//使能FSMC的等待功能
	FSMC_NANDInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b; 	//NAND Flash的数据宽度为8位
	FSMC_NANDInitStructure.FSMC_ECC = FSMC_ECC_Enable;                 	 	//使能ECC特性
	FSMC_NANDInitStructure.FSMC_ECCPageSize = FSMC_ECCPageSize_2048Bytes; 	//ECC页大小2048
	FSMC_NANDInitStructure.FSMC_TCLRSetupTime = 0x00;             
	FSMC_NANDInitStructure.FSMC_TARSetupTime = 0x00;
	FSMC_NANDInitStructure.FSMC_CommonSpaceTimingStruct = &p;
	FSMC_NANDInitStructure.FSMC_AttributeSpaceTimingStruct = &p;
	FSMC_NANDInit(&FSMC_NANDInitStructure);
	//使能FSMC BANK2 
	FSMC_NANDCmd(FSMC_Bank2_NAND, ENABLE);
	
	PC13_Init();
}

/*******************************************************************************
*SRAM_Init
*FSMC的配置和GPIO接口与SRAM存储器。
*这个函数必须被调用在任何读/写操作上的SRAM之前。
*******************************************************************************/
void SRAM_Init(void)
{
  	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  	FSMC_NORSRAMTimingInitTypeDef  p;
  	GPIO_InitTypeDef GPIO_InitStructure; 
  
  	// Enable GPIOs clock 
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
                         RCC_AHB1Periph_GPIOG, ENABLE);

  	// Enable FSMC clock 
  	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 
  
	//-- GPIOs Configuration -----------------------------------------------------
	/*
	 +-------------------+--------------------+------------------+------------------+
	 +                       SRAM pins assignment                                  +
	 +-------------------+--------------------+------------------+------------------+
	 | PD0   <-> FSMC_D2  | PE0  <-> FSMC_NBL0 | PF0  <-> FSMC_A0 | PG0 <-> FSMC_A10 |
	 | PD1   <-> FSMC_D3  | PE1  <-> FSMC_NBL1 | PF1  <-> FSMC_A1 | PG1 <-> FSMC_A11 |
	 | PD4   <-> FSMC_NOE | PE7  <-> FSMC_D4   | PF2  <-> FSMC_A2 | PG2 <-> FSMC_A12 |
	 | PD5   <-> FSMC_NWE | PE8  <-> FSMC_D5   | PF3  <-> FSMC_A3 | PG3 <-> FSMC_A13 |
	 | PG10  <-> FSMC_NE3 | PE9  <-> FSMC_D6   | PF4  <-> FSMC_A4 | PG4 <-> FSMC_A14 |
	 | PD8   <-> FSMC_D13 | PE10 <-> FSMC_D7   | PF5  <-> FSMC_A5 | PG5 <-> FSMC_A15 |
	 | PD9   <-> FSMC_D14 | PE11 <-> FSMC_D8   | PF12 <-> FSMC_A6 |PD13 <-> FSMC_A18 |
	 | PD10  <-> FSMC_D15 | PE12 <-> FSMC_D9   | PF13 <-> FSMC_A7 | PE3 <-> FSMC_A19 |
	 | PD11  <-> FSMC_A16 | PE13 <-> FSMC_D10  | PF14 <-> FSMC_A8 | 
	 | PD12  <-> FSMC_A17 | PE14 <-> FSMC_D11  | PF15 <-> FSMC_A9 | 
	 | PD14  <-> FSMC_D0  | PE15 <-> FSMC_D12  |------------------+
	 | PD15  <-> FSMC_D1  |										|
	 +-------------------+--------------------+
	*/
  	// GPIOD configuration 
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC); 
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 |
                                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

  	GPIO_Init(GPIOD, &GPIO_InitStructure);


  	// GPIOE configuration 
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_7 |
                                GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 | GPIO_Pin_11|
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

  	GPIO_Init(GPIOE, &GPIO_InitStructure);


  	// GPIOF configuration 
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15 , GPIO_AF_FSMC);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 | 
                                GPIO_Pin_4  | GPIO_Pin_5  | 
                                GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;      

  	GPIO_Init(GPIOF, &GPIO_InitStructure);


  	// GPIOG configuration 
  	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5 , GPIO_AF_FSMC);
  	GPIO_PinAFConfig(GPIOG, GPIO_PinSource10 , GPIO_AF_FSMC);


  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  | GPIO_Pin_2  | GPIO_Pin_3 | 
                                GPIO_Pin_4  | GPIO_Pin_5  |GPIO_Pin_10;      

  	GPIO_Init(GPIOG, &GPIO_InitStructure);

	// FSMC Configuration 
  	p.FSMC_AddressSetupTime = 2;
  	p.FSMC_AddressHoldTime = 1;
  	p.FSMC_DataSetupTime = 9;
  	p.FSMC_BusTurnAroundDuration = 1;
  	p.FSMC_CLKDivision = 0;
  	p.FSMC_DataLatency = 0;
  	p.FSMC_AccessMode = FSMC_AccessMode_A;

  	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;
  	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_PSRAM;
  	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
  	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

  	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  	//!< Enable FSMC Bank1_SRAM1 Bank 
  	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE); 

}



/*******************************************************************************
*SRAM_WriteBuffer
*把长度为NumHalfwordToWrite的数据写入到sram中，写入的初始地址为WriteAddr
*******************************************************************************/
void SRAM_WriteBuffer(uint16_t* pBuffer, uint32_t WriteAddr, uint32_t NumHalfwordToWrite)
{
  for (; NumHalfwordToWrite != 0; NumHalfwordToWrite--) 	// while there is data to write
  {
    // Transfer data to the memory 
    *(uint16_t *) (Bank1_SRAM1_ADDR + WriteAddr) = *pBuffer++;

    // Increment the address
    WriteAddr += 2;
  }
}

/*******************************************************************************
*SRAM_ReadBuffer															   *
*从SRAM中读取数据，读取的初始地址为ReadAddr，读取的长度为NumHalfwordToRead，   *
*读取的数据放入到pBuffer数组中												   *
*******************************************************************************/
void SRAM_ReadBuffer(uint16_t* pBuffer, uint32_t ReadAddr, uint32_t NumHalfwordToRead)
{
  for (; NumHalfwordToRead != 0; NumHalfwordToRead--) // while there is data to read 
  {
    // Read a half-word from the memory 
    *pBuffer++ = *(__IO uint16_t*) (Bank1_SRAM1_ADDR + ReadAddr);

    // Increment the address
    ReadAddr += 2;
  }
}
/*******************************************************************************
*NAND_Reset
*复位NAND闪存。
*返回参数   NAND_READY ：当内存准备好下一次操作
*******************************************************************************/
uint32_t NAND_Reset(void)
{
    *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_RESET;

    return (NAND_READY);
}

/*******************************************************************************
*NandFlash_ReadID
*读取NAND 存储的ID
*NAND_ID：一个保存制造商和设备ID的NAND_IDTypeDef结构体指针。
*******************************************************************************/
void NandFlash_ReadID(NAND_IDTypeDef* NAND_ID)
{
  	uint32_t data = 0;
  	// Send Command to the command area  	
  	*(vu8 *)(Bank_NAND_ADDR | CMD_AREA) = 0x90;
  	*(vu8 *)(Bank_NAND_ADDR | ADDR_AREA) = 0x00;
   	// Sequence to read ID from NAND flash 	
   	data = *(vu32 *)(Bank_NAND_ADDR | DATA_AREA);

   	NAND_ID->Maker_ID   = ADDR_1st_CYCLE (data);
   	NAND_ID->Device_ID  = ADDR_2nd_CYCLE (data);
   	NAND_ID->Third_ID   = ADDR_3rd_CYCLE (data);
   	NAND_ID->Fourth_ID  = ADDR_4th_CYCLE (data);  
}
/*******************************************************************************
  * @函数名称	NAND_AddressIncrement
  * @函数说明   增加的NAND内存地址。
  * @输入参数   无
  * @输出参数   无
  * @返回参数   新的增量地址操作状态。它可以是：
  *             - NAND_VALID_ADDRESS：当新的地址是有效的地址
  *             - NAND_INVALID_ADDRESS：当新的地址是无效的地址
*******************************************************************************/
uint32_t NAND_AddressIncrement(NAND_ADDRESS* Address)
{
    uint32_t status = NAND_VALID_ADDRESS;

    Address->Page++;

    if (Address->Page == NAND_BLOCK_SIZE)
    {
        Address->Page = 0;
        Address->Block++;

        if (Address->Block == NAND_ZONE_SIZE)
        {
            Address->Block = 0;
            Address->Zone++;

            if (Address->Zone == NAND_MAX_ZONE)
            {
                status = NAND_INVALID_ADDRESS;
            }
        }
    }

    return (status);
}

/*******************************************************************************
* NAND_ReadStatus
*读取NAND存储状态，使用读状态命令。
*返回参数   NAND闪存的状态。这个参数可以是：
            - NAND_BUSY：当内存忙
            - NAND_READY：当内存准备好下一次操作
            - NAND_ERROR：当前次的操作发生错误
*******************************************************************************/
uint32_t NAND_ReadStatus(void)
{
    uint32_t data = 0x00, status = NAND_BUSY;

    //读取操作状态
    *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_STATUS;
    data = *(__IO uint8_t *)(Bank_NAND_ADDR);

    if ((data & NAND_ERROR) == NAND_ERROR)
    {
        status = NAND_ERROR;
    }
    else if ((data & NAND_READY) == NAND_READY)
    {
        status = NAND_READY;
    }
    else
    {
        status = NAND_BUSY;
    }
    return (status);
}

/*******************************************************************************
*NAND_GetStatus
*获取NAND运行状态
*返回参数   新的NAND操作的状态。此参数是：
  			- NAND_TIMEOUT_ERROR：前面的操作时发生超时错误
            - NAND_READY：当内存准备好下一次操作
*******************************************************************************/
uint32_t NAND_GetStatus(void)
{
    uint32_t timeout = 0x1000000, status = NAND_READY;

    status = NAND_ReadStatus();

    //等待一个NAND操作完成或发生超时
    while ((status != NAND_READY) &&( timeout != 0x00))
    {
        status = NAND_ReadStatus();
        timeout --;
    }

    if (timeout == 0x00)
    {
        status =  NAND_TIMEOUT_ERROR;
    }
    //返回操作状态
    return (status);
}
/*******************************************************************************
*函数名称	NAND_EraseBlock
*函数说明   完全擦除一个NAND Flash块
*输入参数   Address：任何要擦除的块地址
  			Address：第一页地址
  			NumSpareAreaTowrite：需要读取的数据大小
*返回参数   新的NAND操作的状态。此参数是：
  			- NAND_TIMEOUT_ERROR：前面的操作时发生超时错误
			- NAND_READY：当内存准备下一次操作和增加新操作的操作地址。
*******************************************************************************/
uint32_t NAND_EraseBlock(NAND_ADDRESS Address)
{
    *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_ERASE0;

 //   *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_1st_CYCLE(ROW_ADDRESS);
//    *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_2nd_CYCLE(ROW_ADDRESS);
    *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_3rd_CYCLE(ROW_ADDRESS);
    *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_4th_CYCLE(ROW_ADDRESS);
    *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_5fh_CYCLE(ROW_ADDRESS);


    *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_ERASE1;

    return (NAND_GetStatus());
}
/*******************************************************************************
*NAND_WriteSmallPage
*写一个或多个2048字节页大小。
*输入参数   pBuffer：要写入数据的缓冲区的指针
  			Address：第一页地址
  				NumPageToWrite：需要写的页数
* @返回参数  新的NAND操作的状态。此参数是：
  			 - NAND_TIMEOUT_ERROR：前面的操作时发生超时错误
			 - NAND_READY：当内存准备下一次操作和增加新操作的操作地址。参数是：
			 - NAND_VALID_ADDRESS：当新的地址是有效的地址
			 - NAND_INVALID_ADDRESS：当新的地址是无效的地址
*******************************************************************************/
uint32_t NAND_WriteSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToWrite)
{
    uint32_t index = 0x00, numpagewritten = 0x00, addressstatus = NAND_VALID_ADDRESS;
    uint32_t status = NAND_READY, size = 0x00;
    //计算大小
    size = NAND_PAGE_SIZE + (NAND_PAGE_SIZE * numpagewritten);

    while ((NumPageToWrite != 0x00) && (addressstatus == NAND_VALID_ADDRESS) && (status == NAND_READY))
    {
        //页写命令和地址
        *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_AREA_A;
        *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_WRITE0;

//        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = 0x00;
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_1st_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_2nd_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_3rd_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_4th_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_5fh_CYCLE(ROW_ADDRESS);

        //写数据
        for (; index < size; index++)
        {
            *(__IO uint8_t *)(Bank_NAND_ADDR | DATA_AREA) = pBuffer[index];
        }

        *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_WRITE_TRUE1;

        //检测操作是否成功
        status = NAND_GetStatus();
        if (status == NAND_READY)
        {
            numpagewritten++;

            NumPageToWrite--;

            //计算下一页地址
            addressstatus = NAND_AddressIncrement(&Address);
        }
    }

    return (status | addressstatus);
}

/*******************************************************************************
*函数名称	NAND_ReadSmallPage
*读取一个或多个2048字节页大小的内容。
*输入参数   pBuffer：存储数据的缓冲区的指针
  			Address：第一页地址
  			NumPageToWrite：需要读取的页数
*返回参数   新的NAND操作的状态。此参数是：
  			- NAND_TIMEOUT_ERROR：前面的操作时发生超时错误
			- NAND_READY：当内存准备下一次操作和增加新操作的操作地址。参数是：
			- NAND_VALID_ADDRESS：当新的地址是有效的地址
			- NAND_INVALID_ADDRESS：当新的地址是无效的地址
*******************************************************************************/
uint32_t NAND_ReadSmallPage(uint8_t *pBuffer, NAND_ADDRESS Address, uint32_t NumPageToRead)
{
    uint32_t index = 0x00, numpageread = 0x00, addressstatus = NAND_VALID_ADDRESS;
    uint32_t status = NAND_READY, size = 0x00;
    uint32_t i = 0 ;

    //计算大小
    size = NAND_PAGE_SIZE + (NAND_PAGE_SIZE * numpageread);

    while ((NumPageToRead != 0x0) && (addressstatus == NAND_VALID_ADDRESS))
    {
        //页写命令和地址
        *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_AREA_A;

 //       *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = 0x00;
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_1st_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_2nd_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_3rd_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_4th_CYCLE(ROW_ADDRESS);
        *(__IO uint8_t *)(Bank_NAND_ADDR | ADDR_AREA) = ADDR_5fh_CYCLE(ROW_ADDRESS);

        *(__IO uint8_t *)(Bank_NAND_ADDR | CMD_AREA) = NAND_CMD_AREA_TRUE1;

        for (i = 0; i <= 10000; i++);//延时一点时间
        //读取数据
        for (; index < size; index++)
        {
            pBuffer[index]= *(__IO uint8_t *)(Bank_NAND_ADDR | DATA_AREA);
        }

        numpageread++;

        NumPageToRead--;

        //计算下一页地址
        addressstatus = NAND_AddressIncrement(&Address);
    }

    status = NAND_GetStatus();

    return (status | addressstatus);
}


//===========================================================================
// End of file.
//===========================================================================
