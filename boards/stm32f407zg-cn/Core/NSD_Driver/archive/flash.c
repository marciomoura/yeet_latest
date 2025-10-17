/********************************************************************
*M25P32                                                             *
*32Mbit Flash, 分64个扇区, 每个扇区64k字节                          *
*内存空间内容如果不全是0xff, 必须先擦除才能写入。                   *
*单次最多写入256字节，单次擦除64k字节（一个扇区）。                 *
*因为写入长度随机，不方便验证待写入的Flash是否全是0xff，所以全部擦除*
*********************************************************************
*M25P32_Write_Cycle                                                 *
*在主循环中调用的非阻塞模式的写入函数，通过全局变量M25P32_Write_Addr*
*, M25P32_Written_Length, M25P32_Write_Buffer设置写入参数，通过     *
*M25P32_Write_Flag启动写入,写入完成后通过M25P32_Write_Result给出写入*
*结果。                                                             *
********************************************************************/
#include <stm32f4xx.h>

/* M25P SPI Flash supported commands */  
#define sFLASH_CMD_WRITE          0x02  /* Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /* Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /* Write enable instruction */
#define sFLASH_CMD_READ           0x03  /* Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /* Read Status Register instruction  */
#define sFLASH_CMD_RDID           0x9F  /* Read identification */
#define sFLASH_CMD_SE             0xD8  /* Sector Erase instruction */
#define sFLASH_CMD_BE             0xC7  /* Bulk Erase instruction */

#define sFLASH_WIP_FLAG           0x01  /* Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_SPI_PAGESIZE       0x100

#define sFLASH_M25P128_ID         0x202018
#define sFLASH_M25P64_ID          0x202017


#define M25P32_Write_Step_Write_Enable	1
#define M25P32_Write_Step_Erase			2
#define M25P32_Write_Step_Write			3
#define M25P32_Write_Step_Verify		4

//extern void Upload_Ref(int ROM_Result);

unsigned char M25P32_Write_Flag;		//写入标志, 供外部调用, 启动写入流程
unsigned char M25P32_Write_Result;		//写入后的校验结果: 0->ok, 1->fail
unsigned char M25P32_Write_Step;		//当前写入进入了哪一步
unsigned char M25P32_Write_Old_Flag;	//上次的写入标志, 供内部启动写入流程初始化
unsigned char* M25P32_Write_Buffer;		//写入数据区指针, 供外部调用
int M25P32_Write_Length;				//写入长度, 供外部调用
int M25P32_Written_Length;				//已经写入的长度
int M25P32_Write_Length_Once;			//单次写入长度
unsigned int M25P32_Write_Addr_Once;	//单次写入的地址
unsigned char* M25P32_Write_Buffer_Once;//单次写入数据区指针
unsigned int M25P32_Write_Sector;		//待写入的扇区,
unsigned short int M25P32_Write_Offset;	//待写入的地址偏移量
unsigned int M25P32_Write_Addr;			//写入地址, 供外部调用

unsigned char ucW[512];
unsigned char ucV[256];

uint32_t sFLASH_ReadID(void);
unsigned char M25P32_Write_Data(void);

/********************************************************************
*M25P32_Init_Flag                                                   *
*芯片标志数据初始化。                                               *
********************************************************************/
void M25P32_Init_Flag(void)
{
	M25P32_Write_Flag=0;
	M25P32_Write_Old_Flag=0;
	M25P32_Write_Buffer=0;
	M25P32_Write_Length=0;
	M25P32_Written_Length=0;
	M25P32_Write_Addr=0;
 	M25P32_Write_Sector=0;
 	M25P32_Write_Addr_Once=0;
 	M25P32_Write_Buffer_Once=0;
 	M25P32_Write_Length_Once=0;
 	M25P32_Write_Offset=0;
	M25P32_Write_Step=0;
	M25P32_Write_Result=0;
}

/********************************************************************
*M25P32_SPI1_Init                                                   *
*初始化SPI1的NSS、MOSI、MISO、SCK引脚。                             *
********************************************************************/
void M25P32_SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
  	GPIO_InitTypeDef GPIO_InitStructure;

	//分别为SPI1和GPIOA,GPIOE启用时钟信号
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	//将PA5~PA7引脚分配给SPI1
 // GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_SPI1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
  	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

   //PA5~PA7：推拉输出、下拉，50MHz刷新频率
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//PE5,PE6,PA4：推拉输出、上拉、50MHz刷新频率
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  	
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOE, GPIO_Pin_6 | GPIO_Pin_5);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
				
	
 	//SPI初始化，使用SPI连接MP25P32	
  	SPI_I2S_DeInit(SPI1);	 //必须先禁用,才能改变MODE 
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//两线全双工 
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//4分频 
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//先发最高位
  	SPI_InitStructure.SPI_CRCPolynomial = 7; //CRC多项式

  	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //SPI主模式
  	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
}

/********************************************************************
*sFLASH_Init                                                        *
*初始化相关标志及硬件。                                             *
********************************************************************/
void sFLASH_Init(void)
{
	M25P32_SPI1_Init();	
	M25P32_Init_Flag();
	sFLASH_ReadID();
}

/********************************************************************
*sFLASH_SendByte                                                    *
*发送一个字节。                                                     *
********************************************************************/
uint8_t sFLASH_SendByte(uint8_t byte)
{
	
	SPI_I2S_SendData(SPI1, byte);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI1);
}

/********************************************************************
*sFLASH_CS_LOW                                                      *
*片选置低。                                                         *
********************************************************************/
void sFLASH_CS_LOW(void)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_5);	
}

/********************************************************************
*sFLASH_CS_HIGH                                                     *
*片选置高。                                                         *
********************************************************************/
void sFLASH_CS_HIGH(void)
{
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
}

/********************************************************************
*sFLASH_ReadID                                                      *
*读取芯片id。                                                       *
********************************************************************/
uint32_t sFLASH_ReadID(void)
{
	uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
	uint16_t i;
   
	sFLASH_CS_LOW();
	for(i = 0;i < 4; i++);
	sFLASH_SendByte(0x9F);

	Temp0 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	Temp1 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	Temp2 = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
	
	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	for(i = 0;i < 4; i++);
	sFLASH_CS_HIGH();
	for(i = 0;i < 4; i++);
	
	return Temp;
}

/********************************************************************
*sFLASH_WriteEnable                                                 *
*开放写入权限。                                                     *
********************************************************************/
void sFLASH_WriteEnable(void)
{
	uint16_t i;
	sFLASH_CS_LOW();
	for(i = 0;i < 4; i++);
	SPI_I2S_SendData(SPI1, sFLASH_CMD_WREN);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	for(i = 0;i < 4; i++);
	sFLASH_CS_HIGH();
	for(i = 0;i < 4; i++);
}

/********************************************************************
*sFLASH_WaitForWriteEnd                                             *
*等待写入完成。                                                     *
********************************************************************/
void sFLASH_WaitForWriteEnd(void)
{
	uint8_t flashstatus = 0;
	uint16_t i;

	sFLASH_CS_LOW();
	for(i = 0;i < 4; i++);

	do
	{
		sFLASH_SendByte(sFLASH_CMD_RDSR);
		flashstatus = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
		IWDG->KR = 0xAAAA;//feed watchdog
	}
	while ((flashstatus & sFLASH_WIP_FLAG) == SET); /* Write in progress */
	sFLASH_CS_HIGH();
	for(i = 0;i < 4; i++);
}

/********************************************************************
*sFLASH_EraseSector                                                 *
*擦除扇区。                                                         *
********************************************************************/
void sFLASH_EraseSector(uint32_t SectorAddr)
{
	uint16_t i;
	sFLASH_WriteEnable();
 	for(i = 0;i < 20; i++);
 	sFLASH_CS_LOW();
 	for(i = 0;i < 4; i++);
	sFLASH_SendByte(sFLASH_CMD_SE);
	sFLASH_SendByte((SectorAddr & 0xFF0000) >> 16);
	sFLASH_SendByte((SectorAddr & 0xFF00) >> 8);
	sFLASH_SendByte(SectorAddr & 0xFF);
 	sFLASH_CS_HIGH();
	IWDG->KR = 0xAAAA;//feed watchdog
	sFLASH_WaitForWriteEnd();
}

/********************************************************************
*sFLASH_ReadBuffer                                                  *
*读取数据。                                                         *
********************************************************************/
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead)
{
	uint16_t i;
	for(i = 0;i < 20; i++);
	sFLASH_CS_LOW();
	sFLASH_SendByte(sFLASH_CMD_READ);
	sFLASH_SendByte((ReadAddr & 0xFF0000) >> 16);
	sFLASH_SendByte((ReadAddr& 0xFF00) >> 8);
	sFLASH_SendByte(ReadAddr & 0xFF);
	while (NumByteToRead--) /*!< while there is data to be read */
	{
		*pBuffer = sFLASH_SendByte(sFLASH_DUMMY_BYTE);
		pBuffer++;
	}
	for(i = 0;i < 20; i++);
	sFLASH_CS_HIGH();
	
}

/********************************************************************
*sFLASH_WritePage                                                   *
*写入一页(256byte)。                                                *
********************************************************************/
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	sFLASH_WriteEnable();
	sFLASH_CS_LOW();
	sFLASH_SendByte(sFLASH_CMD_WRITE);
	sFLASH_SendByte((WriteAddr & 0xFF0000) >> 16);
	sFLASH_SendByte((WriteAddr & 0xFF00) >> 8);
	sFLASH_SendByte(WriteAddr & 0xFF);

	while (NumByteToWrite--)
	{
		sFLASH_SendByte(*pBuffer);
		pBuffer++;
	}

	sFLASH_CS_HIGH();
	sFLASH_WaitForWriteEnd();
}

/******************************************************************** 
*sFLASH_WriteEnable_Cycle                                           *
*开放写入权限。                                                     *
********************************************************************/
void sFLASH_WriteEnable_Cycle(void)
{
	uint16_t i;
	sFLASH_CS_LOW();
	for(i = 0;i < 4; i++);
	SPI_I2S_SendData(SPI1, sFLASH_CMD_WREN);
}

/********************************************************************
*M25P32_EraseSector_Cycle                                           *
*擦除一个扇区。                                                     *
********************************************************************/
void M25P32_EraseSector_Cycle(void)
{
	int i;
	
	for(i = 0;i < 4; i++);
	sFLASH_CS_HIGH();
	for(i = 0;i < 40; i++);
	sFLASH_CS_LOW();
	for(i = 0;i < 4; i++);
	
	sFLASH_SendByte(sFLASH_CMD_SE);
	sFLASH_SendByte((M25P32_Write_Sector & 0xFF0000) >> 16);
	sFLASH_SendByte((M25P32_Write_Sector & 0xFF00) >> 8);
	sFLASH_SendByte(M25P32_Write_Sector & 0xFF);
 	sFLASH_CS_HIGH();
	for(i = 0;i < 4; i++);
	sFLASH_CS_LOW();
	for(i = 0;i < 4; i++);
}

/********************************************************************
*M25P32_ReInit                                                      *
*准备再次写入。                                                     *
********************************************************************/
void M25P32_ReInit(void)
{
	M25P32_Write_Addr_Once+=M25P32_Write_Length_Once;//目的地址后移
	M25P32_Write_Buffer_Once+=M25P32_Write_Length_Once;//数据地址后移
	if (M25P32_Write_Length-M25P32_Written_Length>256)//剩余内容超过一页
	{
		M25P32_Write_Length_Once=256;
	}
	else
	{
		M25P32_Write_Length_Once=M25P32_Write_Length-M25P32_Written_Length;
	}
	M25P32_Written_Length+=M25P32_Write_Length_Once;//已写入长度加上待写入长度
	sFLASH_WriteEnable_Cycle();
	M25P32_Write_Step=M25P32_Write_Step_Write;	
}

/********************************************************************
*sFLASH_WritePage_Cycle                                             *
*写入一页。                                                         *
********************************************************************/
void sFLASH_WritePage_Cycle(void)
{
	int i,j=M25P32_Write_Length_Once;
	unsigned char* ucS=M25P32_Write_Buffer_Once;
	
	for(i = 0;i < 4; i++);
	sFLASH_CS_HIGH();
	for(i = 0;i < 4; i++);
	sFLASH_CS_LOW();
	sFLASH_SendByte(sFLASH_CMD_WRITE);
	sFLASH_SendByte((M25P32_Write_Addr_Once & 0xFF0000) >> 16);
	sFLASH_SendByte((M25P32_Write_Addr_Once & 0xFF00) >> 8);
	sFLASH_SendByte(M25P32_Write_Addr_Once & 0xFF);

	while (j--)
	{
		sFLASH_SendByte(*ucS++);
	}

	sFLASH_CS_HIGH();
	for(i = 0;i < 4; i++);
	sFLASH_CS_LOW();
	for(i = 0;i < 4; i++);
}

/********************************************************************
*M25P32_Write_Verify                                                *
*读取Flash内容，并与源数据比较校验。                                *
********************************************************************/
unsigned char M25P32_Write_Verify(void)
{
	int i;
	unsigned char* ucD=M25P32_Write_Buffer_Once;
	
	sFLASH_ReadBuffer(ucV, M25P32_Write_Addr_Once, M25P32_Write_Length_Once);
	
	for (i=0;i<M25P32_Write_Length_Once;i++)
	{
		if (ucV[i]!=*ucD++)
		{
			break;
		}
	}
	return i==M25P32_Write_Length_Once;//校验结果: 1-pass, 0-fail
}

/********************************************************************
*M25P32_Write_Cycle                                                 *
*多循环无等待写flash。                                              *
********************************************************************/
void M25P32_Write_Cycle(void)   
{ 
	unsigned char ucT;
	
	if (M25P32_Write_Flag)
	{		
		if (M25P32_Write_Flag!=M25P32_Write_Old_Flag)//写入初始化
		{
			M25P32_Write_Old_Flag=M25P32_Write_Flag;
			M25P32_Write_Offset=M25P32_Write_Addr%256;					//在页内的偏移
			M25P32_Write_Length_Once=256-M25P32_Write_Offset;			//页内剩余空间大小
			if (M25P32_Write_Length_Once>M25P32_Write_Length)
			{   
				M25P32_Write_Length_Once=M25P32_Write_Length;
			}
			M25P32_Written_Length=M25P32_Write_Length_Once;
			M25P32_Write_Addr_Once=M25P32_Write_Addr;					//初始写入地址
			M25P32_Write_Buffer_Once=M25P32_Write_Buffer;				//初始写入地址
			M25P32_Write_Sector=M25P32_Write_Addr-M25P32_Write_Addr%65536;//所在扇区起始地址
			M25P32_Write_Step=M25P32_Write_Step_Write_Enable;
			M25P32_Write_Result=1;
			sFLASH_WriteEnable_Cycle();
		}
		
		switch (M25P32_Write_Step)
		{
			
			case M25P32_Write_Step_Write_Enable:
				if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) != RESET)//等待写入允许
				{					
					M25P32_EraseSector_Cycle();
					M25P32_Write_Step=M25P32_Write_Step_Erase;
				}
				break;
			case M25P32_Write_Step_Erase:
				sFLASH_SendByte(sFLASH_CMD_RDSR);
				ucT=sFLASH_SendByte(sFLASH_DUMMY_BYTE);
				if ((ucT & sFLASH_WIP_FLAG) == RESET)//等待擦除完成
				{
					sFLASH_CS_HIGH();
					for(ucT = 0;ucT < 4; ucT++);
					sFLASH_WriteEnable_Cycle();
					
					M25P32_Write_Sector+=65536;//下一扇区起始地址
					if ( M25P32_Write_Sector<=(M25P32_Write_Addr+M25P32_Write_Length) )//是否擦完
					{
						M25P32_Write_Step=M25P32_Write_Step_Write_Enable;
					}
					else//擦完 
					{
						M25P32_Write_Step=M25P32_Write_Step_Write;					
					}
				}
				break;
			case M25P32_Write_Step_Write:
				if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == SET)//等待写入允许
				{					
					sFLASH_WritePage_Cycle();
 					M25P32_Write_Step=M25P32_Write_Step_Verify;
				}
				break;
			case M25P32_Write_Step_Verify:
				sFLASH_SendByte(sFLASH_CMD_RDSR);
				ucT=sFLASH_SendByte(sFLASH_DUMMY_BYTE);
				if ((ucT & sFLASH_WIP_FLAG) == RESET)//等待写入完成
				{
					sFLASH_CS_HIGH();
					for(ucT = 0;ucT < 4; ucT++);
					ucT=M25P32_Write_Verify();
					if (ucT)//校验通过
					{
						if (M25P32_Written_Length<M25P32_Write_Length)//没有写完
						{
							M25P32_ReInit();
						}
						else
						{
							M25P32_Init_Flag();	
							M25P32_Write_Result=0;
						}
					}
					else
					{
						M25P32_Init_Flag();	
						M25P32_Write_Result=1;						
					}
					//Upload_Ref(1+M25P32_Write_Result+M25P32_Write_Result);
				}
				break;
		}		
	}		
}

/********************************************************************
*Start_sFlash_Write                                                 *
*启动M25P32写入循环。                                               *
*Write_Addr: 写入地址, Write_Buffer:写入数据指针, Write_Length: 长度*
********************************************************************/
unsigned char Start_sFlash_Write(int Write_Addr, unsigned char* Write_Buffer, int Write_Length)
{
	M25P32_Write_Addr=Write_Addr;
	M25P32_Write_Buffer=Write_Buffer;
	M25P32_Write_Length=Write_Length;
	M25P32_Write_Flag=1;//启动循环
//	return M25P32_Write_Data();
	return 0;
}

/********************************************************************
*M25P32_Write_Test                                                  *
*多循环无等待写flash范例。                                          *
********************************************************************/
void M25P32_Write_Test(void)
{
	int i;
	
// 	for (i=0;i<256;i++)
// 	{
// 		ucW[i]=i;
// 		ucW[i+256]=i;
// 	}
// 	sFLASH_ReadBuffer(ucV, M25P32_Write_Addr, M25P32_Write_Length);
// 	

	M25P32_Write_Flag=1;
	M25P32_Write_Addr=0;
	M25P32_Write_Buffer=ucW;
	M25P32_Write_Length=256;
// 	sFLASH_WritePage(ucW,M25P32_Write_Addr,M25P32_Write_Length);
// 	sFLASH_ReadBuffer(ucV, M25P32_Write_Addr, M25P32_Write_Length);
	sFLASH_EraseSector(M25P32_Write_Addr);
	for (i=0;i<256;i++)
	{
		ucW[i]=255-i;
		ucW[i+256]=i;
	}
	sFLASH_ReadBuffer(ucV, M25P32_Write_Addr, M25P32_Write_Length);
	sFLASH_WritePage(ucW,M25P32_Write_Addr,M25P32_Write_Length);
	sFLASH_WritePage(ucW+256,M25P32_Write_Addr+256,M25P32_Write_Length);
	sFLASH_ReadBuffer(ucV, M25P32_Write_Addr, M25P32_Write_Length+256);
	for (i=0;i<256;i++)
	{
		ucW[i]=255-i;
		ucW[i+256]=i;
	}
	M25P32_Write_Length=512;
	
	ucW[0]=2;
	ucW[1]=3;
	ucW[2]=5;
	ucW[3]=7;
	ucW[4]=9;
	ucW[5]=11;
	ucW[6]=13;
	ucW[7]=17;
	ucW[8]=19;
	M25P32_Write_Addr=65533;
	M25P32_Write_Length=9;
}

/********************************************************************
*EEROM_Write                                                        *
*阻塞式写入。                                                       *
********************************************************************/
unsigned char EEROM_Write(int Write_Addr, unsigned char* Write_Buffer, int Write_Length)
{
	M25P32_Write_Addr=Write_Addr;
	M25P32_Write_Buffer=Write_Buffer;
	M25P32_Write_Length=Write_Length;
	
	M25P32_Write_Buffer_Once=M25P32_Write_Buffer;
	M25P32_Write_Offset=Write_Addr%256;							//在页内的偏移
	M25P32_Write_Length_Once=256-M25P32_Write_Offset;			//页内剩余空间大小
	if (M25P32_Write_Length_Once>M25P32_Write_Length)
	{   
		M25P32_Write_Length_Once=M25P32_Write_Length;
	}
	M25P32_Written_Length=M25P32_Write_Length_Once;
	M25P32_Write_Addr_Once=M25P32_Write_Addr;					//初始写入地址
	M25P32_Write_Buffer_Once=M25P32_Write_Buffer;				//初始写入地址
	M25P32_Write_Sector=M25P32_Write_Addr-M25P32_Write_Addr%65536;//所在扇区起始地址
	IWDG->KR = 0xAAAA;//feed watchdog
	sFLASH_EraseSector(M25P32_Write_Addr);
	IWDG->KR = 0xAAAA;
	sFLASH_WritePage(M25P32_Write_Buffer_Once,M25P32_Write_Addr,M25P32_Write_Length_Once);
	IWDG->KR = 0xAAAA;
	M25P32_Write_Result=M25P32_Write_Verify();
	IWDG->KR = 0xAAAA;
	return 1-M25P32_Write_Result;//0:pass, 1:fail
}
//===========================================================================
// End of file.
//===========================================================================
