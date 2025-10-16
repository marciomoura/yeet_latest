/********************************************************************
*M25P32.h                                                           *
*参数固化与读取, 有阻塞式和非阻塞式两种写入.                        *
********************************************************************/
#ifndef _M25P32_H_
#define _M25P32_H_

//extern unsigned char M25P32_Write_Result;		//写入后的校验结果: 0->ok, 1->fail, 外部调用 

void sFLASH_Init(void);
unsigned int sFLASH_ReadID(void);
void sFLASH_EraseSector(unsigned int SectorAddr);
void sFLASH_ReadBuffer(unsigned char* pBuffer, unsigned int ReadAddr, unsigned short int NumByteToRead);
void sFLASH_WritePage(unsigned char* pBuffer, unsigned int WriteAddr, unsigned short int NumByteToWrite);
void M25P32_Write_Cycle(void);
void sFLASH_Init(void);
void M25P32_Write_Test(void);
unsigned char EEROM_Write(int Write_Addr, unsigned char* Write_Buffer, int Write_Length);
#endif

//===========================================================================
// End of file.
//===========================================================================
