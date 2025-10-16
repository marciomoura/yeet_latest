/**************************************************************************************************
* @file    w5200.h                                                                                *
* @author  能事达电气                                                                             *
* @version V1.0.0                                                                                 *
* @date    2018.7.25.                                                                             *
* @brief   w5200网口驱动                                                                          *
**************************************************************************************************/
#include <stm32f4xx.h>

#define DMA_SIZE_1K 1024
#define DMA_SIZE_2K 2048
#define DMA_SIZE_4K 4096
#define DMA_SIZE_8K 8192

extern unsigned char Net_Socket_State[2][2];  
extern unsigned char Net_Tx_Flag[2][2];//发送标志, 供外部程序设置 
extern unsigned char Net_Rx_Flag[2][2];//数据已收到标志, 供外部程序检测
extern unsigned char Net_Rx_Buf[2][8192];//接收缓冲, 供外部程序处理
//extern unsigned char Net_Tx_Buf[2][8192];//发送缓冲, 供外部程序处理
extern unsigned short int Net_Tx_size[2][2];
extern unsigned short int Net_Rx_size[2];

void Network_Init(SPI_TypeDef *SPIx, unsigned char* Self_IP, unsigned char* Socket_Enable, unsigned short int* Self_Port, 
unsigned char* Socket_mode, unsigned char* Dest_IP, unsigned short int* Dest_Port, int DMA_Size);//网口初始化 
void Soft_Delay(unsigned int x);

void Net_Comm(SPI_TypeDef *SPIx, unsigned char* TxData);
//===========================================================================
// End of file.
//===========================================================================
