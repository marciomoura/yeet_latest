/***************************************************************************************
*文件名称   usarts.h  
*作者  		Three Gorges Nengshida Electric Co.,Ltd
*版本 		NSD_STM32F2_Driver_1.0
*修订日期   2014-07-10
*备注  		串口通信(RS232，RS485)
***************************************************************************************/
/************************************************************************************************
*串口驱动使用说明																				*
*																								*
*初始化函数																						*
*串口1：Init_RS232_1(波特率，发送字节，触发接收标志的字节数)									*
*串口2：Init_RS232_2(波特率，发送字节，触发接收标志的字节数)									*
*串口3：Init_RS485(波特率，发送字节，触发接收标志的字节数)										*
*波特率不得大于7500000，发送字节＜512，接收数据<256。											*
*																								*
*接收函数																						*
*串口1：FlagStatus RS232_Rx_1(uint8_t* RxData)													*
*串口2：FlagStatus RS232_Rx_2(uint8_t* RxData)													*
*串口3：FlagStatus RS485_Rx(uint8_t* RxData)													*
*接收缓冲中数据＞触发事件的字节则返回1， RxData是用户的接收缓冲。								*
*																								*
*串口1：void RS232_1_Tx(void)																	*
*串口2：void RS232_2_Tx(void)																	*
*串口3：void RS485_Tx(void)																		*
*只写全局变量RS232_1_Tx_Flag/RS232_2_Tx_Flag/RS485_Tx_Flag，用户置位后启动发送，启动后自动清零。*
*只读全局变量RS232_1_Tx_Enable_Flag/RS232_2_Tx_Enable_Flag/RS485_Tx_Enable_Flag，启动发送后     *
*自动清零，发送完成后置位。为0时禁止修改发送数据区。Usart1_Tx_Data/Usart2_Tx_Data是发送数据区。	*				
************************************************************************************************/

/* Define to prevent recursive inclusion ---------------------------------------------*/
#ifndef __USARTx_H
#define __USARTx_H

#define Rx_Buf_Size 512
#define Tx_Buf_Size 512 

extern uint8_t RS232_1_Tx_Flag,RS232_2_Tx_Flag;
extern uint8_t RS485_Tx_Flag;
extern uint8_t RS232_1_Tx_Enable_Flag,RS232_2_Tx_Enable_Flag;
extern uint8_t RS485_Tx_Enable_Flag;
extern uint8_t Usart1_Tx_Data[Tx_Buf_Size];
extern uint8_t Usart1_Rx_Data[Tx_Buf_Size];
extern uint8_t Usart2_Tx_Data[Rx_Buf_Size];
extern uint8_t Usart2_Rx_Data[Rx_Buf_Size];
extern uint8_t Usart3_Tx_Data[Rx_Buf_Size];
extern uint8_t Usart3_Rx_Data[Rx_Buf_Size];

/***************************************************************************************
*函数名：Init_RS485																	*
*功能：初始化串口3(RS485)																	*
*输入参数：BaudRate 设置波特率														*
		   Tx_Num  发送数据长度														*
		   Rx_Threshold  串口接收缓冲区收到的数据大于等于Rx_Threshold时，读取数据	*
***************************************************************************************/
void Init_RS485(uint32_t BaudRate, uint32_t Tx_Num, uint32_t Rx_Threshold);

/***************************************************************************************
*函数名：RS485_Rx																*
*功能：串口2接收函数															*
*输入参数：RxData 数据接收缓冲区												*
*返回值：1：接收成功，0：接收失败												*
***************************************************************************************/
FlagStatus RS485_Rx(uint8_t* RxData);

/***************************************************************************************
*函数名：RS485_Tx																	*
*功能：串口2发送数据区内全部数据 										  			*
***************************************************************************************/
void RS485_Tx(void);

/***************************************************************************************
*函数名：Init_RS232_1																*
*功能：初始化串口1															*
*输入参数：	BaudRate 设置波特率,参数范围：0~115200									*
			parity   设置校验位,输入参数：											*
		        USART_Parity_No     无校验											*
				USART_Parity_Even   偶校验											*
				USART_Parity_Odd    奇校验                                         	*
			wordlength  设置数据位长度,输入参数：		                            *
				USART_WordLength_8b    8个数据位									*
				USART_WordLength_9b    9个数据位									*
			stopbits	设置停止位长度,输入参数：	                                *
				USART_StopBits_1	1个停止位 										*     
				USART_StopBits_0_5	0.5个停止位 									*	
				USART_StopBits_1_5	1.5个停止位 									*     
				USART_StopBits_2	2个停止位 									    *	
***************************************************************************************/
void Init_RS232_1(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits);

/***************************************************************************************
*函数名：Init_RS232_2																*
*功能：初始化串口2															*
*输入参数：	BaudRate 设置波特率,参数范围：0~115200									*
			parity   设置校验位,输入参数：											*
		        USART_Parity_No     无校验											*
				USART_Parity_Even   偶校验											*
				USART_Parity_Odd    奇校验                                         	*
			wordlength  设置数据位长度,输入参数：		                            *
				USART_WordLength_8b    8个数据位									*
				USART_WordLength_9b    9个数据位									*
			stopbits	设置停止位长度,输入参数：	                                *
				USART_StopBits_1	1个停止位 										*     
				USART_StopBits_0_5	0.5个停止位 									*	
				USART_StopBits_1_5	1.5个停止位 									*     
				USART_StopBits_2	2个停止位 									    *	
***************************************************************************************/
void Init_RS232_2(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits);


void Init_RS232_2_NVIC(uint32_t baudrate,uint16_t parity,uint16_t wordlength,uint16_t stopbits);
/***************************************************************************************
*函数名：RS232_Rx																*
*功能：串口1接收函数															*
*输入参数：RxData 数据接收缓冲区												*
*返回值：1：接收成功，0：接收失败												*
***************************************************************************************/
uint16_t RS232_1_Rx(uint8_t* RxData);

/***************************************************************************************
*函数名：RS232_2_Rx																*
*功能：串口2接收函数															*
*输入参数：RxData 数据接收缓冲区												*
*返回值：1：接收成功，0：接收失败												*
***************************************************************************************/
uint16_t RS232_2_Rx(uint8_t* RxData);	

/***************************************************************************************
*函数名：RS232_1_Tx																	*
*功能：串口1发送数据区内全部数据 											*
***************************************************************************************/
uint8_t RS232_1_Tx(uint8_t tx_data[], uint16_t len)	;

/***************************************************************************************
*函数名：RS232_2_Tx																	*
*功能：串口2发送数据区内全部数据 											*
***************************************************************************************/
uint8_t RS232_2_Tx(uint8_t tx_data[], uint16_t len);


#endif


/******* (C) COPYRIGHT 2013 Three Gorges Nengshida Electric Co.,Ltd *****END OF FILE***/

