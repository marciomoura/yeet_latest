/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USARTx_H
#define __USARTx_H

//#define TXBUFFERSIZE   0x32
//#define RXBUFFERSIZE   0x32
void Init_RS485(unsigned int BaudRate, unsigned int Tx_Num, unsigned int Rx_Threshold);
unsigned char RS485_Rx(unsigned char* RxData);
void RS485_Tx(unsigned char* RS485_Tx_Flag, unsigned char* RS485_Tx_Data);

void Init_RS232(unsigned int BaudRate, unsigned int Tx_Num, unsigned int Rx_Threshold);
unsigned char RS232_Rx(unsigned char* RxData);
void RS232_Tx(unsigned char* RS232_Tx_Flag, unsigned char* RS232_Tx_Data);

#endif

