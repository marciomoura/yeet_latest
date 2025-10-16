/**************************************************************************************************
 * @file    w5200.h                                                                                *
 * @author  ���´����                                                                             *
 * @version V1.0.0                                                                                 *
 * @date    2018.7.25.                                                                             *
 * @brief   w5200��������                                                                          *
 **************************************************************************************************/
#include <stm32f4xx.h>

#define DMA_SIZE_1K 1024
#define DMA_SIZE_2K 2048
#define DMA_SIZE_4K 4096
#define DMA_SIZE_8K 8192

extern unsigned char Net_Socket_State[2][2];
extern unsigned char Net_Tx_Flag[2][2];    // ���ͱ�־, ���ⲿ��������
extern unsigned char Net_Rx_Flag[2][2];    // �������յ���־, ���ⲿ������
extern unsigned char Net_Rx_Buf[2][8192];  // ���ջ���, ���ⲿ������
// extern unsigned char Net_Tx_Buf[2][8192];//���ͻ���, ���ⲿ������
extern unsigned short int Net_Tx_size[2][2];
extern unsigned short int Net_Rx_size[2];

#define COMMON_BASE 0x0000
#define CH_SIZE     0x0100
#define S_BUF_SIZE  8192

#define DMA_SIZE_128 128
#define DMA_SIZE_256 256
#define DMA_SIZE_512 512
#define DMA_SIZE_1K  1024
#define DMA_SIZE_2K  2048
#define DMA_SIZE_4K  4096
#define DMA_SIZE_8K  8192

#define GAR0              (COMMON_BASE + 0x0001)
#define SUBR0             (COMMON_BASE + 0x0005)
#define SHAR0             (COMMON_BASE + 0x0009)
#define SIPR0             (COMMON_BASE + 0x000F)
#define IR2               (COMMON_BASE + 0x0034)
#define Sn_PORT0(ch)      (CH_BASE + ch * CH_SIZE + 0x0004)
#define Sn_DIPR0(ch)      (CH_BASE + ch * CH_SIZE + 0x000C)
#define Sn_DPORT0(ch)     (CH_BASE + ch * CH_SIZE + 0x0010)
#define TX_MEM            COMMON_BASE + 0x8000
#define RX_MEM            COMMON_BASE + 0xc000
#define CH_BASE           (COMMON_BASE + 0x4000)
#define Sn_RXMEM_SIZE(ch) (CH_BASE + ch * CH_SIZE + 0x001E)
#define Sn_TXMEM_SIZE(ch) (CH_BASE + ch * CH_SIZE + 0x001F)
#define Sn_MSSR0(ch)      (CH_BASE + ch * CH_SIZE + 0x0012)
#define Sn_RX_RSR0(ch)    (CH_BASE + ch * CH_SIZE + 0x0026)
#define Sn_RX_RD0(ch)     (CH_BASE + ch * CH_SIZE + 0x0028)
#define RTR_W             (COMMON_BASE + 0x0017)
#define RCR               (COMMON_BASE + 0x0019)
#define Sn_MR(ch)         (CH_BASE + ch * CH_SIZE + 0x0000)
#define Sn_MR_TCP         0x01
#define Sn_CR(ch)         (CH_BASE + ch * CH_SIZE + 0x0001)
#define Sn_CR_OPEN        0x01
#define Sn_CR_LISTEN      0x02
#define Sn_CR_CONNECT     0x04
#define Sn_CR_DISCON      0x08
#define Sn_CR_CLOSE       0x10
#define Sn_CR_SEND        0x20
#define Sn_CR_RECV        0x40

#define Sn_SR(ch) (CH_BASE + ch * CH_SIZE + 0x0003)
#define Sn_IR(ch) (CH_BASE + ch * CH_SIZE + 0x0002)
#define Sn_IR_CON 0x01

#define Sn_TX_FSR0(ch) (CH_BASE + ch * CH_SIZE + 0x0020)
#define Sn_TX_WR0(ch)  (CH_BASE + ch * CH_SIZE + 0x0024)

#define SOCK_CLOSED 0x00
#define SOCK_INIT   0x13
#define SOCK_LISTEN 0x14

#define S_CR_DISCON    0x08
#define S_CR_SEND_KEEP 0x22

#define S_INIT 0x01
#define S_CONN 0x02
#define S_OK   0x04

#define S_IDLING    0x00  // �˿ڴ��ڿ�����
#define S_CHECKING  0x01  // �˿ڼ��״̬��
#define S_SENDING   0x02  // �˿ڷ���������
#define S_RECEIVING 0x03  // �˿ڽ���������

#define S_RECEIVE    0x01
#define S_TRANSMITOK 0x02

#define S_IR_CON     0x01
#define S_IR_DISCON  0x02
#define S_IR_RECV    0x04
#define S_IR_TIMEOUT 0x08
#define S_IR_SENDOK  0x10

#define TRUE    0xff
#define FALSE   0x00
#define Server  1  // ������
#define Client  0  // �ͻ���
#define Failure 0
#define Success 1

#define SPI_Delay 1200

unsigned char Network_Sub_Mask[4] = {255, 255, 255, 0};  // ��������
unsigned char Network_Phy_Addr[6];                       // MAC��ַ

unsigned char Net_Check_Delay[2] = {0};

unsigned short int Net_Rx_Addr[2] = {0xc000, 0xe000};
unsigned short int Net_Tx_Addr[2] = {0x8000, 0xa000};
unsigned short int Net_Socket_Rx_Addr[2][2];
unsigned short int Net_Socket_Tx_Addr[2][2];
unsigned char Net_Enable[2][2] = {0};  // Socket�Ƿ�����
unsigned char Net_Mode[2][2] = {0};    // 0:Client 1:Server
unsigned char Net_Socket_State[2][2];
unsigned char Net_Socket_Data[2][2];
unsigned char Net_State[2];
unsigned char Net_Tx_Flag[2][2];
unsigned char Net_Rx_Flag[2][2];
unsigned short int Net_Rx_size[2];
unsigned short int Net_Tx_size[2][2];
unsigned short int Net_Tx_Addr_Offset[2][2];
unsigned short int Net_Rx_Addr_Offset[2][2];
unsigned char Net_Rx_Buf[2][8192];
// unsigned char Net_Tx_Buf[2][8192];

unsigned char SPI_Tx_Buf[2][8196];
unsigned char SPI_Rx_Buf[2][8196];
unsigned char Net_Receiving_Socket[2];  // ��ǰ���ڽ��յ������׽���
unsigned char Net_Sending_Socket[2];    // ��ǰ���ڽ��յ������׽���
unsigned short int Net_Rx_Delay[2];
unsigned short int Net_Tx_Delay[2];

int DMA_Tx_Size[2];

unsigned char Para_Self_IP[2][4];
unsigned char Para_Gateway[2][4];
unsigned char Para_Socket_Enable[2][2];
unsigned short int Para_Self_Port[2][2];
unsigned char Para_Socket_Mode[2][2];
unsigned char Para_Des_IP[2][8];
unsigned short int Para_Des_Port[2][2];
int Para_DMA_Size[2];

unsigned short int Net_Port[2][8];

void Network_Init(SPI_TypeDef* SPIx,
    unsigned char* Self_IP,
    unsigned char* Socket_Enable,
    unsigned short int* Self_Port,
    unsigned char* Socket_mode,
    unsigned char* Dest_IP,
    unsigned short int* Dest_Port,
    int DMA_Size);  // ���ڳ�ʼ��
void Soft_Delay(unsigned int x);

void Net_Comm(SPI_TypeDef* SPIx, unsigned char* TxData);
//===========================================================================
// End of file.
//===========================================================================
