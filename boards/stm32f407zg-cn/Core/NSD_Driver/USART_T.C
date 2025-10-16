#include "main.h"
/*********************************************************************************************************
** USART��ʼ��
*********************************************************************************************************/
/*
 * @brief
 * @param  None
 * @retval None
 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/************************************************************************************************
 *��������ʹ��˵��
 **
 *																								*
 *��ʼ������
 ** ����1��Init_RS232_1(�����ʣ������ֽڣ��������ձ�־���ֽ���) * ����2��Init_RS232_2(�����ʣ������ֽڣ��������ձ�־���ֽ���)
 ** ����3��Init_RS485(�����ʣ������ֽڣ��������ձ�־���ֽ���) * �����ʲ��ô���7500000�������ֽڣ�512����������<256��
 **
 *																								*
 *���պ���
 ** ����1��FlagStatus RS232_Rx_1(uint8_t* RxData)
 ** ����2��FlagStatus RS232_Rx_2(uint8_t* RxData)
 ** ����3��FlagStatus RS485_Rx(uint8_t* RxData)
 **
 *���ջ��������ݣ������¼����ֽ��򷵻�1��
 *RxData���û��Ľ��ջ��塣
 **
 *																								*
 *����1��void RS232_1_Tx(void)
 ** ����2��void RS232_2_Tx(void)
 ** ����3��void RS485_Tx(void)
 ** ֻдȫ�ֱ���RS232_1_Tx_Flag/RS232_2_Tx_Flag/RS485_Tx_Flag���û���λ���������ͣ��������Զ����㡣*
 *ֻ��ȫ�ֱ���RS232_1_Tx_Enable_Flag/RS232_2_Tx_Enable_Flag/RS485_Tx_Enable_Flag���������ͺ�     *
 *�Զ����㣬������ɺ���λ��Ϊ0ʱ��ֹ�޸ķ�����������Usart1_Tx_Data/Usart2_Tx_Data�Ƿ�����������
 **
 ************************************************************************************************/
#include <stm32f4xx.h>

#define Tx_Buf_Size 512
#define Rx_Buf_Size 512

uint8_t Usart1_Tx_Data[Tx_Buf_Size];
uint8_t Usart1_Rx_Data[Tx_Buf_Size];
uint8_t Usart2_Tx_Data[Rx_Buf_Size];
uint8_t Usart2_Rx_Data[Rx_Buf_Size];
uint8_t Usart3_Tx_Data[Rx_Buf_Size];
uint8_t Usart3_Rx_Data[Rx_Buf_Size];
uint16_t RS232_1_Rx_Count, RS232_2_Rx_Count;
uint16_t RS485_Rx_Count;
uint32_t RS232_1_Tx_Num;
uint32_t RS232_1_Rx_Threshold;
uint32_t RS485_Tx_Num;
uint32_t RS485_Rx_Threshold;
uint8_t RS232_1_Tx_Flag, RS232_2_Tx_Flag;
uint8_t RS232_1_Tx_Enable_Flag, RS232_2_Tx_Enable_Flag;
uint8_t RS485_Tx_Flag;
uint8_t RS485_Tx_Enable_Flag;

/************************************************************************************
 *RS232_1_Init_Para
 ** ��ʼ��COM1����������ز�����
 **
 ************************************************************************************/
void RS232_1_Init_Para(void)
{
    RS232_1_Rx_Count = 0;
    RS232_1_Tx_Flag = 0;
    RS232_1_Tx_Enable_Flag = 1;
}

/************************************************************************************
 *RS232_2_Init_Para
 ** ��ʼ��COM2����������ز�����
 **
 ************************************************************************************/
void RS232_2_Init_Para(void)
{
    RS232_2_Rx_Count = 0;
    RS232_2_Tx_Flag = 0;
    RS232_2_Tx_Enable_Flag = 1;
    RS485_Rx_Count = 0;
}

/************************************************************************************
 *RS485_Init_Para
 ** ��ʼ��COM3����������ز�����
 **
 ************************************************************************************/
void RS485_Init_Para(uint32_t Tx_Num, uint32_t Rx_Threshold)
{
    uint32_t i;

    for (i = 0; i < Tx_Buf_Size; i++) {
        Usart3_Tx_Data[i] = (uint8_t)(i & 0xff);
    }
    for (i = 0; i < Rx_Buf_Size; i++) {
        Usart3_Rx_Data[i] = 0;
    }
    RS485_Rx_Count = 0;  // �������Ѵ������ݵ�ָ��
    RS485_Tx_Flag = 0;
    RS485_Tx_Enable_Flag = 1;
    RS485_Tx_Num = Tx_Num;
    RS485_Rx_Threshold = Rx_Threshold;
}

/************************************************************************************
 *RS232_1_Init_Pin
 ** ��ʼ��RS232������š�
 ** USART1, Tx-PA9, Rx-PA10��
 **
 ************************************************************************************/
void RS232_1_Init_Pin()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // ����USART1ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   // ����USART1��Rx��Tx�Լ�USART2��Tx����ʱ��

    // USART1:Tx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // PA9ָ����USART1
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART1:Rx
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  // PA10ָ����USART1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/************************************************************************************
 *RS232_2_Init_Pin
 ** ��ʼ��RS232������š�
 ** USART2, Tx-PB10, Rx-PB11��
 **
 ************************************************************************************/
void RS232_2_Init_Pin()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  // ����USART3ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);   // ����USART3��Tx��Rx����ʱ��

    // USART3:Tx
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);  // PB10ָ����USART3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3:Rx
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  // PB11ָ����USART3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/************************************************************************************
 *RS485_Init_Pin
 ** ��ʼ��RS485������š�
 ** USART2, Tx-PD5, Rx-PD6��
 **
 ************************************************************************************/
void RS485_Init_Pin(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // ����USART2ʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);   // ����USART2��Tx��Rx����ʱ��

    // USART2:Tx
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);  // PD5ָ����USART2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // USART2:Rx
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);  // PD6ָ����USART2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/************************************************************************************
 *RS232_1_Init_Port
 ** ��ʼ��COM1�����ò����ʡ�
 **
 *************************************************************************************
 *BaudRate��0~7,500,000
 ** Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd						*
 *WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b								*
 *StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2/USART_StopBits_1_5	*
 ************************************************************************************/
void RS232_1_Init_Port(uint32_t baudrate, uint16_t parity, uint16_t wordlength, uint16_t stopbits)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_WordLength = wordlength;
    USART_InitStructure.USART_StopBits = stopbits;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ����Ӳ��������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);  // �򿪴���
}
/************************************************************************************
 *RS232_2_Init_Port
 ** ��ʼ��COM2�����ò����ʡ�
 **
 *************************************************************************************
 *BaudRate��0~7,500,000
 ** Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd						*
 *WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b								*
 *StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2/USART_StopBits_1_5	*
 ************************************************************************************/
void RS232_2_Init_Port(uint32_t baudrate, uint16_t parity, uint16_t wordlength, uint16_t stopbits)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_WordLength = wordlength;
    USART_InitStructure.USART_StopBits = stopbits;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ����Ӳ��������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    USART_Cmd(USART3, ENABLE);  // �򿪴���
}
/************************************************************************************
 *RS485_Init_Port
 ** ��ʼ��COM3�����ò����ʡ�
 **
 *************************************************************************************
 *BaudRate��0~7,500,000
 ** Parity: USART_Parity_No, USART_Parity_Even, USART_Parity_Odd						*
 *WORD_LENGTH: USART_WordLength_8b, USART_WordLength_9b								*
 *StopBits: USART_StopBits_1/USART_StopBits_0_5/USART_StopBits_2/USART_StopBits_1_5	*
 ************************************************************************************/
void RS485_Init_Port(uint32_t BaudRate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = BaudRate;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  // ����Ӳ��������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_Cmd(USART2, ENABLE);  // �򿪴���
}

/************************************************************************************
 *RS232_1_Init_DMA
 ** ��ʼ������1��RS232����DMA���͹��ܡ�
 ** USART1_Tx-DMA2_Stream7_Channel4
 **
 ************************************************************************************/
void RS232_1_Init_DMA_Tx(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    // DMA_DeInit(DMA2_Stream7);		//��������������������״̬
    DMA_StructInit(&DMA_InitStruct);  // ȱʡֵ����ʼ������
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)Usart1_Tx_Data;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;  // �������ڴ浽����
    DMA_InitStruct.DMA_BufferSize = 512;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �ڴ��ַ����
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_MemoryBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream7, &DMA_InitStruct);
    // DMA_Cmd(DMA2_Stream7, ENABLE);
}

/************************************************************************************
 *RS232_2_Init_DMA
 ** ��ʼ������2��RS232����DMA���͹��ܡ�
 ** USART3_Tx-DMA1_Stream3_Channel4	                                       *
 ************************************************************************************/
void RS232_2_Init_DMA_Tx(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    //	DMA_DeInit(DMA1_Stream3);		//��������������������״̬
    DMA_StructInit(&DMA_InitStruct);  // ȱʡֵ����ʼ������
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)Usart2_Tx_Data;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;  // �������ڴ浽����
    DMA_InitStruct.DMA_BufferSize = 512;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �ڴ��ַ����
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_MemoryBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream3, &DMA_InitStruct);
    // DMA_Cmd(DMA1_Stream3, ENABLE);
}

/************************************************************************************
 *RS232_1_Init_DMA_Rx
 ** ��ʼ������1��RS232����DMA���չ��ܡ�
 ** USART1_Rx-DMA2_Stream5_Channel4
 **
 ************************************************************************************/
void RS232_1_Init_DMA_Rx(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    DMA_DeInit(DMA2_Stream5);         // ��������������������״̬
    DMA_StructInit(&DMA_InitStruct);  // ȱʡֵ����ʼ������
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)Usart1_Rx_Data;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;  // ���������赽�ڴ�
    DMA_InitStruct.DMA_BufferSize = Rx_Buf_Size;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �ڴ��ַ����
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;          // ѭ��Ѱַ
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA2_Stream5, &DMA_InitStruct);
    DMA_Cmd(DMA2_Stream5, ENABLE);  // ����DMA������
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
}
/************************************************************************************
 *RS232_2_Init_DMA_Rx
 ** ��ʼ������2��RS232����DMA���չ��ܡ� * USART3_Rx-DMA1_Stream1_Channel4
 **
 ************************************************************************************/
void RS232_2_Init_DMA_Rx(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    DMA_DeInit(DMA1_Stream1);         // ��������������������״̬
    DMA_StructInit(&DMA_InitStruct);  // ȱʡֵ����ʼ������
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)Usart2_Rx_Data;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;  // ���������赽�ڴ�
    DMA_InitStruct.DMA_BufferSize = Rx_Buf_Size;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;          // ѭ��Ѱַ
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �ڴ��ַ����
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Stream1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Stream1, ENABLE);  // ����DMA������
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}
/************************************************************************************
 *RS485_Init_DMA
 ** ��ʼ��RS485��DMA���͹��ܡ�
 ** USART2_Tx-DMA1_Stream6_Channel4
 **
 ************************************************************************************/
void RS485_Init_DMA_Tx(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    DMA_DeInit(DMA1_Stream6);         // ��������������������״̬
    DMA_StructInit(&DMA_InitStruct);  // ȱʡֵ����ʼ������
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)Usart3_Tx_Data;
    DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;  // �������ڴ浽����
    DMA_InitStruct.DMA_BufferSize = RS485_Tx_Num;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �ڴ��ַ����
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_MemoryBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream6, &DMA_InitStruct);
    DMA_Cmd(DMA1_Stream6, ENABLE);
}

/************************************************************************************
 *RS485_Init_DMA_Rx
 ** ��ʼ��RS485��DMA���չ��ܡ�
 ** USART2_Rx-DMA1_Stream5_Channel4
 **
 ************************************************************************************/
void RS485_Init_DMA_Rx(void)
{
    DMA_InitTypeDef DMA_InitStruct;

    DMA_DeInit(DMA1_Stream5);         // ��������������������״̬
    DMA_StructInit(&DMA_InitStruct);  // ȱʡֵ����ʼ������
    DMA_InitStruct.DMA_Channel = DMA_Channel_4;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)Usart3_Rx_Data;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;  // ���������赽�ڴ�
    DMA_InitStruct.DMA_BufferSize = Rx_Buf_Size;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;          // ѭ��Ѱַ
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  // �ڴ��ַ����
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Stream5, &DMA_InitStruct);
    DMA_Cmd(DMA1_Stream5, ENABLE);  // ����DMA������
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
}

/************************************************************************************
 *RS232_1_Init_DMA
 ** ��ʼ������1��RS232)��DMA���ܡ�
 ** USART1_Tx-DMA2_Stream7_Channel4, USART1_Rx-DMA2_Stream5_Channel4					*
 ************************************************************************************/
void RS232_1_Init_DMA(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  // ����DMA2ʱ��
    RS232_1_Init_DMA_Tx();
    RS232_1_Init_DMA_Rx();
}

/************************************************************************************
 *RS232_2_Init_DMA
 ** ��ʼ������2��RS232)��DMA���ܡ�
 ** USART3_Tx-DMA1_Stream6_Channel4, USART2_Rx-DMA1_Stream5_Channel4				*
 ************************************************************************************/
void RS232_2_Init_DMA(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  // ����DMA1ʱ��
    RS232_2_Init_DMA_Tx();
    RS232_2_Init_DMA_Rx();
}
/************************************************************************************
 *RS485_Init_DMA
 ** ��ʼ��RS485��DMA���ܡ�
 ** USART2_Tx-DMA1_Stream6_Channel4, USART2_Rx-DMA1_Stream5_Channel4					*
 ************************************************************************************/
void RS485_Init_DMA(void)
{
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  // ����DMA1ʱ��
    RS485_Init_DMA_Tx();
    RS485_Init_DMA_Rx();
}

/************************************************************************************
 *RS232_1_Tx
 ** ����1������������ȫ�����ݡ�
 **
 ************************************************************************************/
uint8_t RS232_1_Tx(uint8_t tx_data[], uint16_t len)
{
    uint16_t i;
    uint8_t ret = 0;
    if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TCIF7) == SET)  // �������
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);  // �����־
        RS232_1_Tx_Enable_Flag =
            1;  // ֪ͨ�ⲿ���������޸�����������������
        ret = 1;
    }
    if (RS232_1_Tx_Enable_Flag) {
        RS232_1_Tx_Enable_Flag = 0;  // �������������־
        DMA2_Stream7->NDTR = len;
        for (i = 0; i < len; i++)  //
        {
            Usart1_Tx_Data[i] = tx_data[i];
        }
        DMA_Cmd(DMA2_Stream7, ENABLE);                  // ����DMA
        USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // ��������
    }

    // �ݴ�����
    if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_TEIF7) == SET)  // ���ʹ���
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TEIF7);  // �����־
        RS232_1_Init_DMA_Tx();                        // ����DMA
        RS232_1_Tx_Enable_Flag = 1;
    }
    if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_DMEIF7) == SET)  // ֱ��ģʽ����
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_DMEIF7);  // �����־
        RS232_1_Init_DMA_Tx();                         // ����DMA
        RS232_1_Tx_Enable_Flag = 1;
    }
    return ret;
}

/************************************************************************************
 *RS232_2_Tx
 ** ����2������������ȫ�����ݡ�
 **
 ************************************************************************************/
uint8_t RS232_2_Tx(uint8_t tx_data[], uint16_t len)
{
    uint16_t i;
    uint8_t ret = 0;
    if (DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TCIF3) == SET)  // �������
    {
        DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);  // �����־
        RS232_2_Tx_Enable_Flag =
            1;  // ֪ͨ�ⲿ���������޸�����������������
        ret = 1;
    }

    if (RS232_2_Tx_Enable_Flag) {
        RS232_2_Tx_Enable_Flag = 0;
        DMA1_Stream3->NDTR = len;
        for (i = 0; i < len; i++)  //
        {
            Usart2_Tx_Data[i] = tx_data[i];
        }
        DMA_Cmd(DMA1_Stream3, ENABLE);                  // ����DMA
        USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);  // ��������
    }
    // �ݴ�����
    if (DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_TEIF3) == SET)  // ���ʹ���
    {
        DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TEIF3);  // �����־
        RS232_2_Init_DMA_Tx();                        // ����DMA
        RS232_2_Tx_Enable_Flag = 1;
    }
    if (DMA_GetFlagStatus(DMA1_Stream3, DMA_FLAG_DMEIF3) == SET)  // ֱ��ģʽ����
    {
        DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_DMEIF3);  // �����־
        RS232_2_Init_DMA_Tx();                         // ����DMA
        RS232_2_Tx_Enable_Flag = 1;
    }
    return ret;
}

/************************************************************************************
 *RS232_1_Rx
 ** ����1(RS232)�������ݡ�
 **
 ************************************************************************************/
uint16_t RS232_1_Rx(uint8_t* RxData)
{
    int i;
    uint16_t RxNum;
    uint16_t len = 0;

    RxNum = Rx_Buf_Size - DMA_GetCurrDataCounter(DMA2_Stream5);  // �����ѽ��յ�����������
    if (RxNum < RS232_1_Rx_Count) {
        RxNum += Rx_Buf_Size;
    }
    if (RxNum - RS232_1_Rx_Count > 0)  // �����Ϊ8�ֽ�
    {
        len = RxNum - RS232_1_Rx_Count;
        for (i = 0; i < len; i++) {
            *RxData++ = Usart1_Rx_Data[RS232_1_Rx_Count++];  // ��ȡ���ݣ��ȴ���������
            if (RS232_1_Rx_Count >= Rx_Buf_Size) {
                RS232_1_Rx_Count -= Rx_Buf_Size;
            }
        }
    }
    // �ݴ�����
    if (DMA_GetFlagStatus(DMA2_Stream5, DMA_FLAG_TEIF5) == SET)  // ���ʹ���
    {
        DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TEIF5);  // �����־
        RS232_1_Init_DMA_Rx();                        // ����DMA
        len = 0;
    }
    if (DMA_GetFlagStatus(DMA2_Stream7, DMA_FLAG_DMEIF7) == SET)  // ֱ��ģʽ����
    {
        DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_DMEIF7);  // �����־
        RS232_1_Init_DMA_Rx();                         // ����DMA
        len = 0;
    }
    return len;
}

/************************************************************************************
 *RS232_2_Rx
 ** ����2(RS232)�������ݡ�
 **
 ************************************************************************************/
uint16_t RS232_2_Rx(uint8_t* RxData)
{
    int i;
    uint16_t RxNum;
    uint16_t len = 0;

    RxNum = Rx_Buf_Size - DMA_GetCurrDataCounter(DMA1_Stream1);  // �����ѽ��յ�����������

    if (RxNum < RS232_2_Rx_Count) {
        RxNum += Rx_Buf_Size;
    }
    if (RxNum - RS232_2_Rx_Count > 0)  // �����ݽ���
    {
        len = RxNum - RS232_2_Rx_Count;
        for (i = 0; i < len; i++) {
            *RxData++ = Usart2_Rx_Data[RS232_2_Rx_Count++];  // ��ȡ���ݣ�����ָ��������ȴ���������
            if (RS232_2_Rx_Count >= Rx_Buf_Size) {
                RS232_2_Rx_Count -= Rx_Buf_Size;
            }
        }
    }
    // 	if (RxNum-RS232_2_Rx_Count>=RS232_2_Rx_Threshold)//���ݳ���ΪThreshold
    // 	{
    // 		Rx_Flag=SET;
    // 		for (i=0;i<RS232_2_Rx_Threshold;i++)
    // 		{
    // 			*RxData++=Usart2_Rx_Data[RS232_2_Rx_Count++];//��ȡ���ݣ�����ָ��������ȴ���������
    // 			if (RS232_2_Rx_Count>=Rx_Buf_Size)
    // 			{
    // 				RS232_2_Rx_Count-=Rx_Buf_Size;
    // 			}
    // 		}
    // 	}
    // �ݴ�����
    if (DMA_GetFlagStatus(DMA1_Stream1, DMA_FLAG_TEIF1) == SET)  // ���ʹ���
    {
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TEIF1);  // �����־
        RS232_2_Init_DMA_Rx();                        // ����DMA
        len = 0;
    }
    if (DMA_GetFlagStatus(DMA1_Stream1, DMA_FLAG_DMEIF1) == SET)  // ֱ��ģʽ����
    {
        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_DMEIF1);  // �����־
        RS232_2_Init_DMA_Rx();                         // ����DMA
        len = 0;
    }
    return len;
}
/************************************************************************************
 *RS485_Tx
 ** ������������ȫ�����ݡ�
 **
 ************************************************************************************/
void RS485_Tx(void)
{
    if (DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TCIF6) == SET)  // �������
    {
        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);  // �����־
        RS485_Tx_Enable_Flag =
            1;  // ֪ͨ�ⲿ���������޸�����������������
    }

    if (RS485_Tx_Flag && RS485_Tx_Enable_Flag) {
        RS485_Tx_Flag = 0;
        RS485_Tx_Enable_Flag = 0;
        DMA_Cmd(DMA1_Stream6, ENABLE);                  // ����DMA
        USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  // ��������
    }
    // �ݴ�����
    if (DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_TEIF6) == SET)  // ���ʹ���
    {
        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TEIF6);  // �����־
        RS485_Init_DMA_Tx();                          // ����DMA
        RS485_Tx_Enable_Flag = 1;
    }
    if (DMA_GetFlagStatus(DMA1_Stream6, DMA_FLAG_DMEIF6) == SET)  // ֱ��ģʽ����
    {
        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_DMEIF6);  // �����־
        RS485_Init_DMA_Tx();                           // ����DMA
        RS485_Tx_Enable_Flag = 1;
    }
}

/************************************************************************************
 *RS485_Rx
 ** �������ݡ�
 **
 ************************************************************************************/
FlagStatus RS485_Rx(uint8_t* RxData)
{
    int i;
    uint32_t RxNum;
    FlagStatus Rx_Flag = RESET;

    RxNum = Rx_Buf_Size - DMA_GetCurrDataCounter(DMA1_Stream5);  // �����ѽ��յ�����������
    if (RxNum < RS485_Rx_Count) {
        RxNum += Rx_Buf_Size;
    }
    if (RxNum - RS485_Rx_Count >= RS485_Rx_Threshold)  // ���ݳ���ΪThreshold
    {
        Rx_Flag = SET;
        for (i = 0; i < RS485_Rx_Threshold; i++) {
            *RxData++ = Usart3_Rx_Data[RS485_Rx_Count++];  // ��ȡ���ݣ�����ָ��������ȴ���������
            if (RS485_Rx_Count >= Rx_Buf_Size) {
                RS485_Rx_Count -= Rx_Buf_Size;
            }
        }
    }
    // �ݴ�����
    if (DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TEIF5) == SET)  // ���ʹ���
    {
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TEIF5);  // �����־
        RS485_Init_DMA_Rx();                          // ����DMA
    }
    if (DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_DMEIF5) == SET)  // ֱ��ģʽ����
    {
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_DMEIF5);  // �����־
        RS485_Init_DMA_Rx();                           // ����DMA
    }
    return Rx_Flag;
}

void Init_RS232_2_NVIC_Cofig(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

/************************************************************************************
 *Init_RS232_1
 ** ��ʼ������1��
 **
 ************************************************************************************/
void Init_RS232_1(uint32_t baudrate, uint16_t parity, uint16_t wordlength, uint16_t stopbits)
{
    RS232_1_Init_Para();  // ��������ʼ��
    RS232_1_Init_Pin();   // ���ż�ʱ�ӳ�ʼ��
    RS232_1_Init_Port(baudrate, parity, wordlength, stopbits);
    RS232_1_Init_DMA();
}
/************************************************************************************
 *Init_RS232_2
 ** ��ʼ������2,DMA��ʽ
 **
 ************************************************************************************/
void Init_RS232_2(uint32_t baudrate, uint16_t parity, uint16_t wordlength, uint16_t stopbits)
{
    RS232_2_Init_Para();  // ��������ʼ��
    RS232_2_Init_Pin();   // ���ż�ʱ�ӳ�ʼ��
    RS232_2_Init_Port(baudrate, parity, wordlength, stopbits);
    RS232_2_Init_DMA();
}
/************************************************************************************
 *Init_RS232_2
 ** ��ʼ������2���жϽ��շ�ʽ
 **
 ************************************************************************************/
void Init_RS232_2_NVIC(uint32_t baudrate, uint16_t parity, uint16_t wordlength, uint16_t stopbits)
{
    RS232_2_Init_Para();  // ��������ʼ��
    RS232_2_Init_Pin();   // ���ż�ʱ�ӳ�ʼ��
    RS232_2_Init_Port(baudrate, parity, wordlength, stopbits);
    Init_RS232_2_NVIC_Cofig();
}
/************************************************************************************
 *Init_RS485
 ** ��ʼ������2��
 **
 ************************************************************************************/
void Init_RS485(uint32_t BaudRate, uint32_t Tx_Num, uint32_t Rx_Threshold)
{
    RS485_Init_Para(Tx_Num, Rx_Threshold);  // ��������ʼ��
    RS485_Init_Pin();                       // ���ż�ʱ�ӳ�ʼ��
    RS485_Init_Port(BaudRate);              // 9600, n, 8, 1
    RS485_Init_DMA();
}

void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        //
        /* Read one byte from the receive data register */
        // unsigned char ch = USART_ReceiveData(USART6);
        // ch = USART_ReceiveData(USART6);
        // Uart_CharRxReady(ch);

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

//===========================================================================
// End of file.
//===========================================================================
