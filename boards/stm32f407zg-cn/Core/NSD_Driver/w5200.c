/**************************************************************************************************
 * @file    w5200.c                                                                                *
 * @author  ���´����                                                                             *
 * @version V1.0.0                                                                                 *
 * @date    2016.8.10.                                                                             *
 * @brief   w5200��������                                                                          *
 **************************************************************************************************/
#include <stm32f4xx.h>

#include "w5200Reg.h"

extern unsigned char uchNodeNum;

void Network_Init(SPI_TypeDef *SPIx,
    unsigned char *Self_IP,
    unsigned char *Socket_Enable,
    unsigned short int *Self_Port,
    unsigned char *Socket_mode,
    unsigned char *Dest_IP,
    unsigned short int *Dest_Port,
    int DMA_Size);

/********************************************************************
 *Soft_Delay                                                         *
 *������ʱ��                                                         *
 ********************************************************************/
void Soft_Delay(unsigned int x)
{
    unsigned int i;
    for (i = 0; i < x; i++);
}

/********************************************************************
 *Net_Para_Init                                                      *
 *��ʼ��Net����������ز�����                                        *
 ********************************************************************/
void Net_Para_Init(int Index)
{
    unsigned char i;

    Net_State[Index] = 0;  // ״̬����
    Net_Rx_size[Index] = 0;

    for (i = 0; i < 2; i++) {
        Net_Rx_Addr_Offset[Index][i] = 0;
        Net_Tx_Addr_Offset[Index][i] = 0;
        Net_Socket_Rx_Addr[Index][i] = Net_Rx_Addr[i];
        Net_Socket_Tx_Addr[Index][i] = Net_Tx_Addr[i];
        Net_Socket_State[Index][i] = 0;  // ����1(Socket0...Socket7)״̬��¼��ʼ��
        Net_Socket_Data[Index][i] = 0;  // ����1(Socket0...Socket7)���պͷ������ݵ�״̬��ʼ��
        Net_Tx_Flag[Index][i] = 0;
        Net_Rx_Flag[Index][i] = 0;
    }
    Net_Receiving_Socket[Index] = 8;  // Socket not exist
    Net_Sending_Socket[Index] = 8;
    Net_Rx_Delay[Index] = 0;
    Net_Tx_Delay[Index] = 0;
}

/********************************************************************
 *SPI2_Init                                                          *
 *SPI2Ӳ����ʼ��                                                     *
 *SCK:PB13,MOSI:PB15,MISO:PB14,NSS:PB12, Reset:PA12, CS:PA11         *
 ********************************************************************/
void SPI2_Init(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable the SPI clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    // Connect SPI pins to AF5
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

    // SPI SCK pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // SPI  MISO pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // SPI  MOSI pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // NSS
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // CS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BSRRH = GPIO_Pin_11;  // CS low

    // Reset
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIOA->BSRRH = GPIO_Pin_12;  // Reset off

    // SPI configuration
    SPI_I2S_DeInit(SPI2);  // �����Ƚ���,���ܸı�MODE
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  // ����ȫ˫��
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  // 2��Ƶ
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    // Initializes the SPI communication
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI2, &SPI_InitStructure);

    // The Data transfer is performed in the SPI interrupt routine
    SPI_Cmd(SPI2, DISABLE);
    SPI_SSOutputCmd(SPI2, ENABLE);
}

/********************************************************************
 *SPI3_Init                                                          *
 *SPI3Ӳ����ʼ��                                                     *
 *SCK:PC10,MOSI:PC12,MISO:PC11,NSS:PA15,Reset:PB4, CS:PB3            *
 ********************************************************************/
void SPI3_Init(void)
{
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable the SPI clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // Connect SPI pins to AF5
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

    // SPI SCK pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // SPI  MISO pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // SPI  MOSI pin configuration
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // SPI  NSS pin configuration
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // CS
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIOB->BSRRH = GPIO_Pin_3;  // CS low

    // Reset
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIOB->BSRRH = GPIO_Pin_4;  // Reset off

    // SPI configuration
    SPI_I2S_DeInit(SPI3);  // �����Ƚ���,���ܸı�MODE
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  // ����ȫ˫��
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  // 2��Ƶ
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    // Initializes the SPI communication
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_Init(SPI3, &SPI_InitStructure);

    SPI_Cmd(SPI3, DISABLE);
    SPI_SSOutputCmd(SPI3, ENABLE);
}

/********************************************************************
 *SPI2_DMA_Config                                                    *
 *SPI2�����շ���DMA��ʼ��                                            *
 ********************************************************************/
void SPI2_DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructureTx, DMA_InitStructureRx;

    // ����DMAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    // ����ͨ��
    DMA_DeInit(DMA1_Stream4);
    DMA_InitStructureTx.DMA_Channel = DMA_Channel_0;                       // ѡ��ͨ��0
    DMA_InitStructureTx.DMA_PeripheralBaseAddr = (unsigned int)&SPI2->DR;  // SPI1��DR�Ĵ�����ַ��SPI data register
    DMA_InitStructureTx.DMA_Memory0BaseAddr =
        (unsigned int)SPI_Tx_Buf[0];                           //(unsigned int)SPI_Tx_Buf[0]���Զ����һ������;
    DMA_InitStructureTx.DMA_DIR = DMA_DIR_MemoryToPeripheral;  // ����DMA�Ĵ��䷽���ڴ浽��Χ�豸
    DMA_InitStructureTx.DMA_BufferSize = DMA_Tx_Size[0] + 4;  // ������DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructureTx.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  // ��DMA���������ģʽ�����DMAѡ�õ�ͨ����CHx���ж���������ӣ���Ҫʹ���������ģʽ��
    DMA_InitStructureTx.DMA_MemoryInc =
        DMA_MemoryInc_Enable;  // DMA�ڴ��ַ�Զ�����ģʽ����DMA���ʶ���ڴ����ʱ����Ҫʹ��DMA_MemoryInc_Enable
    DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // ����DMA�ڷ���ʱÿ�β��������ݳ���
    DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;       // ����DMA�Ĵ���ģʽ��ͨ��ģʽ
    DMA_InitStructureTx.DMA_Priority = DMA_Priority_Low;  // ����DMA�����ȼ��𣺹�4������
    DMA_InitStructureTx.DMA_FIFOMode = DMA_FIFOMode_Disable;  //
    DMA_InitStructureTx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructureTx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructureTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream4, &DMA_InitStructureTx);

    // ʹ��SPI2��DMA�շ�����
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Stream4, DISABLE);

    // ����ͨ��
    DMA_DeInit(DMA1_Stream3);
    DMA_StructInit(&DMA_InitStructureRx);  // ȱʡֵ����ʼ������
    DMA_InitStructureRx.DMA_Channel = DMA_Channel_0;
    DMA_InitStructureRx.DMA_PeripheralBaseAddr = (unsigned int)&SPI2->DR;  // SPI2��DR�Ĵ�����ַ
    DMA_InitStructureRx.DMA_Memory0BaseAddr = (unsigned int)SPI_Rx_Buf[0];
    DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructureRx.DMA_BufferSize = DMA_Tx_Size[0] + 4;
    DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;  // DMA�ڴ��ַ�Զ�����ģʽ
    DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureRx.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructureRx.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructureRx.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructureRx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructureRx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructureRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &DMA_InitStructureRx);

    DMA_Cmd(DMA1_Stream3, ENABLE);
}

/********************************************************************
 *SPI3_DMA_Config                                                    *
 *SPI3�����շ���DMA��ʼ��                                            *
 ********************************************************************/
void SPI3_DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructureTx, DMA_InitStructureRx;

    // ����DMAʱ��
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

    // ����ͨ��
    DMA_DeInit(DMA1_Stream7);
    DMA_InitStructureTx.DMA_Channel = DMA_Channel_0;                       // ѡ��ͨ��0
    DMA_InitStructureTx.DMA_PeripheralBaseAddr = (unsigned int)&SPI3->DR;  // SPI1��DR�Ĵ�����ַ��SPI data register
    DMA_InitStructureTx.DMA_Memory0BaseAddr = (unsigned int)SPI_Tx_Buf[1];
    DMA_InitStructureTx.DMA_DIR = DMA_DIR_MemoryToPeripheral;  // ����DMA�Ĵ��䷽���ڴ浽��Χ�豸
    DMA_InitStructureTx.DMA_BufferSize = DMA_Tx_Size[1] + 4;  // ������DMA�ڴ���ʱ�������ĳ���
    DMA_InitStructureTx.DMA_PeripheralInc =
        DMA_PeripheralInc_Disable;  // ��DMA���������ģʽ�����DMAѡ�õ�ͨ����CHx���ж���������ӣ���Ҫʹ���������ģʽ��
    DMA_InitStructureTx.DMA_MemoryInc =
        DMA_MemoryInc_Enable;  // DMA�ڴ��ַ�Զ�����ģʽ����DMA���ʶ���ڴ����ʱ����Ҫʹ��DMA_MemoryInc_Enable
    DMA_InitStructureTx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // ����DMA�ڷ���ʱÿ�β��������ݳ���
    DMA_InitStructureTx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureTx.DMA_Mode = DMA_Mode_Normal;       // ����DMA�Ĵ���ģʽ��ͨ��ģʽ
    DMA_InitStructureTx.DMA_Priority = DMA_Priority_Low;  // ����DMA�����ȼ��𣺹�4������
    DMA_InitStructureTx.DMA_FIFOMode = DMA_FIFOMode_Disable;  //
    DMA_InitStructureTx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructureTx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructureTx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream7, &DMA_InitStructureTx);

    // ʹ��SPI3��DMA���͹���
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Stream7, DISABLE);

    // ����ͨ��
    DMA_DeInit(DMA1_Stream2);
    DMA_StructInit(&DMA_InitStructureRx);  // ȱʡֵ����ʼ������
    DMA_InitStructureRx.DMA_Channel = DMA_Channel_0;
    DMA_InitStructureRx.DMA_PeripheralBaseAddr = (unsigned int)&SPI3->DR;  // SPI2��DR�Ĵ�����ַ
    DMA_InitStructureRx.DMA_Memory0BaseAddr = (unsigned int)SPI_Rx_Buf[1];
    DMA_InitStructureRx.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructureRx.DMA_BufferSize = DMA_Tx_Size[1] + 4;
    DMA_InitStructureRx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructureRx.DMA_MemoryInc = DMA_MemoryInc_Enable;  // DMA�ڴ��ַ�Զ�����ģʽ
    DMA_InitStructureRx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructureRx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructureRx.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructureRx.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructureRx.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructureRx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructureRx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructureRx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream2, &DMA_InitStructureRx);

    DMA_Cmd(DMA1_Stream2, ENABLE);
}

/********************************************************************
 *RST_On                                                             *
 *���ڸ�λ�����ø�                                                   *
 ********************************************************************/
void RST_On(int Index)
{
    if (Index) {
        GPIOB->BSRRL = GPIO_Pin_4;
    }
    else {
        GPIOA->BSRRL = GPIO_Pin_12;
    }
}

/********************************************************************
 *RST_Off                                                            *
 *���ڸ�λ�����õ�                                                   *
 ********************************************************************/
void RST_Off(int Index)
{
    if (Index) {
        GPIOB->BSRRH = GPIO_Pin_4;
    }
    else {
        GPIOA->BSRRH = GPIO_Pin_12;
    }
}

/********************************************************************
 *Net_Write_Byte                                                     *
 *��ָ����ַд���ֽڵ�W5200��                                        *
 ********************************************************************/
void Net_Write_Byte(SPI_TypeDef *SPIx, unsigned short int addr, unsigned char dat)
{
    int i = 0;

    SPI_Cmd(SPIx, ENABLE);

    // Send W5200 Register address
    SPI_I2S_SendData(SPIx, (unsigned char)(addr / 256));
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && i++ < SPI_Delay);
    SPI_I2S_SendData(SPIx, (unsigned char)addr);
    i = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++ < SPI_Delay));

    // Send WRITE Command
    SPI_I2S_SendData(SPIx, 0x80);
    i = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++ < SPI_Delay));
    SPI_I2S_SendData(SPIx, 0x01);
    i = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++ < SPI_Delay));

    // Write data to W5200 register
    SPI_I2S_SendData(SPIx, dat);
    i = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++ < SPI_Delay));

    SPI_Cmd(SPIx, DISABLE);
    Soft_Delay(2);
}

/********************************************************************
 *Net_Write                                                          *
 *��ָ����ַд��ָ�����ȵ��ַ�����W5200                              *
 ********************************************************************/
void Net_Write(SPI_TypeDef *SPIx, unsigned short int Reg_Addr, unsigned char *Data_ptr, unsigned short int Data_Size)
{
    unsigned short i, j = 0;
    // Set W5200 CS Low
    SPI_Cmd(SPIx, ENABLE);

    SPI_I2S_SendData(SPIx, Reg_Addr / 256);
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    SPI_I2S_SendData(SPIx, Reg_Addr);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));

    // Send WRITE Command, send write data size
    i = Data_Size | 0x8000;
    SPI_I2S_SendData(SPIx, i / 256);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    SPI_I2S_SendData(SPIx, i);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));

    // Write data to W5200 register, size=Data_Size
    for (i = 0; i < Data_Size; i++) {
        SPI_I2S_SendData(SPIx, *Data_ptr);
        j = 0;
        while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
        Data_ptr++;
    }

    // Set W5200 CS HIGH
    SPI_Cmd(SPIx, DISABLE);
    Soft_Delay(2);
}

/********************************************************************
 *Net_Read_Byte                                                      *
 *��W5200��ָ����ַ��ȡ�����ַ���                                    *
 ********************************************************************/
unsigned char Net_Read_Byte(SPI_TypeDef *SPIx, unsigned short int Reg_Addr)
{
    unsigned char i;
    int j = 0;
    SPI_Cmd(SPIx, ENABLE);
    // Send W5200 Register address
    SPI_I2S_SendData(SPIx, (unsigned char)(Reg_Addr / 256));
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    SPI_I2S_SendData(SPIx, (unsigned char)Reg_Addr);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    // Send READ Command, send read data size
    SPI_I2S_SendData(SPIx, 0x00);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    SPI_I2S_SendData(SPIx, 0x01);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++ < SPI_Delay));

    i = SPI_I2S_ReceiveData(SPIx);
    // Read 1 byte from W5200 register
    SPI_I2S_SendData(SPIx, 0x00);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++ < SPI_Delay));
    i = SPI_I2S_ReceiveData(SPIx);

    SPI_Cmd(SPIx, DISABLE);
    Soft_Delay(2);
    return i;
}

/********************************************************************
 *Net_Read                                                           *
 *��W5200��ָ����ַ��ȡָ�����ȵ��ַ���                              *
 ********************************************************************/
void Net_Read(SPI_TypeDef *SPIx, unsigned short int Reg_Addr, unsigned char *Data_ptr, unsigned short int Data_Size)
{
    unsigned short int i = 0, j = 0;

    SPI_Cmd(SPIx, ENABLE);

    // Send W5200 Register address
    SPI_I2S_SendData(SPIx, Reg_Addr / 256);
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++ < SPI_Delay));
    SPI_I2S_SendData(SPIx, Reg_Addr);
    i = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (i++ < SPI_Delay));

    // Send READ Command, send read data size
    i = Data_Size & 0x7fff;
    SPI_I2S_SendData(SPIx, i / 256);
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    SPI_I2S_SendData(SPIx, i);
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) && (j++ < SPI_Delay));
    j = 0;
    while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++ < SPI_Delay));

    i = SPI_I2S_ReceiveData(SPIx);
    // Read data from W5200 register, size=Data_Size
    for (i = 0; i < Data_Size; i++) {
        SPI_I2S_SendData(SPIx, 0x00);
        j = 0;
        while ((SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET) && (j++ < SPI_Delay));
        *Data_ptr = SPI_I2S_ReceiveData(SPIx);
        Data_ptr++;
    }
    // Set W5200 CS as HIGH
    SPI_Cmd(SPIx, DISABLE);
    Soft_Delay(2);
}

/********************************************************************
 *��������Net_Config                                                 *
 *���ܣ��������ڵ�W5200�����÷��ͻ������ͽ��ջ������Ĵ�С����������  *
 *ʱ�䣬�������Դ�����	                                              *
 ********************************************************************/
void Net_Config(SPI_TypeDef *SPIx)
{
    unsigned char i;

    for (i = 0; i < 2; i++) {
        Net_Write_Byte(SPIx, Sn_RXMEM_SIZE(i), 0x08);  // Socket Rx memory size=8k
        Net_Write_Byte(SPIx, Sn_TXMEM_SIZE(i), 0x08);  // Socket Tx memory size=8k
    }
    Net_Write_Byte(SPIx, RTR_W, 0xf);  // 400ms
    Net_Write_Byte(SPIx, RTR_W + 1, 0xa0);
}

/********************************************************************
 *������: Socket_Mode                                                *
 *���ܣ�	��Ϊ������ʱ������, ��ʼ����.                             *
 *       ��Ϊ�ͻ���ʱ������, ��ʼ����.                             *
 *����: s : SOCKET��                                                 *
 *����: �ɹ��򷵻�TRUE, ���򷵻�FALSE                                *
 ********************************************************************/
unsigned char Socket_Mode(SPI_TypeDef *SPIx, unsigned char s, unsigned char Mode)
{
    unsigned char r = TRUE;

    Net_Write_Byte(SPIx, Sn_MR(s), Sn_MR_TCP);   // Set Socket in TCP Mode
    Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_OPEN);  // Open Socket

    if (Net_Read_Byte(SPIx, Sn_SR(s)) != SOCK_INIT) {
        Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_CLOSE);  // Failed to open Socket, Close it and retuen false
        r = FALSE;
    }
    else {
        if (Mode == Server) {
            Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_LISTEN);  // Set Socket in TCP Server Mode

            if (Net_Read_Byte(SPIx, Sn_SR(s)) != SOCK_LISTEN) {
                Net_Write_Byte(
                    SPIx, Sn_CR(s), Sn_CR_CLOSE);  // Failed to set in TCP Server Mode, Close Socket and return false
                r = FALSE;
            }
        }
        else {
            Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_CONNECT);
        }
    }

    return r;
}

/********************************************************************
 *Socket_Correct_Addr                                                *
 *������ʹ��DMA��ʽ�������ݷ���ʱ�����W5200�ķ��ͻ�������ַ����У�� *
 *�������øú���                                                     *
 ********************************************************************/
void Socket_Correct_Addr(SPI_TypeDef *SPIx, unsigned char s)
{
    unsigned short int i;
    unsigned short int tx_offset, tx_offset1;
    int Index;

    Index = (SPIx == SPI3);

    // ��ȡ���ͻ�������ƫ����
    tx_offset = Net_Read_Byte(SPIx, Sn_TX_WR0(s));
    tx_offset *= 256;
    tx_offset += Net_Read_Byte(SPIx, Sn_TX_WR0(s) + 1);
    tx_offset1 = tx_offset;

    i = tx_offset / S_BUF_SIZE;  // ����ʵ�ʵ�����ƫ������S0_TX_SIZE��Ҫ��ǰ��#define�ж���*/
                                 // ע��S0_TX_SIZE��ֵ��W5200_Init()������W5200_TMSR��ȷ��*/
    tx_offset = tx_offset - i * S_BUF_SIZE;
    i = tx_offset % DMA_Tx_Size[Index];
    i = DMA_Tx_Size[Index] - i;  // Ҫ���������ݳ���

    Net_Write(SPIx, (TX_MEM + s * S_BUF_SIZE + tx_offset), SPI_Tx_Buf[0], i);

    // ������һ�ε�ƫ����
    tx_offset1 += i;
    Net_Tx_Addr_Offset[Index][s] = tx_offset1;
    Net_Write_Byte(SPIx, Sn_TX_WR0(s), (tx_offset1 / 256));
    Net_Write_Byte(SPIx, (Sn_TX_WR0(s) + 1), tx_offset1);
    Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_SEND);  // Send SEND command, start send process

    Net_Socket_Tx_Addr[Index][s] += (tx_offset + i);
    if (Net_Socket_Tx_Addr[Index][s] >= (0x8000 + s * S_BUF_SIZE + S_BUF_SIZE)) {
        Net_Socket_Tx_Addr[Index][s] = 0x8000 + s * S_BUF_SIZE;
    }
}

/********************************************************************************
 *������: Network_Init                                                           *
 *���ܣ�	���ڳ�ʼ������                                                          *
 *W5200����Socket��ʼ������������IP�����ء��Ƿ����á��˿ڡ�C/S��Ŀ��IP��Ŀ��˿�.*
 ********************************************************************************/
void Network_Init(SPI_TypeDef *SPIx,
    unsigned char *Self_IP,
    unsigned char *Socket_Enable,
    unsigned short int *Self_Port,
    unsigned char *Socket_mode,
    unsigned char *Dest_IP,
    unsigned short int *Dest_Port,
    int DMA_Size)
{
    unsigned short int i, j;
    unsigned char ucC[2];
    unsigned char *ucPt;
    unsigned char Gate_Way[4];
    int Index;

    Index = (SPIx == SPI3);
    if (Index) {
        SPI3_Init();  // ����SPI��ʼ�������ų�ʼ������λ���ų�ʼ������λ��������͵�ƽ
    }
    else {
        SPI2_Init();
    }

    Soft_Delay(50000);
    RST_On(Index);
    for (i = 0; i < 400; i++)  // ��ʱ����150ms
    {
        Soft_Delay(60000);
    }

    DMA_Tx_Size[Index] = 0;
    if (DMA_Size == DMA_SIZE_128 || DMA_Size == DMA_SIZE_256 || DMA_Size == DMA_SIZE_512 || DMA_Size == DMA_SIZE_1K ||
        DMA_Size == DMA_SIZE_2K || DMA_Size == DMA_SIZE_4K || DMA_Size == DMA_SIZE_8K) {
        DMA_Tx_Size[Index] = DMA_Size;
        if (Index) {
            SPI3_DMA_Config();  // DMA��ʼ��
        }
        else {
            SPI2_DMA_Config();
        }
    }
    Net_Config(SPIx);  // ���ø������ӵĻ����С

    if (uchNodeNum)  // B���ͻ���
    {
        // ����MAC��ַ
        Network_Phy_Addr[0] = 0x34;
        Network_Phy_Addr[1] = 0x24;
        Network_Phy_Addr[2] = 0xE8;
        Network_Phy_Addr[3] = 0xB9;
        Network_Phy_Addr[4] = 0x8B + uchNodeNum;
        Network_Phy_Addr[5] = 0x31 + Index;
    }
    else  // A���ͻ���
    {
        // ����MAC��ַ
        Network_Phy_Addr[0] = 0x34;
        Network_Phy_Addr[1] = 0x24;
        Network_Phy_Addr[2] = 0xE8;
        Network_Phy_Addr[3] = 0xB9 + uchNodeNum;
        Network_Phy_Addr[4] = 0x8B;
        Network_Phy_Addr[5] = 0x33 + Index;
    }
    ucPt = Self_IP;
    Gate_Way[0] = *ucPt++;
    Gate_Way[1] = *ucPt++;
    Gate_Way[2] = *ucPt++;
    Gate_Way[3] = 1;
    Net_Write(SPIx, GAR0, Gate_Way, 4);           // ��������IP��ַ
    Net_Write(SPIx, SUBR0, Network_Sub_Mask, 4);  // ������������
    Net_Write(SPIx, SHAR0, Network_Phy_Addr, 6);  // ����MAC��ַ
    Net_Write(SPIx, SIPR0, Self_IP, 4);           // ���ñ���IP��ַ
    i = 4000;
    ucC[0] = i >> 8;
    ucC[1] = i & 0xff;
    Net_Write(SPIx, RTR_W, ucC, 2);
    ucC[0] = 1;
    Net_Write(SPIx, RCR, ucC, 1);

    //--Socket0...Socket7	���ض˿ںš�Ŀ�ĵض˿ںź�Ŀ�ĵ�IP��ַ������
    for (i = 0; i < 2; i++) {
        Net_Enable[Index][i] = Socket_Enable[i];
        Net_Mode[Index][i] = Socket_mode[i];  // ģʽ: ������/�ͻ���

        // �����˿ں�
        j = Self_Port[i];
        ucC[0] = j >> 8;
        ucC[1] = j & 0xff;
        Net_Write(SPIx, Sn_PORT0(i), ucC, 2);  // ���ñ��ض˿ں�

        // Ŀ��IP
        Net_Write(SPIx, Sn_DIPR0(i), Dest_IP + i * 4, 4);
        // Dest_IP+=4;

        // Ŀ��˿ں�
        j = Dest_Port[i];
        ucC[0] = j >> 8;
        ucC[1] = j & 0xff;
        Net_Write(SPIx, Sn_DPORT0(i), ucC, 2);
    }

    Net_Para_Init(Index);  // ��������ʼ��

    // ��¼����, ������������
    for (i = 0; i < 4; i++) {
        Para_Self_IP[Index][i] = Self_IP[i];
        Para_Gateway[Index][i] = Gate_Way[i];
        Para_Des_IP[Index][i] = Dest_IP[i];
        Para_Des_IP[Index][i + 4] = Dest_IP[i + 4];
    }
    for (i = 0; i < 2; i++) {
        Para_Socket_Enable[Index][i] = Socket_Enable[i];
        Para_Self_Port[Index][i] = Self_Port[i];
        Para_Socket_Mode[Index][i] = Socket_mode[i];
        Para_Des_Port[Index][i] = Dest_Port[i];
    }
    Para_DMA_Size[Index] = DMA_Size;
    for (i = 0; i < 8; i++) {
        Net_Port[Index][i] = Self_Port[i];
    }
}

/********************************************************************
 *Net_Interrupt_Process                                              *
 *���ڵ�״̬��ȡ����������                                           *
 ********************************************************************/
void Net_Interrupt_Process(SPI_TypeDef *SPIx)
{
    unsigned char i, j, k = 1, n;
    int Index;
    unsigned char ucC[2];

    Index = (SPIx == SPI3);
    i = Net_Read_Byte(SPIx, IR2);  // ��ȡSocket�ж�
    for (n = 0; n < 2; n++) {
        if (i & k) {
            j = Net_Read_Byte(SPIx, Sn_IR(n));
            Net_Write_Byte(SPIx, Sn_IR(n), j);  // ��д���жϱ�־

            if (j & S_IR_CON)  // ��TCPģʽ��,Socket�ɹ�����
            {
                if (Net_Mode[Index][n] == Server) {
                    Net_Socket_State[Index][n] |= S_CONN;
                }
                else if (Net_Mode[Index][n] == Client) {
                    Net_Socket_State[Index][n] |= (S_CONN | S_OK);
                }
            }
            if (j & S_IR_DISCON)  // ��TCPģʽ��Socket�Ͽ����Ӵ���
            {
                Net_Write_Byte(SPIx, Sn_CR(n), Sn_CR_DISCON);  // Disconnect socket
                Net_Write_Byte(SPIx, Sn_CR(n), Sn_CR_CLOSE);   // Close socket
                Net_Socket_Rx_Addr[Index][n] = Net_Rx_Addr[n];
                Net_Socket_Tx_Addr[Index][n] = Net_Tx_Addr[n];
                Net_Rx_Addr_Offset[Index][n] = 0;

                Net_Socket_State[Index][n] = 0;
            }
            if (j & S_IR_SENDOK)  // Socket���ݷ�����ɣ������ٴ�����S_tx_process()������������
            {
                Net_Socket_Data[Index][n] |= S_TRANSMITOK;
            }
            if (j & S_IR_RECV)  // Socket���յ����ݣ���������S_rx_process()����
            {
                Net_Socket_Data[Index][n] |= S_RECEIVE;
            }
            if (j & S_IR_TIMEOUT)  // Socket���ӻ����ݴ��䳬ʱ����
            {
                if (Net_Socket_State[Index][n] == 7) {
                    Net_Write_Byte(SPIx, Sn_CR(n), Sn_CR_CLOSE);  // Close socket
                    Net_Socket_State[Index][n] = 0;
                    Net_Socket_Rx_Addr[Index][n] = Net_Rx_Addr[n];
                    Net_Socket_Tx_Addr[Index][n] = Net_Tx_Addr[n];
                    Net_Rx_Addr_Offset[Index][n] = 0;
                    ucC[0] = Net_Port[Index][n] >> 8;
                    ucC[1] = Net_Port[Index][n] & 0xff;
                    Net_Write(SPIx, Sn_PORT0(i), ucC, 2);
                }
            }
        }
        k <<= 1;
    }
}

/********************************************************************
 *Net_DMA_rx_process                                                 *
 *���ڲ���DMA��ʽ�������ݽ��յĹ��̣����ݽ��ճ���Ϊ128               *
 ********************************************************************/
unsigned char Net_DMA_rx_process(SPI_TypeDef *SPIx, unsigned char s)
{
    unsigned char i = Failure;
    int j;
    int Index;

    Index = (SPIx == SPI3);
    if (Net_State[Index] == S_IDLING) {
        Net_State[Index] = S_RECEIVING;  // �������״̬

        SPI_Tx_Buf[Index][0] = (unsigned char)(Net_Socket_Rx_Addr[Index][s] >> 8);  // ���ջ�������ַ��ÿ���˿ڵĵ�ַ�ǲ�һ����
        SPI_Tx_Buf[Index][1] = (unsigned char)Net_Socket_Rx_Addr[Index][s];
        SPI_Tx_Buf[Index][2] = DMA_Tx_Size[Index] >> 8;  // ģʽ�����գ����ȣ�8192
        SPI_Tx_Buf[Index][3] = DMA_Tx_Size[Index] & 0xff;
        // ģʽ�����գ����ȣ�8192
        // Received Data Size
        Net_Rx_size[Index] = Net_Read_Byte(SPIx, Sn_RX_RSR0(s));
        Net_Rx_size[Index] *= 256;
        Net_Rx_size[Index] += Net_Read_Byte(SPIx, Sn_RX_RSR0(s) + 1);

        Net_Socket_Rx_Addr[Index][s] += Net_Rx_size[Index];

        if (Net_Socket_Rx_Addr[Index][s] >= Net_Rx_Addr[s] + S_BUF_SIZE) {
            Net_Socket_Rx_Addr[Index][s] = Net_Rx_Addr[s];
        }

        SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, ENABLE);  // ����DMA��
        // Set W5200 CS as Low
        SPI_Cmd(SPIx, ENABLE);
        if (Index)  // ����DMA����
        {
            DMA_Cmd(DMA1_Stream7, ENABLE);
        }
        else {
            DMA_Cmd(DMA1_Stream4, ENABLE);
        }

        i = Failure;
    }
    else if (Net_State[Index] == S_RECEIVING) {
        if (Index) {
            j = DMA_GetFlagStatus(DMA1_Stream7, DMA_FLAG_TCIF7);
        }
        else {
            j = DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4);
        }
        if (j == SET) {
            if (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == 0) {
                if (Index) {
                    DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);  // �����־
                }
                else {
                    DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
                }

                SPI_Cmd(SPIx, DISABLE);
                SPI_I2S_DMACmd(SPIx, SPI_I2S_DMAReq_Rx, DISABLE);  // �ر�DMA��
                for (j = 0; j < DMA_Tx_Size[Index]; j++) {
                    Net_Rx_Buf[Index][j] = SPI_Rx_Buf[Index][j + 4];
                }
                Net_Rx_Addr_Offset[Index][s] += Net_Rx_size[Index];

                Net_Write_Byte(SPIx, Sn_RX_RD0(s), (Net_Rx_Addr_Offset[Index][s] / 256));
                Net_Write_Byte(SPIx, (Sn_RX_RD0(s) + 1), Net_Rx_Addr_Offset[Index][s]);

                Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_RECV);  // Send RECV command

                Net_State[Index] = S_IDLING;
                Net_Socket_Data[Index][s] &= ~S_RECEIVE;
                i = Success;
            }
        }
    }
    return i;
}

/********************************************************************************
 *Net_Rx                                                                         *
 *���ڽ��պ���                                                                   *
 ********************************************************************************/
void Net_Rx(SPI_TypeDef *SPIx)
{
    unsigned short int i, j, k;
    unsigned short int rx_offset;
    int Index;

    Index = (SPIx == SPI3);

    for (i = 0; i < 2; i++) {
        if (Net_Enable[Index][i] &&
            (Net_Socket_State[Index][i] ==
                (S_INIT | S_CONN | S_OK)) &&  // �����˴������׽��������ӳɹ�
            (Net_Socket_Data[Index][i] & S_RECEIVE))  // �յ�����
        {
            // Received Data Size
            Net_Rx_size[Index] = Net_Read_Byte(SPIx, Sn_RX_RSR0(i));
            Net_Rx_size[Index] *= 256;
            Net_Rx_size[Index] += Net_Read_Byte(SPIx, Sn_RX_RSR0(i) + 1);

            // rx offset
            rx_offset = Net_Read_Byte(SPIx, Sn_RX_RD0(i));
            rx_offset *= 256;
            rx_offset += Net_Read_Byte(SPIx, Sn_RX_RD0(i) + 1);

            j = rx_offset / S_BUF_SIZE;
            k = rx_offset - j * S_BUF_SIZE;  // ���ƫ����
            j = k + Net_Rx_size[Index];      /// ����β��

            if (j > S_BUF_SIZE)  // ����β��
            {
                j = S_BUF_SIZE - k;
                Net_Read(SPIx, RX_MEM + i * S_BUF_SIZE + k, Net_Rx_Buf[Index], j);
                Net_Read(SPIx, RX_MEM + i * S_BUF_SIZE, Net_Rx_Buf[Index] + j, Net_Rx_size[Index] - j);
            }
            else {
                Net_Read(SPIx, RX_MEM + i * S_BUF_SIZE + k, Net_Rx_Buf[Index], Net_Rx_size[Index]);
            }

            // �µ�ƫ�Ƶ�ַ
            rx_offset += Net_Rx_size[Index];
            Net_Write_Byte(SPIx, Sn_RX_RD0(i), (rx_offset / 256));
            Net_Write_Byte(SPIx, Sn_RX_RD0(i) + 1, rx_offset);
            Net_Write_Byte(SPIx, Sn_CR(i), Sn_CR_RECV);  // ��������
            Net_Socket_Data[Index][i] &= ~S_RECEIVE;
            Net_Rx_Flag[Index][i] = 1;
        }
    }
}

/********************************************************************************
 *Net_Tx                                                                         *
 *���ڷ��ͺ���                                                                   *
 ********************************************************************************/
void Net_Tx(SPI_TypeDef *SPIx, unsigned char *TxData)
{
    unsigned short int i, j, k;
    unsigned short int tx_offset, tx_offset1;
    int Index;

    Index = (SPIx == SPI3);

    for (i = 0; i < 2; i++) {
        if (Net_Enable[Index][i] && Net_Tx_Flag[Index][i]  // �ⲿ�������÷��ͱ�־
            && (Net_Socket_State[Index][i] ==
                   (S_INIT | S_CONN | S_OK)))  // �����˴������׽��������ӳɹ�
        {
            Net_Socket_Data[Index][i] &= ~S_TRANSMITOK;  // ������ͱ�־

            j = Net_Read_Byte(SPIx, Sn_TX_FSR0(i));
            j *= 256;
            j += Net_Read_Byte(SPIx, Sn_TX_FSR0(i) + 1);

            if (j > Net_Tx_size[Index][i])  // ʣ�෢�ͻ��峤�ȴ��ڷ��ͳ���
            {
                // ��ȡ���ͻ�������ƫ����
                tx_offset = Net_Read_Byte(SPIx, Sn_TX_WR0(i));
                tx_offset *= 256;
                tx_offset += Net_Read_Byte(SPIx, Sn_TX_WR0(i) + 1);
                tx_offset1 = tx_offset;

                j = tx_offset / S_BUF_SIZE;
                k = tx_offset - j * S_BUF_SIZE;
                j = k + Net_Tx_size[Index][i];

                if (j > S_BUF_SIZE)  // �������ͻ���, ��Ҫ������д��
                {
                    j = S_BUF_SIZE - k;
                    Net_Write(SPIx, TX_MEM + i * S_BUF_SIZE + k, TxData, j);
                    Net_Write(SPIx, TX_MEM + i * S_BUF_SIZE, &TxData[j], Net_Tx_size[Index][i] - j);
                }
                else {
                    Net_Write(SPIx, TX_MEM + i * S_BUF_SIZE + k, TxData, Net_Tx_size[Index][i]);
                }

                // ������һ�ε�ƫ����
                tx_offset1 += Net_Tx_size[Index][i];
                Net_Tx_Addr_Offset[Index][i] = tx_offset1;
                Net_Write_Byte(SPIx, Sn_TX_WR0(i), (tx_offset1 / 256));
                Net_Write_Byte(SPIx, (Sn_TX_WR0(i) + 1), tx_offset1);

                Net_Write_Byte(SPIx, Sn_CR(i), Sn_CR_SEND);  // Send SEND command, start send process
                Net_Tx_Flag[Index][i] = 0;

                // Net_tx_process(SPIx, i, Net_Tx_Buf[Index][i]);
            }
        }
    }
}

/********************************************************************
 *Net_Mode_Select                                                    *
 *���ڹ���ģʽ��ѡ�񡢷��������������Ϳͻ��˽������ӵĺ���
 **
 ********************************************************************/
void Net_Mode_Select(SPI_TypeDef *SPIx, unsigned char s)
{
    int Index;

    Index = (SPIx == SPI3);
    if (Net_Socket_State[Index][s] == 0) {
        if (Net_State[Index] == S_IDLING)  // ȷ�����ͺͽ��ն��Ѿ����
        {
            Net_State[Index] = S_CHECKING;                 // ����״̬��������
            if (Socket_Mode(SPIx, s, Net_Mode[Index][s]))  // �򿪶˿�, ��������
            {
                Net_Socket_State[Index][s] = S_INIT;
            }
            else {
                Net_Socket_State[Index][s] = 0;
            }
            Net_State[Index] = S_IDLING;
        }
    }
    else if (Net_Socket_State[Index][s] == (S_INIT | S_CONN)) {
        if (Net_Mode[Index][s] == Server) {
            if (Net_State[Index] == S_IDLING) {
                Net_State[Index] = S_CHECKING;
                if (DMA_Tx_Size[Index] == DMA_SIZE_128 || DMA_Tx_Size[Index] == DMA_SIZE_256 ||
                    DMA_Tx_Size[Index] == DMA_SIZE_512 || DMA_Tx_Size[Index] == DMA_SIZE_1K ||
                    DMA_Tx_Size[Index] == DMA_SIZE_2K || DMA_Tx_Size[Index] == DMA_SIZE_4K ||
                    DMA_Tx_Size[Index] == DMA_SIZE_8K) {
                    Socket_Correct_Addr(SPIx, s);  // Socket0У��
                }
                Net_Socket_State[Index][s] = S_INIT | S_CONN | S_OK;  // ��ַУ׼���
                Net_State[Index] = S_IDLING;
            }
        }
    }
}

/********************************************************************
 *Net_State_Check                                                    *
 *���ڹ���״̬�Ķ�ȡ����ش����ĺ���                                 *
 ********************************************************************/
void Net_State_Check(SPI_TypeDef *SPIx)
{
    int Index;

    Index = (SPIx == SPI3);
    if (Net_State[Index] == S_IDLING) {
        Net_State[Index] = S_CHECKING;  // ����״̬��������
        if (((Net_Socket_State[Index][0] & S_INIT) == S_INIT) | ((Net_Socket_State[Index][1] & S_INIT) == S_INIT)) {
            Net_Interrupt_Process(SPIx);  // �¼���������
        }
        Net_State[Index] = S_IDLING;
    }
}

/********************************************************************
 *Net_Cycle                                                          *
 *����ͨ��ѭ������                                                   *
 ********************************************************************/
void Net_Cycle(SPI_TypeDef *SPIx)
{
    unsigned char i;
    int Index;

    Index = (SPIx == SPI3);
    for (i = 0; i < 2; i++) {
        if (Net_Enable[Index][i]) {
            Net_Mode_Select(SPIx, i);
        }
    }

    if ((++Net_Check_Delay[Index] > 1200) && (Net_State[Index] == S_IDLING))  // 1.2s
    {
        Net_Check_Delay[Index] = 0;
        if (Net_State[Index] == S_IDLING) {
            Net_State[Index] = S_CHECKING;

            for (i = 0; i < 2; i++) {
                if (Net_Enable[Index][i] && (Net_Socket_State[Index][i] & S_INIT)) {
                    Net_Write_Byte(SPIx, Sn_CR(i), S_CR_SEND_KEEP);
                }
            }
            Net_State[Index] = S_IDLING;
        }
    }
    Net_State_Check(SPIx);
}

/********************************************************************************
 *Net_DMA_Rx                                                                     *
 *���ڽ��պ���                                                                   *
 ********************************************************************************/
void Net_DMA_Rx(SPI_TypeDef *SPIx)
{
    unsigned char i;
    int Index;

    Index = (SPIx == SPI3);

    if (Net_State[Index] == S_RECEIVING)  // �������ڽ�������
    {
        if (Net_DMA_rx_process(SPIx, Net_Receiving_Socket[Index]))  // ���ݴ洢��Net_Rx_Buf[Index][s]
        {
            Net_Rx_Flag[Index][Net_Receiving_Socket[Index]] = 1;  // ֻ��λ�������ݴ��������
            Net_Receiving_Socket[Index] =
                8;  // �����ǰ�����׽�����ţ�ָ�򲻴��ڵ��׽���
            Net_Rx_Delay[Index] = 0;
        }
        else {
            if (++Net_Rx_Delay[Index] >= 100)  // δ���DMA��������
            {
                Net_Rx_Delay[Index] = 0;
                Net_Receiving_Socket[Index] = 8;
                Net_State[Index] = S_IDLING;
                Net_Socket_State[Index][i] = 0;  // �׽������Ӹ�λ
            }
        }
    }
    else {
        for (i = 0; i < 2; i++) {
            if (Net_Enable[Index][i] &&
                (Net_Socket_State[Index][i] ==
                    (S_INIT | S_CONN |
                        S_OK)) &&  // �����˴������׽��������ӳɹ�
                (Net_Socket_Data[Index][i] & S_RECEIVE))  // �յ�����
            {
                Net_Receiving_Socket[Index] = i;  // ָ��ǰ�׽���
                Net_DMA_rx_process(SPIx, i);
                break;
            }
        }
    }
}

/********************************************************************
 *Net_DMA_tx_process                                                 *
 *���ڲ���DMA��ʽ�������ݷ��͵Ĺ��̣����ݷ��ͳ���Ϊ128               *
 ********************************************************************/
unsigned char Net_DMA_tx_process(SPI_TypeDef *SPIx, unsigned char s, unsigned char *tx_buffer)
{
    unsigned char i;
    int j;
    int Index;
    unsigned char Write_Command = 0x80;

    Index = (SPIx == SPI3);

    i = Failure;

    if (Net_State[Index] == S_IDLING)  // ������У�����������
    {
        Net_State[Index] = S_SENDING;  // ���뷢��״̬

        SPI_Tx_Buf[Index][2] = DMA_Tx_Size[Index] >> 8;  // ����
        SPI_Tx_Buf[Index][3] = DMA_Tx_Size[Index] & 0xff;
        SPI_Tx_Buf[Index][2] |= Write_Command;                                      // ģʽ������
        SPI_Tx_Buf[Index][0] = (unsigned char)(Net_Socket_Tx_Addr[Index][s] >> 8);  // ���ͻ�������ַ(��8λ)
        SPI_Tx_Buf[Index][1] = (unsigned char)(Net_Socket_Tx_Addr[Index][s]);  // ���ͻ�������ַ(��8λ)
        for (j = 0; j < DMA_Tx_Size[Index]; j++) {
            SPI_Tx_Buf[Index][j + 4] = tx_buffer[j];
        }
        Net_Socket_Tx_Addr[Index][s] += DMA_Tx_Size[Index];
        if (Net_Socket_Tx_Addr[Index][s] >= (0x8000 + s * S_BUF_SIZE + S_BUF_SIZE)) {
            Net_Socket_Tx_Addr[Index][s] = 0x8000 + s * S_BUF_SIZE;
        }
        SPI_Cmd(SPIx, ENABLE);
        if (Index) {
            DMA_Cmd(DMA1_Stream7, ENABLE);
        }
        else {
            DMA_Cmd(DMA1_Stream4, ENABLE);
        }

        i = Failure;
    }
    else if (Net_State[Index] == S_SENDING) {
        if (Index) {
            j = DMA_GetFlagStatus(DMA1_Stream7, DMA_FLAG_TCIF7);
        }
        else {
            j = DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4);
        }
        if (j == SET) {
            if (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_BSY) == 0) {
                if (Index) {
                    DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);  // �����־
                }
                else {
                    DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
                }

                SPI_Cmd(SPIx, DISABLE);

                Net_Tx_Addr_Offset[Index][s] += DMA_Tx_Size[Index];
                Net_Write_Byte(SPIx, Sn_TX_WR0(s), (Net_Tx_Addr_Offset[Index][s] / 256));
                Net_Write_Byte(SPIx, (Sn_TX_WR0(s) + 1), Net_Tx_Addr_Offset[Index][s]);
                Net_Write_Byte(SPIx, Sn_CR(s), Sn_CR_SEND);  // Send SEND command, start send process

                Net_State[Index] = S_IDLING;  // ������ɣ��������״̬
                Net_Tx_Flag[Index][Net_Sending_Socket[Index]] = 0;
                Net_Sending_Socket[Index] = 8;
                i = Success;
            }
        }
    }
    return i;
}

/********************************************************************************
 *Net_DMA_Tx                                                                     *
 *���ڷ��ͺ���                                                                   *
 ********************************************************************************/
void Net_DMA_Tx(SPI_TypeDef *SPIx, unsigned char *TxData)
{
    unsigned char i;
    int Index;

    Index = (SPIx == SPI3);

    if (Net_State[Index] == S_SENDING)  // �������ڷ�������
    {
        if (Net_DMA_tx_process(SPIx, Net_Sending_Socket[Index], TxData))  // ������� , 1�����ڽ�ʹ��һ��socket
        {
            Net_Tx_Delay[Index] = 0;
        }
        else {
            if (++Net_Tx_Delay[Index] >= 100)  // DMA���ͳ�ʱ
            {
                Net_Tx_Delay[Index] = 0;
                Net_Sending_Socket[Index] = 8;
                Net_Socket_State[Index][Net_Sending_Socket[Index]] = 0;  // �׽��ָ�λ
                Net_State[Index] = S_IDLING;
            }
        }
    }
    else if (Net_State[Index] == S_IDLING) {
        for (i = 0; i < 2; i++) {
            if (Net_Enable[Index][i] && Net_Tx_Flag[Index][i]  // �ⲿ�������÷��ͱ�־
                &&
                (Net_Socket_State[Index][i] ==
                    (S_INIT | S_CONN | S_OK)))  // �����˴������׽��������ӳɹ�
            {
                Net_Sending_Socket[Index] = i;                                // ָ��ǰ�׽���
                Net_Socket_Data[Index][i] &= ~S_TRANSMITOK;                   // ������ͱ�־
                Net_DMA_tx_process(SPIx, Net_Sending_Socket[Index], TxData);  // ��������
                break;  // һ�ν��ܴ���һ���׽��ַ���
            }
        }
    }
}

/********************************************************************************
 *Net_Comm                                                                       *
 *����ͨ�ź���                                                                   *
 ********************************************************************************/
void Net_Comm(SPI_TypeDef *SPIx, unsigned char *TxData)
{
    Net_Cycle(SPIx);
    if (DMA_Tx_Size[SPIx == SPI3])  //
    {
        Net_DMA_Tx(SPIx, TxData);
        Net_DMA_Rx(SPIx);
    }
    else {
        Net_Tx(SPIx, TxData);
        Net_Rx(SPIx);
    }
}
//===========================================================================
// End of file.
//===========================================================================
