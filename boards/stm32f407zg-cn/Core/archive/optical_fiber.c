/************************************************************************************
 *����ͨ������                                                                       *
 *                                                                                   *
 *                                                                                   *
 ************************************************************************************/
#include "fpga.h"  // FPGA configuration functions
#include "pub.h"
#include "stm32f4xx_fsmc.h"

#define Bank1_SRAM4_ADDR ((uint32_t)0x6C000000)
#define KR_KEY_RELOAD    ((unsigned short int)0xAAAA)
#define KR_KEY_ENABLE    ((unsigned short int)0xCCCC)

/********************************************************************************
 *Optical_fiber_FSMC_Init                                                        *
 *FSMC��������FPGA����ͨ��                                                       *
 ********************************************************************************/
void Optical_fiber_FSMC_Init(void)
{
    FSMC_NORSRAMInitTypeDef FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef p;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIOs clock
    RCC_AHB1PeriphClockCmd(
        RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF | RCC_AHB1Periph_GPIOG, ENABLE);

    // Enable FSMC clock
    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

    //-- GPIOs Configuration -----------------------------------------------------
    /*
     +-------------------+--------------------+------------------+------------------+
     +                       pins assignment                                  +
     +-------------------+--------------------+------------------+------------------+
     | PD0   <-> FSMC_D2  |(PE0  <-> FSMC_NBL0)| PF0  <-> FSMC_A0 | PG0 <-> FSMC_A10 |
     | PD1   <-> FSMC_D3  |(PE1  <-> FSMC_NBL1)| PF1  <-> FSMC_A1 | PG1 <-> FSMC_A11 |
     | PD4   <-> FSMC_NOE | PE7  <-> FSMC_D4   | PF2  <-> FSMC_A2 | PG2 <-> FSMC_A12 |
     | PD5   <-> FSMC_NWE | PE8  <-> FSMC_D5   | PF3  <-> FSMC_A3 | PG3 <-> FSMC_A13 |
     | PG12  <-> FSMC_NE4 | PE9  <-> FSMC_D6   | PF4  <-> FSMC_A4 | PG4 <-> FSMC_A14 |
     | PD8   <-> FSMC_D13 | PE10 <-> FSMC_D7   | PF5  <-> FSMC_A5 | PG5 <-> FSMC_A15 |
     | PD9   <-> FSMC_D14 | PE11 <-> FSMC_D8   | PF12 <-> FSMC_A6 |------------------+
     | PD10  <-> FSMC_D15 | PE12 <-> FSMC_D9   | PF13 <-> FSMC_A7 |
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
    //	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | /* GPIO_Pin_7 |*/
                                  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_14 |
                                  GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // GPIOE configuration
    //	GPIO_PinAFConfig(GPIOE, GPIO_PinSource0 , GPIO_AF_FSMC);
    //	GPIO_PinAFConfig(GPIOE, GPIO_PinSource1 , GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = /*GPIO_Pin_0  | GPIO_Pin_1 |*/ GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // GPIOF configuration
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

    GPIO_Init(GPIOF, &GPIO_InitStructure);

    // GPIOG configuration
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource12, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin =
        GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12;

    GPIO_Init(GPIOG, &GPIO_InitStructure);

    // FSMC Configuration
    p.FSMC_AddressSetupTime = 6;
    p.FSMC_AddressHoldTime = 15;
    p.FSMC_DataSetupTime = 21;
    p.FSMC_BusTurnAroundDuration = 1;
    p.FSMC_CLKDivision = 1;
    p.FSMC_DataLatency = 0;
    p.FSMC_AccessMode = FSMC_AccessMode_A;

    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
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

    //!< Enable FSMC Bank1_SRAM4 Bank
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}

/********************************************************************************
 *��������BaudRate_Init                                                          *
 *���ܣ� ��ʼ������ͨ�ŵ�ͨ�Ų�����, BaudRate�����ܱ�1382400����                 *
 ********************************************************************************/
void BaudRate_Init(uint32_t BaudRate)
{
    uint32_t data;

    data = (1382400 / BaudRate);
    *(unsigned short int*)(Bank1_SRAM4_ADDR + 0x04) = data;
}

/********************************************************************************
 *��������Optical_fiber_Init                                                     *
 *��  �ܣ�����ͨ�ų�ʼ������                                                     *
 *��  ����BaudRate��	 ���ô���ͨ�ŵĲ����ʣ�	��ֵ�����ܱ�1382400����             *
 * Note: Must be called AFTER TIMx_Init() for timer-based delays                *
 ********************************************************************************/
void Optical_fiber_Init(uint32_t BaudRate)
{
    FPGA_Status_t fpga_status;

    // Initialize FSMC for FPGA communication
    Optical_fiber_FSMC_Init();

    // Initialize FPGA control pins
    FPGA_Init_Pins();

    // Configure FPGA with 5 second timeout
    fpga_status = FPGA_Configure(5000);

    if (fpga_status != FPGA_CONFIG_SUCCESS) {
        // FPGA configuration failed/timeout
        // System can continue but optical fiber won't work
        // Could set an error flag here if needed
        return;
    }

    // FPGA configured successfully - perform internal reset sequence
    FPGA_Reset();

    // Configure baud rate
    BaudRate_Init(BaudRate);

    // Set node number
    *(unsigned short int*)(Bank1_SRAM4_ADDR + 16) = (unsigned short int)uchNodeNum;
}

/********************************************************************************
 *��������Tx_Buffer                                                              *
 *��  �ܣ������ַ������ͺ���                                                     *
 *��  ����port ����ͨ��ѡ��                                                      *
 *        - 0  ������1��2��3��4��5��6ʱ�㲥����                                  *
 *        - 1  ������1                                                           *
 *        - 2  ������2                                                           *
 *        - 3  ������3                                                           *
 *        - 4  ������4                                                           *
 *        - 5  ������5                                                           *
 *        - 6  ������6                                                           *
 *        - 100������1��2��3��4�㲥����                                          *
 *        - 101������5��6�㲥����                                                *
 *        data[] ���ݷ��ͻ�����                                                  *
 *        len ���ݷ��ͳ���                                                       *
 ********************************************************************************/
void Tx_Buffer(unsigned short int port, unsigned char data[], unsigned short int len)
{
    unsigned short int i;
    for (i = 0; i < len; i++) {
        *(unsigned short int*)(Bank1_SRAM4_ADDR + (port + 100) * 2) = (unsigned short int)data[i];
    }
}

/********************************************************************************
 *��������Read_State                                                             *
 *��  �ܣ���ȡFPGA���ڽ���״̬                                                   *
 *��  ����addr ��ȡ��ַ                                                          *
 ********************************************************************************/
unsigned short int Read_State(unsigned short int addr)
{
    unsigned short int ret;
    ret = *(__IO unsigned short int*)(Bank1_SRAM4_ADDR + addr * 2);
    return ret;
}

/********************************************************************************
 *��������Rx_Buffer                                                              *
 *��  �ܣ��������ݽ��պ���                                                       *
 *��  ����port ����ͨ��ѡ��                                                      *
 *        - 1������1                                                             *
 *        - 2������2                                                             *
 *        - 3������3                                                             *
 *        - 4������4                                                             *
 *        - 5������5                                                             *
 *        - 6������6                                                             *
 *        data[] ���ݽ��ջ�����                                                  *
 *����ֵ�����ݽ��ճ���                                                           *
 ********************************************************************************/
unsigned short int Rx_Buffer(unsigned short int port, unsigned char data[])
{
    unsigned short int len = 0, i = 0;
    while (((Read_State(port + 100) & 0x0002) == 0) && (i++ < 255)) {
        data[len] = (unsigned char)Read_State(port);
        len++;
    }
    return len;
}

//===========================================================================
// End of file.
//===========================================================================
