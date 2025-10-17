#include "stm32f4xx.h"
#define KR_KEY_RELOAD    ((uint16_t)0xAAAA)
#define KR_KEY_ENABLE    ((uint16_t)0xCCCC)
/********************************************************************************
*WatchDog                                                                       *
*若使用看门狗，必须先使用IWDG_Init函数初始化看门狗,	并及时调用Feed_Dog喂狗      *
********************************************************************************/


/********************************************************************************
*WatchDog_Init                                                                  *
*独立看门狗初始化函数,time为看门狗的复位时间，最大值为0xFFF,单位：ms            *
********************************************************************************/
void WatchDog_Init(uint16_t time)
{
	DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP,ENABLE);	//调试中止程序时看门狗暂停
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)	//如果看门狗已复位则清除复位标志
  	{ 
    	RCC_ClearFlag();
  	}
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);				//看门狗的时钟频率为32KHz/32=1KHz
	IWDG_SetReload(time);
	IWDG_ReloadCounter();
	IWDG_Enable();	
}

/********************************************************************************
*Feed_Dog                                                                       *
*喂狗函数                                                                       *
********************************************************************************/
void Feed_Dog(void)
{
	//IWDG_ReloadCounter();
	IWDG->KR = KR_KEY_RELOAD;//复制了IWDG_ReloadCounter函数
}

//===========================================================================
// End of file.
//===========================================================================
