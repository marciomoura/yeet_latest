/********************************************************************
 * COPYRIGHT --  
 ********************************************************************
 * Program: Ex
 * File: mode.c
 * Author: Yu Xiang
 * Created: Jan. 15, 2018
 *******************************************************************/

/********************************************************************
*mode.c                                                             *
*                                                                   *
*方式切换程序.                                                      *
*Copyright: 2002-2018 Nengshida Electric Ltd.	                      *
*Author: Yu Xiang                                                   *
*Jan. 15, 2018                                                     	*
********************************************************************/
#include "pub.h"
#include<math.h>

#define V_Ref_Min_0 0.1				//空载时电压给定值下限
#define V_Ref_Max_0 1.15			//空载时电压给定值上限
#define V_Ref_Min_1 0.7				//并网时电压给定值下限
#define V_Ref_Max_1 1.1				//并网时电压给定值上限
#define V_Ref_Step 4e-6			    //增磁步长


#define Ifd_Ref_Min_0 0.05			//空载时励磁电流给定值下限
#define Ifd_Ref_Max_0 0.65			//空载时励磁电流给定值上限
#define Ifd_Ref_Min_1 0.3			//并网时励磁电流给定值下限
#define Ifd_Ref_Max_1 1.095			//并网时励磁电流给定值上限
#define Ifd_Ref_Step 2.0e-6			//励磁电流的増减磁步长


/********************************************************************
*  work_mode_v                                                      *
*  ref = Vref                                                       *
********************************************************************/
void Work_Mode_V(float Vref)
{
	AVR_Flag=1;
	AER_Flag=0;
	FCR_Flag=0;
	
	Ref_Ctrl=Vref;
	Ref_Step=V_Ref_Step;
	
	if (Breaker_Flag)
	{
		Ref_Max=V_Ref_Max_1;	//给定值范围
		Ref_Min=V_Ref_Min_1;		
	}
	else
	{
		Ref_Max=V_Ref_Max_0;
		Ref_Min=V_Ref_Min_0;			
	}
	if (Ref_Ctrl>Ref_Max)
	{
		Ref_Ctrl=Ref_Max;
	}
	if (Ref_Ctrl<Ref_Min)
	{
		Ref_Ctrl=Ref_Min;
	}

	if (Breaker_Flag  )
	{
		Ref=Ref_Ctrl+Kvrefq*Qav;
	}
	else
	{
		Ref=Ref_Ctrl;
	}
	Ref_Step_Limit=0.05f;
}

/********************************************************************
*  work_mode_Ifd                                                    *
*  ref = Ifd_ref                                                    *
********************************************************************/
void Work_Mode_Ifd(float Ifd_ref)
{
	AVR_Flag=0;
	AER_Flag=0;
	FCR_Flag=1;
	
	Ref_Step=Ifd_Ref_Step;
	
	if (Breaker_Flag)
	{
		if (Ifd_Max_Limit<(float)Ifd_Ref_Max_1)
		{
			Ref_Max=(float)Ifd_Max_Limit;
		}
		else
		{
			Ref_Max=Ifd_Ref_Max_1;
		}
		Ref_Min=Ifd_Ref_Min_1;		
	}
	else
	{
		Ref_Max=Ifd_Ref_Max_0;
		Ref_Min=Ifd_Ref_Min_0;
	}
	
	if (Ref_Ctrl>Ref_Max)
	{
		Ref_Ctrl=Ref_Max;
	}
	if (Ref_Ctrl<Ref_Min)
	{
		Ref_Ctrl=Ref_Min;
	}
	Ref=Ref_Ctrl;
	Ref_Step_Limit=0.05f;/*单次増减磁允许调节范围*/
}

/********************************************************************
*  work_mode_Vfd                                                    *
*  ref = Q_ref                                                      *
********************************************************************/
void Work_Mode_Vfd(float Vfd_ref)
{
	AVR_Flag=0;
	AER_Flag=1;
	FCR_Flag=0;
	
	if (Breaker_Flag)
	{
		Ref_Max=V_Ref_Max_1;	//给定值范围
		Ref_Min=V_Ref_Min_1;		
	}
	else
	{
		Ref_Max=V_Ref_Max_0;
		Ref_Min=V_Ref_Min_0;			
	}
	if (Ref_Ctrl>Ref_Max)
	{
		Ref_Ctrl=Ref_Max;
	}
	if (Ref_Ctrl<Ref_Min)
	{
		Ref_Ctrl=Ref_Min;
	}

	Ref=Ref_Ctrl;
	Ref_Step_Limit=0.05f;
}


//===========================================================================
// End of file.
//===========================================================================

