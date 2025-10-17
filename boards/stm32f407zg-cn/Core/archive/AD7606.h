  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AD7606_H
#define __AD7606_H

#define ADC_CHANNNEL       0x08					 	 //
#define WRITE_READ_ADDR    0x800

void Init_AD7606(void);
void Start_Read_AD7606(unsigned short int* pBuffer);
void Read_Start_AD7606(unsigned short int* pBuffer);					   
void Start_AD7606(void);
void Read_AD7606(unsigned short int* pBuffer);

#endif   



