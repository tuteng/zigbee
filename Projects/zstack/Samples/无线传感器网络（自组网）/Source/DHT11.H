#ifndef __DHT11_H__
#define __DHT11_H__

#define uint unsigned int
#define U8 unsigned char
#define NOP()  asm("NOP")

#define Red_LED     P1_4 

//unsigned char WenDu,ShiDu;//定义温度存放变量

//#define wenshi P0_0


#define DATA_PIN P0_7

/*******函数声明*********/
void Delay_us(void); //1 us延时
void Delay_10us(void); //10 us延时
void Delay_ms(uint Time);//n ms延时
void COM(void);	// 温湿写入
unsigned char * DHT11(void) ;  //温湿传感启动


extern void halMcuWaitUs(unsigned short usec);

//static uint8 pTxData[APP_PAYLOAD_LENGTH];
//static basicRfCfg_t basicRfConfig;

//uint16 count=0;




#endif