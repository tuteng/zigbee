#ifndef DS18B20_H
#define DS18B20_H

#include "ioCC2530.h"


static char sound = 0,vibrat = 0,infra = 0;
#define SoundSensor       P0_0    //P00连接声音传感器的开关信号引脚
//#define anaSoundSensor  P0_1
//P0_1:声音模拟量
#define VibrationSensor   P0_2    //P02连接震动传感器的开关信号引脚
//#define anaVibrationSensor  P0_3
//P0_3:震动模拟量
#define InfraredSensor    P0_4    //P04连接红外传感器的开关信号引脚
//#include "hal.h"

#define CL_DQ  P1_1=0
#define SET_DQ P1_1=1 
#define SET_OUT P1DIR|=0x02
#define SET_IN  P1DIR&=~0x02
#define IN_DQ  P1_1
#define Yellow_LED  P1_5

#define CL_DQ0  P1_0=0
#define SET_DQ0 P1_0=1 
#define SET_OUT0 P1DIR|=0x01
#define SET_IN0  P1DIR&=~0x01
#define IN_DQ0  P1_0

extern unsigned char id[8];
extern unsigned char sensor_data_value[2];
extern unsigned char flag;

extern unsigned char id0[8];


//void Delay_1us(void); 
void Delay_nus(unsigned int n) ;
void write_1820(unsigned char x) ; 
unsigned char read_1820(void);  
void init_1820(void) ; 
unsigned int read_data1(unsigned char *buffer );
void get_id(void);
void Match_DS18B20(unsigned char * buffer);
void IO_initial();


void write0_1820(unsigned char x) ;
unsigned char read0_1820(void); 
void init0_1820(void) ; 
unsigned int read_data0(unsigned char *buffer );
void Match0_DS18B20(unsigned char * buffer);
void ds18b20_main0(void);
void get0_id(void);
void int_init(void);

void Delayms(unsigned int xms);

#endif

