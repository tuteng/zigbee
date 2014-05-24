
#include "DS18B20.h"

unsigned char id[8];
unsigned char id0[8];
unsigned char sensor_data_value[2];
unsigned char flag; 

void Delay_nus(unsigned int s) 
{
  while (s--)
  {
    asm("NOP");
    asm("NOP");
    asm("NOP");
  }
}

void init_1820(void) 
{
  SET_OUT; 
  SET_DQ;//输出1
  CL_DQ; 
  Delay_nus(550);//拉低一段时间 
  SET_DQ;//释放 
  SET_IN;//输入 
  Delay_nus(40); //释放总线后等待15-60us
  while(IN_DQ)  {;}//等待回复 
  Delay_nus(240);//回复的低电平在60到240us  
  SET_OUT; 
  SET_DQ;//回到初始DQ=1；
}

void init0_1820(void) 
{
  SET_OUT0; 
  SET_DQ0;//输出1
  CL_DQ0; 
  Delay_nus(550);//拉低一段时间 
  SET_DQ0;//释放 
  SET_IN0;//输入 
  Delay_nus(40); //释放总线后等待15-60us
  while(IN_DQ0)  {;}//等待回复 
  Delay_nus(240);//回复的低电平在60到240us  
  SET_OUT0; 
  SET_DQ0;//回到初始DQ=1；
}

void write_1820(unsigned char x)  
{  
  unsigned char m;    
  SET_OUT;
  for(m=0;m<8;m++)  
  {  
     CL_DQ;
     if(x&(1<<m))    //写数据，从低位开始  
     SET_DQ; 
     else  
     CL_DQ;  
     Delay_nus(40);   //15~60us  
     SET_DQ;  
  }
  SET_DQ;  
} 
void write0_1820(unsigned char x)  
{  
  unsigned char m;    
  SET_OUT0;
  for(m=0;m<8;m++)  
  {  
     CL_DQ0;
     if(x&(1<<m))    //写数据，从低位开始  
     SET_DQ0; 
     else  
     CL_DQ0;  
     Delay_nus(40);   //15~60us  
     SET_DQ0;  
  }
  SET_DQ0;  
} 

unsigned char read_1820(void)  
{   
  unsigned char temp,k,n;    
  temp=0;  
  for(n=0;n<8;n++)  
  {  
    CL_DQ;     
    SET_DQ; 
    SET_IN;  
    k=IN_DQ;    //读数据,从低位开始  
    if(k)  
    temp|=(1<<n);  
    else  
    temp&=~(1<<n);  
    Delay_nus(70); //60~120us      
    SET_OUT;
  
  }  
  return (temp);  
}  

unsigned char read0_1820(void)  
{   
  unsigned char temp,k,n;    
  temp=0;  
  for(n=0;n<8;n++)  
  {  
    CL_DQ0;     
    SET_DQ0; 
    SET_IN0;  
    k=IN_DQ0;    //读数据,从低位开始  
    if(k)  
    temp|=(1<<n);  
    else  
    temp&=~(1<<n);  
    Delay_nus(70); //60~120us      
    SET_OUT0;
  
  }  
  return (temp);  
}  

//搜索ID
void get_id()
{     
  init_1820();        //resert
  write_1820(0x33); 
  
  for(unsigned char ii=0;ii<8;ii++)
  {
    id[ii]=read_1820();
  }
}

void get0_id()
{     
  init0_1820();        //resert
  write0_1820(0x33); 
  
  for(unsigned char ii=0;ii<8;ii++)
  {
    id0[ii]=read0_1820();
  }
}


void Match_DS18B20(unsigned char *buffer)
{
 unsigned char i;
 write_1820(0x55);
 for(i=0;i<8;i++){
   write_1820(buffer[i]);
   }
 } 
void Match0_DS18B20(unsigned char *buffer)
{
 unsigned char i;
 write0_1820(0x55);
 for(i=0;i<8;i++){
   write0_1820(buffer[i]);
   }
 } 






