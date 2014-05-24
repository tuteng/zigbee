//---------------温湿度传感器 DHT11----------------//
/*                  Creat By ES                       */
/*           http://es-tech.taobao.com          */
/******************************************************
实验内容：
1.测量温湿度传感器的温度和湿度数据并显示在液晶上。

注意：
DHT11的Vcc引脚连接CC2530模块的VCC，GND连接CC2530模块的GND，数据
引脚连接CC2530模块的P00引脚。

******************************************************/

#include <ioCC2530.h>
//#include "JLX12864.h" 
#include "DHT11.h" 







void halMcuWaitUs(unsigned short usec)
{
    usec>>= 1;
    while(usec--)
    {
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
    }
}

//温湿度定义
U8 U8FLAG,U8temp;
U8 ShiDu_H,ShiDu_L;//定义湿度存放变量
U8 WenDu,ShiDu;//定义温度存放变量
U8 U8T_data_H,U8T_data_L,U8RH_data_H,U8RH_data_L,U8checkdata;
U8 U8T_data_H_temp,U8T_data_L_temp,U8RH_data_H_temp,U8RH_data_L_temp,U8checkdata_temp;
U8 U8comdata;







/****************************
//延时函数
*****************************/
void Delay_us(void) //1 us延时

{
    halMcuWaitUs(1);   
}

void Delay_10us(void) //10 us延时
{
   halMcuWaitUs(10);
}

void Delay_ms(uint Time)//n ms延时
{
  unsigned char i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delay_10us();
  }
}


/***********************
   温湿度传感
***********************/
void COM(void)	// 温湿写入
{     
    U8 i;         
    for(i=0;i<8;i++)    
    {
     U8FLAG=2; 
     DATA_PIN=0;
     DATA_PIN=1;
     while((!DATA_PIN)&&U8FLAG++);
     Delay_10us();
     Delay_10us();
     Delay_10us();
     U8temp=0;
     if(DATA_PIN)U8temp=1;
     U8FLAG=2;
     while((DATA_PIN)&&U8FLAG++);   
     if(U8FLAG==1)break;    
     U8comdata<<=1;
     U8comdata|=U8temp; 
     }    
}

//-------------------------------- 　　
//-----湿度读取子程序 ------------ 　　
//-------------------------------- 　　
//----以下变量均为全局变量-------- 　　
//----温度高8位== U8T_data_H------ 　　
//----温度低8位== U8T_data_L------ 　　
//----湿度高8位== U8RH_data_H----- 　　
//----湿度低8位== U8RH_data_L----- 　　
//----校验 8位 == U8checkdata----- 　　
//----调用相关子程序如下---------- 　　
//---- Delay();, Delay_10us();COM(); 　　
//-------------------------------- 

unsigned char * DHT11(void)   //温湿传感启动
{ 
    unsigned char *data2;
    data2 = 0;
    DATA_PIN=0;
    Delay_ms(19);  //主机拉低18ms
    DATA_PIN=1;     //总线由上拉电阻拉高 主机延时40us 
    P0DIR &= ~0x01; //重新配置IO口方向
    Delay_10us();
    Delay_10us();						
    Delay_10us();
    Delay_10us();  
    //判断从机是否有低电平响应信号 如不响应则跳出，响应则向下运行 
     if(!DATA_PIN) 
     {
      U8FLAG=2; //判断从机是否发出 80us 的低电平响应信号是否结束 
      while((!DATA_PIN)&&U8FLAG++);
      U8FLAG=2;//判断从机是否发出 80us 的高电平，如发出则进入数据接收状态
      while((DATA_PIN)&&U8FLAG++); 
      COM();//数据接收状态 
      U8RH_data_H_temp=U8comdata;
      COM();
      U8RH_data_L_temp=U8comdata;
      COM();
      U8T_data_H_temp=U8comdata;
      COM();
      U8T_data_L_temp=U8comdata;
      COM();
      U8checkdata_temp=U8comdata;
      DATA_PIN=1; 
      //数据校验 
      U8temp=(U8T_data_H_temp+U8T_data_L_temp+U8RH_data_H_temp+U8RH_data_L_temp);
       if(U8temp==U8checkdata_temp)
      {
          U8RH_data_H=U8RH_data_H_temp;
          U8RH_data_L=U8RH_data_L_temp;
          U8T_data_H=U8T_data_H_temp;
          U8T_data_L=U8T_data_L_temp;
          U8checkdata=U8checkdata_temp;
       }
       WenDu=U8T_data_H;
       ShiDu=U8RH_data_H;
    } 
    else 
    {  
      WenDu=0;
      ShiDu=0;
    } 
    P0DIR |= 0x01;
    
 //   *data2 = WenDu;
   *data2 = ShiDu;
//   *(data2+2) = U8T_data_L;
 //   data[3] = 0;
    return data2;
}


