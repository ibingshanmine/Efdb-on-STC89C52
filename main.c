/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : 0x9e733e9
 * Version            : V0.0.1
 * Date               : 2023/11/29
 * Description        : Main program body.
 * Open Source License: GPL3.0
 * E-mail             : bsk166147049@163.com
 *******************************************************************************/
#include <STC89C5xRC.H>
#include "intrins.h"

#define scale	100   //pwm的周期
#define SMG_A_DP_PORT	P0  //数码管段选引脚
#define EEPROM_ADDRESS	0x00	//定义数据存入EEPROM的起始地址

typedef unsigned int u16;	//对系统默认数据类型进行重定义
typedef unsigned char u8;	//对系统默认数据类型进行重定义

//管脚定义
sbit DC_Motor=P1^0;//定义直流电机控制管脚
sbit KEY1=P3^1;
sbit KEY2=P3^0;
//sbit KEY3=P3^2;   //被红外占用P32io口
sbit KEY4=P3^3;
sbit LED1=P2^5;
sbit LED2=P2^6;
sbit LED3=P2^7;
sbit LSA=P2^2;  //数码管位选引脚定义
sbit LSB=P2^3;	//数码管位选引脚定义
sbit LSC=P2^4;	//数码管位选引脚定义
sbit IRED=P3^2; //红外引脚定义
sbit DS18B20_PORT=P3^7;		//DS18B20数据口定义
//定义EEPROM控制脚
sbit IIC_SCL=P2^1;//SCL时钟线
sbit IIC_SDA=P2^0;//SDA数据线
sbit DOUT = P3^7;	  //输出
sbit CLK  = P3^6;	  //时钟
sbit DIN  = P3^4;	  //输入
sbit CS   = P3^5;	  //片选
//字库
u8 gsmg_code[30]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71, 
                  0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef,
					0X76,0x73,0x00,0x40}; //H,P,buxianshi,- 
//变量
unsigned long int sys=0;  //定时器1系统时间
//unsigned int system=0;    
seg[8]={28,28,28,28,28,28,28,28}; //数码管外设数组
unsigned char key;       //按键变量
unsigned char key_sanf_delay,key_old;  //按键变量
unsigned char key_val,key_down,key_up;  //按键变量
unsigned long int seg_tag=0;     //数码管刷新依赖
//u8 dir=0;     
u16 duty=100;  //占空比，上电风扇初始状态
u8 gired_data[4];//存储4个字节接收码（地址码+地址反码+控制码+控制反码）
//#define DC_MOTOR_RUN_TIME	5000	//定义直流电机运行时间为5000ms
u8 mode=0;  //数码管界面切换
u8 one_tag=0; //界面切换需要
u8 wendu_fuhao;  //温度正负标志
//u8 save_value=0;  //
u8 wendu=20;  //温控模式温度设定值
u16 adc_value=0;  //adc模式，读取原始数据
float adc_vol;//ADC电压值
int temp_value; //转换后的温度绝对值 
//标志位
bit kaiji_tag=0; //开机标志
bit dianjiqidong_tag=0;  //电机启动标志
bit wenkongmoshi=0;  //温控模式标志
bit shoukongmoshi=1;  //手控模式标志
bit adckongmoshi=0;  //adc控制模式标志
bit guangming=0;  //adc的光敏模式标志
bit reming=0;    //adc的热敏模式标志
bit dianzu=1;    //adc的电阻模式标志
bit suoanjian=0;  //锁主机按键标志
//****************************************底层代码区*************************************************************
/*******************************************************************************
* 函 数 名       : delay_ms
* 函数功能		 : ms延时函数，ms=1时，大约延时1ms
* 输    入       : ms：ms延时时间
* 输    出    	 : 无
*******************************************************************************/
void delay_ms(u16 ms)
{
	u16 i,j;
	for(i=ms;i>0;i--)
		for(j=110;j>0;j--);
}
void delay_10us(u16 ten_us)
{
	while(ten_us--);
}

/*******************************************************************************
* 函 数 名       : Key_Read
* 函数功能		 : 矩阵按键扫描函数
* 输    入       : 无
* 输    出    	 : 键值
*******************************************************************************/
unsigned char Key_Read(void)
{
	if(!KEY1)return 17;
	else if(!KEY2)return 18;
	//else if(!KEY3)return 19;  //被红外占用P32io口
	else if(!KEY4)return 20;
	if(kaiji_tag==1)
	{
//    P10=0;P11=1;P12=1;P13=1;P14=1;P15=1;
//    if(!P14)return 16;
//    else if(!P15)return 12;
//    else if(!P16)return 8;
//    else if(!P17)return 4;
//    P10=1;
	P11=0;P12=1;P13=1;P14=1;P15=1;
    if(!P14)return 15;
    else if(!P15)return 11;
    else if(!P16)return 7;
    else if(!P17)return 3;
    P11=1;P12=0;
    if(!P14)return 14;
    else if(!P15)return 10;
    else if(!P16)return 6;
    else if(!P17)return 2;
    P12=1;P13=0;
    if(!P14)return 13;
    else if(!P15)return 9;
    else if(!P16)return 5;
    else if(!P17)return 1;
	}
    return 0;
}

/*******************************************************************************
* 函 数 名         : ds18b20_reset
* 函数功能		   : 复位DS18B20  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void ds18b20_reset(void)
{
	DS18B20_PORT=0;	//拉低DQ
	delay_10us(75);	//拉低750us
	DS18B20_PORT=1;	//DQ=1
	delay_10us(2);	//20US
}

/*******************************************************************************
* 函 数 名         : ds18b20_check
* 函数功能		   : 检测DS18B20是否存在
* 输    入         : 无
* 输    出         : 1:未检测到DS18B20的存在，0:存在
*******************************************************************************/
u8 ds18b20_check(void)
{
	u8 time_temp=0;

	while(DS18B20_PORT&&time_temp<20)	//等待DQ为低电平
	{
		time_temp++;
		delay_10us(1);	
	}
	if(time_temp>=20)return 1;	//如果超时则强制返回1
	else time_temp=0;
	while((!DS18B20_PORT)&&time_temp<20)	//等待DQ为高电平
	{
		time_temp++;
		delay_10us(1);
	}
	if(time_temp>=20)return 1;	//如果超时则强制返回1
	return 0;
}

/*******************************************************************************
* 函 数 名         : ds18b20_read_bit
* 函数功能		   : 从DS18B20读取一个位
* 输    入         : 无
* 输    出         : 1/0
*******************************************************************************/
u8 ds18b20_read_bit(void)
{
	u8 dat=0;
	
	DS18B20_PORT=0;
	_nop_();_nop_();
	DS18B20_PORT=1;	
	_nop_();_nop_(); //该段时间不能过长，必须在15us内读取数据
	if(DS18B20_PORT)dat=1;	//如果总线上为1则数据dat为1，否则为0
	else dat=0;
	delay_10us(5);
	return dat;
} 

/*******************************************************************************
* 函 数 名         : ds18b20_read_byte
* 函数功能		   : 从DS18B20读取一个字节
* 输    入         : 无
* 输    出         : 一个字节数据
*******************************************************************************/
u8 ds18b20_read_byte(void)
{
	u8 i=0;
	u8 dat=0;
	u8 temp=0;

	for(i=0;i<8;i++)//循环8次，每次读取一位，且先读低位再读高位
	{
		temp=ds18b20_read_bit();
		dat=(temp<<7)|(dat>>1);
	}
	return dat;	
}

/*******************************************************************************
* 函 数 名         : ds18b20_write_byte
* 函数功能		   : 写一个字节到DS18B20
* 输    入         : dat：要写入的字节
* 输    出         : 无
*******************************************************************************/
void ds18b20_write_byte(u8 dat)
{
	u8 i=0;
	u8 temp=0;

	for(i=0;i<8;i++)//循环8次，每次写一位，且先写低位再写高位
	{
		temp=dat&0x01;//选择低位准备写入
		dat>>=1;//将次高位移到低位
		if(temp)
		{
			DS18B20_PORT=0;
			_nop_();_nop_();
			DS18B20_PORT=1;	
			delay_10us(6);
		}
		else
		{
			DS18B20_PORT=0;
			delay_10us(6);
			DS18B20_PORT=1;
			_nop_();_nop_();	
		}	
	}	
}

/*******************************************************************************
* 函 数 名         : ds18b20_start
* 函数功能		   : 开始温度转换
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void ds18b20_start(void)
{
	ds18b20_reset();//复位
	ds18b20_check();//检查DS18B20
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0x44);//转换命令	
}

/*******************************************************************************
* 函 数 名         : ds18b20_init
* 函数功能		   : 初始化DS18B20的IO口 DQ 同时检测DS的存在
* 输    入         : 无
* 输    出         : 1:不存在，0:存在
*******************************************************************************/ 
u8 ds18b20_init(void)
{
	ds18b20_reset();
	return ds18b20_check();	
}

/*******************************************************************************
* 函 数 名         : ds18b20_read_temperture
* 函数功能		   : 从ds18b20得到温度值
* 输    入         : 无
* 输    出         : 温度数据
*******************************************************************************/
float ds18b20_read_temperture(void)
{
	float temp;
	u8 dath=0;
	u8 datl=0;
	u16 value=0;

	ds18b20_start();//开始转换
	ds18b20_reset();//复位
	ds18b20_check();
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0xbe);//读存储器

	datl=ds18b20_read_byte();//低字节
	dath=ds18b20_read_byte();//高字节
	value=(dath<<8)+datl;//合并为16位数据

	if((value&0xf800)==0xf800)//判断符号位，负温度
	{
		value=(~value)+1; //数据取反再加1
		temp=value*(-0.0625);//乘以精度	
	}
	else //正温度
	{
		temp=value*0.0625;	
	}
	return temp;
}

/*******************************************************************************
* 函 数 名       : ired_init
* 函数功能		 : 红外接受初始化函数
* 输    入       : 无
* 输    出    	 : 无
*******************************************************************************/
void ired_init(void)
{
	IT0=1;	//下降沿触发
	EX0=1;	//打开中断0允许
	EA=1;	//打开总中断
	IRED=1;	//初始化端口
}
/*******************************************************************************
* 函 数 名       : ired
* 函数功能		 : 红外接受中断函数，外部中断0触发
* 输    入       : 无
* 输    出    	 : 无
*******************************************************************************/
void ired() interrupt 0	//外部中断0服务函数
{
	u8 ired_high_time=0;
	u16 time_cnt=0;
	u8 i=0,j=0;

	if(IRED==0)
	{
		time_cnt=1000;
		while((!IRED)&&(time_cnt))//等待引导信号9ms低电平结束，若超过10ms强制退出
		{
			delay_10us(1);//延时约10us
			time_cnt--;
			if(time_cnt==0)return;		
		}
		if(IRED)//引导信号9ms低电平已过，进入4.5ms高电平
		{
			time_cnt=500;
			while(IRED&&time_cnt)//等待引导信号4.5ms高电平结束，若超过5ms强制退出
			{
				delay_10us(1);
				time_cnt--;
				if(time_cnt==0)return;	
			}
			for(i=0;i<4;i++)//循环4次，读取4个字节数据
			{
				for(j=0;j<8;j++)//循环8次读取每位数据即一个字节
				{
					time_cnt=600;
					while((IRED==0)&&time_cnt)//等待数据1或0前面的0.56ms结束，若超过6ms强制退出
					{
						delay_10us(1);
						time_cnt--;
						if(time_cnt==0)return;	
					}
					time_cnt=20;
					while(IRED)//等待数据1或0后面的高电平结束，若超过2ms强制退出
					{
						delay_10us(10);//约0.1ms
						ired_high_time++;
						if(ired_high_time>20)return;	
					}
					gired_data[i]>>=1;//先读取的为低位，然后是高位
					if(ired_high_time>=8)//如果高电平时间大于0.8ms，数据则为1，否则为0
						gired_data[i]|=0x80;
					ired_high_time=0;//重新清零，等待下一次计算时间
				}
			}
		}
		if(gired_data[2]!=~gired_data[3])//校验控制码与反码，错误则返回
		{
			for(i=0;i<4;i++)
				gired_data[i]=0;
			return;	
		}
	}		
}

/*******************************************************************************
* 函 数 名       : iic_start
* 函数功能		 : 产生IIC起始信号
* 输    入       : 无
* 输    出    	 : 无
*******************************************************************************/
void iic_start(void)
{
	IIC_SDA=1;//如果把该条语句放在SCL后面，第二次读写会出现问题
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=0;	//当SCL为高电平时，SDA由高变为低
	delay_10us(1);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据
	delay_10us(1);
}

/*******************************************************************************
* 函 数 名         : iic_stop
* 函数功能		   : 产生IIC停止信号   
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void iic_stop(void)
{	
	IIC_SDA=0;//如果把该条语句放在SCL后面，第二次读写会出现问题
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=1;	//当SCL为高电平时，SDA由低变为高
	delay_10us(1);			
}

/*******************************************************************************
* 函 数 名         : iic_ack
* 函数功能		   : 产生ACK应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void iic_ack(void)
{
	IIC_SCL=0;
	IIC_SDA=0;	//SDA为低电平
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;
}

/*******************************************************************************
* 函 数 名         : iic_nack
* 函数功能		   : 产生NACK非应答  
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void iic_nack(void)
{
	IIC_SCL=0;
	IIC_SDA=1;	//SDA为高电平
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;	
}

/*******************************************************************************
* 函 数 名         : iic_wait_ack
* 函数功能		   : 等待应答信号到来   
* 输    入         : 无
* 输    出         : 1，接收应答失败
        			 0，接收应答成功
*******************************************************************************/
u8 iic_wait_ack(void)
{
	u8 time_temp=0;
	
	IIC_SCL=1;
	delay_10us(1);
	while(IIC_SDA)	//等待SDA为低电平
	{
		time_temp++;
		if(time_temp>100)//超时则强制结束IIC通信
		{	
			iic_stop();
			return 1;	
		}			
	}
	IIC_SCL=0;
	return 0;	
}

/*******************************************************************************
* 函 数 名         : iic_write_byte
* 函数功能		   : IIC发送一个字节 
* 输    入         : dat：发送一个字节
* 输    出         : 无
*******************************************************************************/
void iic_write_byte(u8 dat)
{                        
    u8 i=0; 
	   	    
    IIC_SCL=0;
    for(i=0;i<8;i++)	//循环8次将一个字节传出，先传高再传低位
    {              
        if((dat&0x80)>0) 
			IIC_SDA=1;
		else
			IIC_SDA=0;
        dat<<=1; 	  
		delay_10us(1);  
		IIC_SCL=1;
		delay_10us(1); 
		IIC_SCL=0;	
		delay_10us(1);
    }	 
}

/*******************************************************************************
* 函 数 名         : iic_read_byte
* 函数功能		   : IIC读一个字节 
* 输    入         : ack=1时，发送ACK，ack=0，发送nACK 
* 输    出         : 应答或非应答
*******************************************************************************/
u8 iic_read_byte(u8 ack)
{
	u8 i=0,receive=0;
   	
    for(i=0;i<8;i++ )	//循环8次将一个字节读出，先读高再传低位
	{
        IIC_SCL=0; 
        delay_10us(1);
		IIC_SCL=1;
        receive<<=1;
        if(IIC_SDA)receive++;   
		delay_10us(1); 
    }					 
    if (!ack)
        iic_nack();
    else
        iic_ack();  
		  
    return receive;
}

/*******************************************************************************
* 函 数 名         : at24c02_write_one_byte
* 函数功能		   : 在AT24CXX指定地址写入一个数据
* 输    入         : addr:写入数据的目的地址 
					 dat:要写入的数据
* 输    出         : 无
*******************************************************************************/
void at24c02_write_one_byte(u8 addr,u8 dat)
{				   	  	    																 
    iic_start();  
	iic_write_byte(0XA0);	//发送写命令	    	  
	iic_wait_ack();	   
    iic_write_byte(addr);	//发送写地址   
	iic_wait_ack(); 	 										  		   
	iic_write_byte(dat);	//发送字节    							   
	iic_wait_ack();  		    	   
    iic_stop();				//产生一个停止条件
	delay_ms(10);	 
}

/*******************************************************************************
* 函 数 名         : at24c02_read_one_byte
* 函数功能		   : 在AT24CXX指定地址读出一个数据
* 输    入         : addr:开始读数的地址 
* 输    出         : 读到的数据
*******************************************************************************/
u8 at24c02_read_one_byte(u8 addr)
{				  
	u8 temp=0;		  	    																 
    iic_start();  
	iic_write_byte(0XA0);	//发送写命令	   
	iic_wait_ack(); 
    iic_write_byte(addr); 	//发送写地址  
	iic_wait_ack();	    
	iic_start();  	 	   
	iic_write_byte(0XA1); 	//进入接收模式         			   
	iic_wait_ack();	 
    temp=iic_read_byte(0);	//读取字节		   
    iic_stop();				//产生一个停止条件    
	return temp;			//返回读取的数据
}

/*******************************************************************************
* 函 数 名       : xpt2046_wirte_data
* 函数功能		 : XPT2046写数据
* 输    入       : dat：写入的数据
* 输    出    	 : 无
*******************************************************************************/
void xpt2046_wirte_data(u8 dat)
{
	u8 i;

	CLK = 0;
	_nop_();
	for(i=0;i<8;i++)//循环8次，每次传输一位，共一个字节
	{
		DIN = dat >> 7;//先传高位再传低位
		dat <<= 1;//将低位移到高位
		CLK = 0;//CLK由低到高产生一个上升沿，从而写入数据
		_nop_();	
		CLK = 1;
		_nop_();
	}
}

/*******************************************************************************
* 函 数 名       : xpt2046_read_data
* 函数功能		 : XPT2046读数据
* 输    入       : 无
* 输    出    	 : XPT2046返回12位数据
*******************************************************************************/
u16	xpt2046_read_data(void)
{
	u8 i;
	u16 dat=0;

	CLK = 0;
	_nop_();
	for(i=0;i<12;i++)//循环12次，每次读取一位，大于一个字节数，所以返回值类型是u16
	{
		dat <<= 1;
		CLK = 1;
		_nop_();
		CLK = 0; //CLK由高到低产生一个下降沿，从而读取数据
		_nop_();
		dat |= DOUT;//先读取高位，再读取低位。	
	}
	return dat;	
}

/*******************************************************************************
* 函 数 名       : xpt2046_read_adc_value
* 函数功能		 : XPT2046读AD数据
* 输    入       : cmd：指令
* 输    出    	 : XPT2046返回AD值
*******************************************************************************/
u16 xpt2046_read_adc_value(u8 cmd)
{
	u8 i;
	u16 adc_value=0;

	CLK = 0;//先拉低时钟
	CS  = 0;//使能XPT2046
	xpt2046_wirte_data(cmd);//发送命令字
	for(i=6; i>0; i--);//延时等待转换结果
	CLK = 1;
	_nop_();
	CLK = 0;//发送一个时钟，清除BUSY
	_nop_();
	adc_value=xpt2046_read_data();
	CS = 1;//关闭XPT2046
	return adc_value;
}

/*******************************************************************************
* 函 数 名       : Timer0Init
* 函数功能		 : 定时器0初始化
* 输    入       : 无
* 输    出    	 : 无
*******************************************************************************/
void Timer0Init(void)		
{
	TMOD |= 0x01;		//设置定时器模式
	TL0 = 0x9A;		//设置定时初始值
	TH0 = 0xA9;		//设置定时初始值
	TF0 = 0;		//清除TF0标志
	ET0=1;			//打开中断
	TR0 = 1;		//定时器0开始计时
	EA=1;			//打开总中断
}

/*******************************************************************************
* 函 数 名       : Timer1Init
* 函数功能		 : 定时器1初始化
* 输    入       : 无
* 输    出    	 : 无
*******************************************************************************/
void Timer1Init(void)		
{

	TMOD |= 0x10;		//设置定时器模式
	TL1 = 0x91;		//设置定时初始值
	TH1 = 0xFf;		//设置定时初始值
	TF1 = 0;		//清除TF1标志
	TR1 = 1;		//定时器1开始计时
	ET1=1;    		//打开中断
	EA=1;			//打开总中断
}

//****************************************主函数区*************************************************************
void main()
{
	DC_Motor=0;    //上电，使电机处于停转状态
	wendu=at24c02_read_one_byte(EEPROM_ADDRESS);  //从eeprom中读取数据
	Timer0Init();   //定时器0初始化
	Timer1Init();   //定时器1初始化
	ired_init();//红外初始化
	ds18b20_init();//初始化DS18B20
	while(1)		
	{
		if(wenkongmoshi==1&&kaiji_tag==1){LED1=0;LED2=1;LED3=1;}
		if(adckongmoshi==1&&kaiji_tag==1){LED1=1;LED2=0;LED3=1;}
		if(shoukongmoshi==1&&kaiji_tag==1){LED1=1;LED2=1;LED3=0;}
		switch(mode)
        {
            case 0:
            {
                if(one_tag!=0){one_tag=0;seg[0]=28;
				seg[1]=28;
				seg[2]=28;
				seg[3]=28;
				seg[4]=28;
				seg[5]=28;
				seg[6]=28;
				seg[7]=28;}

            }
            break;
            case 1:
            {
                if(one_tag!=1){one_tag=1;}
				seg[0]=27;
				seg[1]=duty/100%10;
				seg[2]=duty/10%10;
				seg[3]=duty/1%10;
				seg[4]=wendu_fuhao;
				seg[5]=temp_value/100%10;
				seg[6]=temp_value/10%10+16;
				seg[7]=temp_value/1%10;
				//seg[5]=gsmg_code[gired_data[2]/16];
				//seg[6]=gsmg_code[gired_data[2]%16];
//                seg[0]=ds18b20/100000;
//                seg[1]=ds18b20/10000%10+16;
//                seg[2]=ds18b20/1000%10;
//                seg[3]=ds18b20/100%10;
//                seg[4]=ds18b20/10%10;
//                seg[5]=ds18b20%10;
            }
            break;
            case 2:
            {
                if(one_tag!=2){one_tag=2;}
				seg[0]=26;
				seg[1]=28;
				seg[2]=28;
				seg[3]=28;
				seg[4]=28;
				seg[5]=28;
				seg[6]=wendu/10;
				seg[7]=wendu%10;
            }
            break;
			case 3:
            {
                if(one_tag!=2){one_tag=2;seg[6]=28;}
				if(wenkongmoshi==1)seg[0]=1;else seg[0]=28;		//第一位数码管显示1表示温控模式
				if(adckongmoshi==1)seg[1]=1;else seg[1]=28;		//第二位数码管显示1表示adc模式
				if(shoukongmoshi==1)seg[2]=1;else seg[2]=28;	//第三位数码管显示1表示手控模式
				if(adckongmoshi==1&&dianzu==1)seg[3]=1;else seg[3]=28;  //第四位数码管显示1表示adc的电阻模式
				if(adckongmoshi==1&&guangming==1)seg[4]=1;else seg[4]=28;  //第五位数码管显示1表示adc的光敏模式
				if(adckongmoshi==1&&reming==1)seg[5]=1;else seg[5]=28;		//第六位数码管显示1表示adc的热敏模式
				if(suoanjian==1)seg[7]=1;else seg[7]=28;   //第八位数码管显示1表示主机按键不能使用。

            }
            break;
        }
		
		if((int)(temp_value/10)>(wendu+1)&&wenkongmoshi==1)
		{
			duty=0;
		}
		else if((int)(temp_value/10)<(wendu-1)&&wenkongmoshi==1)
		{
			duty=60;
		}
		else if((int)(temp_value/10)<(wendu+1)&&(int)(temp_value/10)>(wendu-1)&&wenkongmoshi==1)
		{
			duty=30;
		}
		if(adckongmoshi==1&&dianzu==1)
		{
			if(adc_value<50)duty=0;
			else if(adc_value>4050)duty=100;
			else duty=adc_value/100;
		}
		if(adckongmoshi==1&&guangming==1)
		{
			if(adc_value<100)duty=100; //黑，晚上不转
			else if(adc_value>1000)duty=0;
			else duty=100-(adc_value/10);
		}
		if(adckongmoshi==1&&reming==1)
		{
			if((adc_value-1500)<100)duty=100; //温度低，不转
			else if((adc_value-1500)>500)duty=0;
			else duty=60-((adc_value-1600)/7);
		}
		if(gired_data[2]==0x45)
		{
				kaiji_tag=!kaiji_tag;
				if(kaiji_tag==0)
				{
					dianjiqidong_tag=0;
					mode=0;
					DC_Motor=0;
					P2=0xff;
					//duty=100;
				}
				else 
				{
					mode=1;
					//DC_Motor=0;
					//dianjiqidong_tag=0;
				}
		}
		if(gired_data[2]==0x44&&kaiji_tag==1){dianjiqidong_tag=!dianjiqidong_tag;DC_Motor=0;}
		if(gired_data[2]==0x07&&kaiji_tag==1&&mode==2)
			{
				at24c02_write_one_byte(EEPROM_ADDRESS,wendu);
			}
			if(gired_data[2]==0x46&&kaiji_tag==1)
			{
				if(wenkongmoshi==1){wenkongmoshi=0;adckongmoshi=1;shoukongmoshi=0;}
				else if(adckongmoshi==1){wenkongmoshi=0;adckongmoshi=0;shoukongmoshi=1;}
				else if(shoukongmoshi==1){wenkongmoshi=1;adckongmoshi=0;shoukongmoshi=0;}
			}
			if(gired_data[2]==0x47&&kaiji_tag==1){suoanjian=!suoanjian;}
			if(gired_data[2]==0x40&&kaiji_tag==1){wendu--;if(wendu<=14)wendu=15;}
			if(gired_data[2]==0x43&&kaiji_tag==1){wendu++;if(wendu>=36)wendu=35;}
		if(gired_data[2]==0x15&&kaiji_tag==1&&adckongmoshi==1)
		{
			if(guangming==1){guangming=0;reming=1;dianzu=0;}
			else if(reming==1){guangming=0;reming=0;dianzu=1;}
			else if(dianzu==1){guangming=1;reming=0;dianzu=0;}
		}
		if(gired_data[2]==0x09&&kaiji_tag==1)
		{
			if(guangming==1){guangming=0;reming=0;dianzu=1;}
			else if(reming==1){guangming=1;reming=0;dianzu=0;}
			else if(dianzu==1){guangming=0;reming=1;dianzu=0;}
		}
		if(gired_data[2]==0x19&&kaiji_tag==1)
		{	
			if(mode==1)mode=3;
			else if(mode==3)mode=1;
			else if(mode==2)mode=1;
		}
		if(gired_data[2]==0x0d&&kaiji_tag==1){mode=2;}
		if(kaiji_tag==1)
		{
		if(gired_data[2]==0x16)duty=0;
		if(gired_data[2]==0x0c)duty=6;
		if(gired_data[2]==0x18)duty=12;
		if(gired_data[2]==0x5e)duty=18;
		if(gired_data[2]==0x08)duty=24;
		if(gired_data[2]==0x1c)duty=30;
		if(gired_data[2]==0x5a)duty=36;
		if(gired_data[2]==0x42)duty=42;
		if(gired_data[2]==0x52)duty=48;
		if(gired_data[2]==0x4a)duty=54;
		}
		gired_data[2]=0;
//*******************main按键控制区**********
		if(key_down)
        { 
			if(suoanjian==0)
			{
			if(key_down==1&&kaiji_tag==1){wendu--;if(wendu<=14)wendu=15;}
			if(key_down==2&&kaiji_tag==1)
			{
				mode=2;
			}
			if(key_down==3&&kaiji_tag==1){wendu++;if(wendu>=36)wendu=35;}
//			if(key_down==4){duty=0;}
			
			if(key_down==5&&adckongmoshi==1){guangming=1;reming=0;dianzu=0;}
			if(key_down==6&&kaiji_tag==1)
			{
				if(mode==1)mode=3;
				else if(mode==3)mode=1;
				else if(mode==2)mode=1;
			}
			if(key_down==7&&adckongmoshi==1){guangming=0;reming=1;dianzu=0;}
//			if(key_down==8){duty=0;}
			
			if(key_down==9)
			{
				wenkongmoshi=1;
				adckongmoshi=0;
				shoukongmoshi=0;
			}//温控模式
			if(key_down==10)
			{
				wenkongmoshi=0;
				adckongmoshi=1;
				shoukongmoshi=0;
				guangming=0;reming=0;dianzu=1;
			}//adc模式
			if(key_down==11)
			{
				wenkongmoshi=0;
				adckongmoshi=0;
				shoukongmoshi=1;

			}//手控模式
//			if(key_down==12){duty=0;}
			
			if(key_down==13&&shoukongmoshi==1){duty=0;}
			if(key_down==14&&shoukongmoshi==1){duty=30;}
			if(key_down==15&&shoukongmoshi==1){duty=60;}
//			if(key_down==16){duty=100;}
			}
			if(key_down==17)
			{
				kaiji_tag=!kaiji_tag;
				if(kaiji_tag==0)
				{
					dianjiqidong_tag=0;
					mode=0;
					DC_Motor=0;
					P2=0xff;
					//duty=100;
				}
				else 
				{
					mode=1;
					//DC_Motor=0;
					//dianjiqidong_tag=0;
				}
			}
			if(key_down==18&&kaiji_tag==1&&mode==2)
			{
				at24c02_write_one_byte(EEPROM_ADDRESS,wendu);
			}
			//if(key_down==19){duty=40;}  //被红外占用P32io口
			if(key_down==20&&kaiji_tag==1){dianjiqidong_tag=!dianjiqidong_tag;DC_Motor=0;}

          key_down =0;
        }
//**********************按键控制区结束********************

	}		
}

//****************************************温度按键扫描区*************************************************************

void tim0(void) interrupt 1
{
	TL0 = 0x9A;		//设置定时初始值
	TH0 = 0xA9;		//设置定时初始值
	sys++;
	if(++key_sanf_delay == 10)key_sanf_delay=0;//10ms one keyboard scan
	if(adckongmoshi==1&&sys%5==0)
	{
		if(dianzu==1)adc_value=xpt2046_read_adc_value(0x94);//测量电位器
		if(reming==1)adc_value=xpt2046_read_adc_value(0xD4);//测量热敏电阻
		if(guangming==1)adc_value=xpt2046_read_adc_value(0xA4);//测量光敏电阻
		//sys=0;
	}
	if(sys%30==0)//间隔一段时间读取温度值，间隔时间要大于温度传感器转换温度时间
	{
		temp_value=ds18b20_read_temperture()*10;//保留温度值小数后一位
		if(temp_value<0)//负温度
		{
			temp_value=-temp_value;
			wendu_fuhao=29;//显示负号	
		}
		else 
		{
			wendu_fuhao=28;//不显示
		}
		
	}
	
    if(key_sanf_delay==0)
    {
        key_sanf_delay=1;
        key_val=Key_Read();
        key_down = key_val & (key_old ^ key_val);
        key_up = ~key_val & (key_old ^ key_val);
        key_old = key_val;
    }
}
//****************************************pwm波，seg扫描区*************************************************************

void time1() interrupt 3 
{
	static u16 i;
	static u16 j;
	TL1 = 0x91;		//设置定时初始值
	TH1 = 0xFf;		//设置定时初始值
	i++;
	if(i>=scale){i=0;}
	if(kaiji_tag==1)
	{
		j++;
		if(j>=4){j=0;seg_tag++;}
		//SMG_A_DP_PORT=0XFF;
		switch(seg_tag%8)
		{
			case 0: LSC=1;LSB=1;LSA=1;break;
			case 1: LSC=1;LSB=1;LSA=0;break;
			case 2: LSC=1;LSB=0;LSA=1;break;
			case 3: LSC=1;LSB=0;LSA=0;break;
			case 4: LSC=0;LSB=1;LSA=1;break;
			case 5: LSC=0;LSB=1;LSA=0;break;
			case 6: LSC=0;LSB=0;LSA=1;break;
			case 7: LSC=0;LSB=0;LSA=0;break;
		}
	}
		SMG_A_DP_PORT=gsmg_code[seg[seg_tag%8]];
	if(dianjiqidong_tag==1)
	{
		if(i>duty)
		{
			DC_Motor=1;
		}
		else
		{
			DC_Motor=0;
			
		}
	}
}

