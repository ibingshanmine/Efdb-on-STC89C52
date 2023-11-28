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

#define scale	100   //pwm������
#define SMG_A_DP_PORT	P0  //����ܶ�ѡ����
#define EEPROM_ADDRESS	0x00	//�������ݴ���EEPROM����ʼ��ַ

typedef unsigned int u16;	//��ϵͳĬ���������ͽ����ض���
typedef unsigned char u8;	//��ϵͳĬ���������ͽ����ض���

//�ܽŶ���
sbit DC_Motor=P1^0;//����ֱ��������ƹܽ�
sbit KEY1=P3^1;
sbit KEY2=P3^0;
//sbit KEY3=P3^2;   //������ռ��P32io��
sbit KEY4=P3^3;
sbit LED1=P2^5;
sbit LED2=P2^6;
sbit LED3=P2^7;
sbit LSA=P2^2;  //�����λѡ���Ŷ���
sbit LSB=P2^3;	//�����λѡ���Ŷ���
sbit LSC=P2^4;	//�����λѡ���Ŷ���
sbit IRED=P3^2; //�������Ŷ���
sbit DS18B20_PORT=P3^7;		//DS18B20���ݿڶ���
//����EEPROM���ƽ�
sbit IIC_SCL=P2^1;//SCLʱ����
sbit IIC_SDA=P2^0;//SDA������
sbit DOUT = P3^7;	  //���
sbit CLK  = P3^6;	  //ʱ��
sbit DIN  = P3^4;	  //����
sbit CS   = P3^5;	  //Ƭѡ
//�ֿ�
u8 gsmg_code[30]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71, 
                  0xbf,0x86,0xdb,0xcf,0xe6,0xed,0xfd,0x87,0xff,0xef,
					0X76,0x73,0x00,0x40}; //H,P,buxianshi,- 
//����
unsigned long int sys=0;  //��ʱ��1ϵͳʱ��
//unsigned int system=0;    
seg[8]={28,28,28,28,28,28,28,28}; //�������������
unsigned char key;       //��������
unsigned char key_sanf_delay,key_old;  //��������
unsigned char key_val,key_down,key_up;  //��������
unsigned long int seg_tag=0;     //�����ˢ������
//u8 dir=0;     
u16 duty=100;  //ռ�ձȣ��ϵ���ȳ�ʼ״̬
u8 gired_data[4];//�洢4���ֽڽ����루��ַ��+��ַ����+������+���Ʒ��룩
//#define DC_MOTOR_RUN_TIME	5000	//����ֱ���������ʱ��Ϊ5000ms
u8 mode=0;  //����ܽ����л�
u8 one_tag=0; //�����л���Ҫ
u8 wendu_fuhao;  //�¶�������־
//u8 save_value=0;  //
u8 wendu=20;  //�¿�ģʽ�¶��趨ֵ
u16 adc_value=0;  //adcģʽ����ȡԭʼ����
float adc_vol;//ADC��ѹֵ
int temp_value; //ת������¶Ⱦ���ֵ 
//��־λ
bit kaiji_tag=0; //������־
bit dianjiqidong_tag=0;  //���������־
bit wenkongmoshi=0;  //�¿�ģʽ��־
bit shoukongmoshi=1;  //�ֿ�ģʽ��־
bit adckongmoshi=0;  //adc����ģʽ��־
bit guangming=0;  //adc�Ĺ���ģʽ��־
bit reming=0;    //adc������ģʽ��־
bit dianzu=1;    //adc�ĵ���ģʽ��־
bit suoanjian=0;  //������������־
//****************************************�ײ������*************************************************************
/*******************************************************************************
* �� �� ��       : delay_ms
* ��������		 : ms��ʱ������ms=1ʱ����Լ��ʱ1ms
* ��    ��       : ms��ms��ʱʱ��
* ��    ��    	 : ��
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
* �� �� ��       : Key_Read
* ��������		 : ���󰴼�ɨ�躯��
* ��    ��       : ��
* ��    ��    	 : ��ֵ
*******************************************************************************/
unsigned char Key_Read(void)
{
	if(!KEY1)return 17;
	else if(!KEY2)return 18;
	//else if(!KEY3)return 19;  //������ռ��P32io��
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
* �� �� ��         : ds18b20_reset
* ��������		   : ��λDS18B20  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ds18b20_reset(void)
{
	DS18B20_PORT=0;	//����DQ
	delay_10us(75);	//����750us
	DS18B20_PORT=1;	//DQ=1
	delay_10us(2);	//20US
}

/*******************************************************************************
* �� �� ��         : ds18b20_check
* ��������		   : ���DS18B20�Ƿ����
* ��    ��         : ��
* ��    ��         : 1:δ��⵽DS18B20�Ĵ��ڣ�0:����
*******************************************************************************/
u8 ds18b20_check(void)
{
	u8 time_temp=0;

	while(DS18B20_PORT&&time_temp<20)	//�ȴ�DQΪ�͵�ƽ
	{
		time_temp++;
		delay_10us(1);	
	}
	if(time_temp>=20)return 1;	//�����ʱ��ǿ�Ʒ���1
	else time_temp=0;
	while((!DS18B20_PORT)&&time_temp<20)	//�ȴ�DQΪ�ߵ�ƽ
	{
		time_temp++;
		delay_10us(1);
	}
	if(time_temp>=20)return 1;	//�����ʱ��ǿ�Ʒ���1
	return 0;
}

/*******************************************************************************
* �� �� ��         : ds18b20_read_bit
* ��������		   : ��DS18B20��ȡһ��λ
* ��    ��         : ��
* ��    ��         : 1/0
*******************************************************************************/
u8 ds18b20_read_bit(void)
{
	u8 dat=0;
	
	DS18B20_PORT=0;
	_nop_();_nop_();
	DS18B20_PORT=1;	
	_nop_();_nop_(); //�ö�ʱ�䲻�ܹ�����������15us�ڶ�ȡ����
	if(DS18B20_PORT)dat=1;	//���������Ϊ1������datΪ1������Ϊ0
	else dat=0;
	delay_10us(5);
	return dat;
} 

/*******************************************************************************
* �� �� ��         : ds18b20_read_byte
* ��������		   : ��DS18B20��ȡһ���ֽ�
* ��    ��         : ��
* ��    ��         : һ���ֽ�����
*******************************************************************************/
u8 ds18b20_read_byte(void)
{
	u8 i=0;
	u8 dat=0;
	u8 temp=0;

	for(i=0;i<8;i++)//ѭ��8�Σ�ÿ�ζ�ȡһλ�����ȶ���λ�ٶ���λ
	{
		temp=ds18b20_read_bit();
		dat=(temp<<7)|(dat>>1);
	}
	return dat;	
}

/*******************************************************************************
* �� �� ��         : ds18b20_write_byte
* ��������		   : дһ���ֽڵ�DS18B20
* ��    ��         : dat��Ҫд����ֽ�
* ��    ��         : ��
*******************************************************************************/
void ds18b20_write_byte(u8 dat)
{
	u8 i=0;
	u8 temp=0;

	for(i=0;i<8;i++)//ѭ��8�Σ�ÿ��дһλ������д��λ��д��λ
	{
		temp=dat&0x01;//ѡ���λ׼��д��
		dat>>=1;//���θ�λ�Ƶ���λ
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
* �� �� ��         : ds18b20_start
* ��������		   : ��ʼ�¶�ת��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void ds18b20_start(void)
{
	ds18b20_reset();//��λ
	ds18b20_check();//���DS18B20
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0x44);//ת������	
}

/*******************************************************************************
* �� �� ��         : ds18b20_init
* ��������		   : ��ʼ��DS18B20��IO�� DQ ͬʱ���DS�Ĵ���
* ��    ��         : ��
* ��    ��         : 1:�����ڣ�0:����
*******************************************************************************/ 
u8 ds18b20_init(void)
{
	ds18b20_reset();
	return ds18b20_check();	
}

/*******************************************************************************
* �� �� ��         : ds18b20_read_temperture
* ��������		   : ��ds18b20�õ��¶�ֵ
* ��    ��         : ��
* ��    ��         : �¶�����
*******************************************************************************/
float ds18b20_read_temperture(void)
{
	float temp;
	u8 dath=0;
	u8 datl=0;
	u16 value=0;

	ds18b20_start();//��ʼת��
	ds18b20_reset();//��λ
	ds18b20_check();
	ds18b20_write_byte(0xcc);//SKIP ROM
    ds18b20_write_byte(0xbe);//���洢��

	datl=ds18b20_read_byte();//���ֽ�
	dath=ds18b20_read_byte();//���ֽ�
	value=(dath<<8)+datl;//�ϲ�Ϊ16λ����

	if((value&0xf800)==0xf800)//�жϷ���λ�����¶�
	{
		value=(~value)+1; //����ȡ���ټ�1
		temp=value*(-0.0625);//���Ծ���	
	}
	else //���¶�
	{
		temp=value*0.0625;	
	}
	return temp;
}

/*******************************************************************************
* �� �� ��       : ired_init
* ��������		 : ������ܳ�ʼ������
* ��    ��       : ��
* ��    ��    	 : ��
*******************************************************************************/
void ired_init(void)
{
	IT0=1;	//�½��ش���
	EX0=1;	//���ж�0����
	EA=1;	//�����ж�
	IRED=1;	//��ʼ���˿�
}
/*******************************************************************************
* �� �� ��       : ired
* ��������		 : ��������жϺ������ⲿ�ж�0����
* ��    ��       : ��
* ��    ��    	 : ��
*******************************************************************************/
void ired() interrupt 0	//�ⲿ�ж�0������
{
	u8 ired_high_time=0;
	u16 time_cnt=0;
	u8 i=0,j=0;

	if(IRED==0)
	{
		time_cnt=1000;
		while((!IRED)&&(time_cnt))//�ȴ������ź�9ms�͵�ƽ������������10msǿ���˳�
		{
			delay_10us(1);//��ʱԼ10us
			time_cnt--;
			if(time_cnt==0)return;		
		}
		if(IRED)//�����ź�9ms�͵�ƽ�ѹ�������4.5ms�ߵ�ƽ
		{
			time_cnt=500;
			while(IRED&&time_cnt)//�ȴ������ź�4.5ms�ߵ�ƽ������������5msǿ���˳�
			{
				delay_10us(1);
				time_cnt--;
				if(time_cnt==0)return;	
			}
			for(i=0;i<4;i++)//ѭ��4�Σ���ȡ4���ֽ�����
			{
				for(j=0;j<8;j++)//ѭ��8�ζ�ȡÿλ���ݼ�һ���ֽ�
				{
					time_cnt=600;
					while((IRED==0)&&time_cnt)//�ȴ�����1��0ǰ���0.56ms������������6msǿ���˳�
					{
						delay_10us(1);
						time_cnt--;
						if(time_cnt==0)return;	
					}
					time_cnt=20;
					while(IRED)//�ȴ�����1��0����ĸߵ�ƽ������������2msǿ���˳�
					{
						delay_10us(10);//Լ0.1ms
						ired_high_time++;
						if(ired_high_time>20)return;	
					}
					gired_data[i]>>=1;//�ȶ�ȡ��Ϊ��λ��Ȼ���Ǹ�λ
					if(ired_high_time>=8)//����ߵ�ƽʱ�����0.8ms��������Ϊ1������Ϊ0
						gired_data[i]|=0x80;
					ired_high_time=0;//�������㣬�ȴ���һ�μ���ʱ��
				}
			}
		}
		if(gired_data[2]!=~gired_data[3])//У��������뷴�룬�����򷵻�
		{
			for(i=0;i<4;i++)
				gired_data[i]=0;
			return;	
		}
	}		
}

/*******************************************************************************
* �� �� ��       : iic_start
* ��������		 : ����IIC��ʼ�ź�
* ��    ��       : ��
* ��    ��    	 : ��
*******************************************************************************/
void iic_start(void)
{
	IIC_SDA=1;//����Ѹ���������SCL���棬�ڶ��ζ�д���������
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=0;	//��SCLΪ�ߵ�ƽʱ��SDA�ɸ߱�Ϊ��
	delay_10us(1);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ��������
	delay_10us(1);
}

/*******************************************************************************
* �� �� ��         : iic_stop
* ��������		   : ����IICֹͣ�ź�   
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void iic_stop(void)
{	
	IIC_SDA=0;//����Ѹ���������SCL���棬�ڶ��ζ�д���������
	delay_10us(1);
	IIC_SCL=1;
	delay_10us(1);
	IIC_SDA=1;	//��SCLΪ�ߵ�ƽʱ��SDA�ɵͱ�Ϊ��
	delay_10us(1);			
}

/*******************************************************************************
* �� �� ��         : iic_ack
* ��������		   : ����ACKӦ��  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void iic_ack(void)
{
	IIC_SCL=0;
	IIC_SDA=0;	//SDAΪ�͵�ƽ
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;
}

/*******************************************************************************
* �� �� ��         : iic_nack
* ��������		   : ����NACK��Ӧ��  
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void iic_nack(void)
{
	IIC_SCL=0;
	IIC_SDA=1;	//SDAΪ�ߵ�ƽ
	delay_10us(1);
   	IIC_SCL=1;
	delay_10us(1);
	IIC_SCL=0;	
}

/*******************************************************************************
* �� �� ��         : iic_wait_ack
* ��������		   : �ȴ�Ӧ���źŵ���   
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
        			 0������Ӧ��ɹ�
*******************************************************************************/
u8 iic_wait_ack(void)
{
	u8 time_temp=0;
	
	IIC_SCL=1;
	delay_10us(1);
	while(IIC_SDA)	//�ȴ�SDAΪ�͵�ƽ
	{
		time_temp++;
		if(time_temp>100)//��ʱ��ǿ�ƽ���IICͨ��
		{	
			iic_stop();
			return 1;	
		}			
	}
	IIC_SCL=0;
	return 0;	
}

/*******************************************************************************
* �� �� ��         : iic_write_byte
* ��������		   : IIC����һ���ֽ� 
* ��    ��         : dat������һ���ֽ�
* ��    ��         : ��
*******************************************************************************/
void iic_write_byte(u8 dat)
{                        
    u8 i=0; 
	   	    
    IIC_SCL=0;
    for(i=0;i<8;i++)	//ѭ��8�ν�һ���ֽڴ������ȴ����ٴ���λ
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
* �� �� ��         : iic_read_byte
* ��������		   : IIC��һ���ֽ� 
* ��    ��         : ack=1ʱ������ACK��ack=0������nACK 
* ��    ��         : Ӧ����Ӧ��
*******************************************************************************/
u8 iic_read_byte(u8 ack)
{
	u8 i=0,receive=0;
   	
    for(i=0;i<8;i++ )	//ѭ��8�ν�һ���ֽڶ������ȶ����ٴ���λ
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
* �� �� ��         : at24c02_write_one_byte
* ��������		   : ��AT24CXXָ����ַд��һ������
* ��    ��         : addr:д�����ݵ�Ŀ�ĵ�ַ 
					 dat:Ҫд�������
* ��    ��         : ��
*******************************************************************************/
void at24c02_write_one_byte(u8 addr,u8 dat)
{				   	  	    																 
    iic_start();  
	iic_write_byte(0XA0);	//����д����	    	  
	iic_wait_ack();	   
    iic_write_byte(addr);	//����д��ַ   
	iic_wait_ack(); 	 										  		   
	iic_write_byte(dat);	//�����ֽ�    							   
	iic_wait_ack();  		    	   
    iic_stop();				//����һ��ֹͣ����
	delay_ms(10);	 
}

/*******************************************************************************
* �� �� ��         : at24c02_read_one_byte
* ��������		   : ��AT24CXXָ����ַ����һ������
* ��    ��         : addr:��ʼ�����ĵ�ַ 
* ��    ��         : ����������
*******************************************************************************/
u8 at24c02_read_one_byte(u8 addr)
{				  
	u8 temp=0;		  	    																 
    iic_start();  
	iic_write_byte(0XA0);	//����д����	   
	iic_wait_ack(); 
    iic_write_byte(addr); 	//����д��ַ  
	iic_wait_ack();	    
	iic_start();  	 	   
	iic_write_byte(0XA1); 	//�������ģʽ         			   
	iic_wait_ack();	 
    temp=iic_read_byte(0);	//��ȡ�ֽ�		   
    iic_stop();				//����һ��ֹͣ����    
	return temp;			//���ض�ȡ������
}

/*******************************************************************************
* �� �� ��       : xpt2046_wirte_data
* ��������		 : XPT2046д����
* ��    ��       : dat��д�������
* ��    ��    	 : ��
*******************************************************************************/
void xpt2046_wirte_data(u8 dat)
{
	u8 i;

	CLK = 0;
	_nop_();
	for(i=0;i<8;i++)//ѭ��8�Σ�ÿ�δ���һλ����һ���ֽ�
	{
		DIN = dat >> 7;//�ȴ���λ�ٴ���λ
		dat <<= 1;//����λ�Ƶ���λ
		CLK = 0;//CLK�ɵ͵��߲���һ�������أ��Ӷ�д������
		_nop_();	
		CLK = 1;
		_nop_();
	}
}

/*******************************************************************************
* �� �� ��       : xpt2046_read_data
* ��������		 : XPT2046������
* ��    ��       : ��
* ��    ��    	 : XPT2046����12λ����
*******************************************************************************/
u16	xpt2046_read_data(void)
{
	u8 i;
	u16 dat=0;

	CLK = 0;
	_nop_();
	for(i=0;i<12;i++)//ѭ��12�Σ�ÿ�ζ�ȡһλ������һ���ֽ��������Է���ֵ������u16
	{
		dat <<= 1;
		CLK = 1;
		_nop_();
		CLK = 0; //CLK�ɸߵ��Ͳ���һ���½��أ��Ӷ���ȡ����
		_nop_();
		dat |= DOUT;//�ȶ�ȡ��λ���ٶ�ȡ��λ��	
	}
	return dat;	
}

/*******************************************************************************
* �� �� ��       : xpt2046_read_adc_value
* ��������		 : XPT2046��AD����
* ��    ��       : cmd��ָ��
* ��    ��    	 : XPT2046����ADֵ
*******************************************************************************/
u16 xpt2046_read_adc_value(u8 cmd)
{
	u8 i;
	u16 adc_value=0;

	CLK = 0;//������ʱ��
	CS  = 0;//ʹ��XPT2046
	xpt2046_wirte_data(cmd);//����������
	for(i=6; i>0; i--);//��ʱ�ȴ�ת�����
	CLK = 1;
	_nop_();
	CLK = 0;//����һ��ʱ�ӣ����BUSY
	_nop_();
	adc_value=xpt2046_read_data();
	CS = 1;//�ر�XPT2046
	return adc_value;
}

/*******************************************************************************
* �� �� ��       : Timer0Init
* ��������		 : ��ʱ��0��ʼ��
* ��    ��       : ��
* ��    ��    	 : ��
*******************************************************************************/
void Timer0Init(void)		
{
	TMOD |= 0x01;		//���ö�ʱ��ģʽ
	TL0 = 0x9A;		//���ö�ʱ��ʼֵ
	TH0 = 0xA9;		//���ö�ʱ��ʼֵ
	TF0 = 0;		//���TF0��־
	ET0=1;			//���ж�
	TR0 = 1;		//��ʱ��0��ʼ��ʱ
	EA=1;			//�����ж�
}

/*******************************************************************************
* �� �� ��       : Timer1Init
* ��������		 : ��ʱ��1��ʼ��
* ��    ��       : ��
* ��    ��    	 : ��
*******************************************************************************/
void Timer1Init(void)		
{

	TMOD |= 0x10;		//���ö�ʱ��ģʽ
	TL1 = 0x91;		//���ö�ʱ��ʼֵ
	TH1 = 0xFf;		//���ö�ʱ��ʼֵ
	TF1 = 0;		//���TF1��־
	TR1 = 1;		//��ʱ��1��ʼ��ʱ
	ET1=1;    		//���ж�
	EA=1;			//�����ж�
}

//****************************************��������*************************************************************
void main()
{
	DC_Motor=0;    //�ϵ磬ʹ�������ͣת״̬
	wendu=at24c02_read_one_byte(EEPROM_ADDRESS);  //��eeprom�ж�ȡ����
	Timer0Init();   //��ʱ��0��ʼ��
	Timer1Init();   //��ʱ��1��ʼ��
	ired_init();//�����ʼ��
	ds18b20_init();//��ʼ��DS18B20
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
				if(wenkongmoshi==1)seg[0]=1;else seg[0]=28;		//��һλ�������ʾ1��ʾ�¿�ģʽ
				if(adckongmoshi==1)seg[1]=1;else seg[1]=28;		//�ڶ�λ�������ʾ1��ʾadcģʽ
				if(shoukongmoshi==1)seg[2]=1;else seg[2]=28;	//����λ�������ʾ1��ʾ�ֿ�ģʽ
				if(adckongmoshi==1&&dianzu==1)seg[3]=1;else seg[3]=28;  //����λ�������ʾ1��ʾadc�ĵ���ģʽ
				if(adckongmoshi==1&&guangming==1)seg[4]=1;else seg[4]=28;  //����λ�������ʾ1��ʾadc�Ĺ���ģʽ
				if(adckongmoshi==1&&reming==1)seg[5]=1;else seg[5]=28;		//����λ�������ʾ1��ʾadc������ģʽ
				if(suoanjian==1)seg[7]=1;else seg[7]=28;   //�ڰ�λ�������ʾ1��ʾ������������ʹ�á�

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
			if(adc_value<100)duty=100; //�ڣ����ϲ�ת
			else if(adc_value>1000)duty=0;
			else duty=100-(adc_value/10);
		}
		if(adckongmoshi==1&&reming==1)
		{
			if((adc_value-1500)<100)duty=100; //�¶ȵͣ���ת
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
//*******************main����������**********
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
			}//�¿�ģʽ
			if(key_down==10)
			{
				wenkongmoshi=0;
				adckongmoshi=1;
				shoukongmoshi=0;
				guangming=0;reming=0;dianzu=1;
			}//adcģʽ
			if(key_down==11)
			{
				wenkongmoshi=0;
				adckongmoshi=0;
				shoukongmoshi=1;

			}//�ֿ�ģʽ
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
			//if(key_down==19){duty=40;}  //������ռ��P32io��
			if(key_down==20&&kaiji_tag==1){dianjiqidong_tag=!dianjiqidong_tag;DC_Motor=0;}

          key_down =0;
        }
//**********************��������������********************

	}		
}

//****************************************�¶Ȱ���ɨ����*************************************************************

void tim0(void) interrupt 1
{
	TL0 = 0x9A;		//���ö�ʱ��ʼֵ
	TH0 = 0xA9;		//���ö�ʱ��ʼֵ
	sys++;
	if(++key_sanf_delay == 10)key_sanf_delay=0;//10ms one keyboard scan
	if(adckongmoshi==1&&sys%5==0)
	{
		if(dianzu==1)adc_value=xpt2046_read_adc_value(0x94);//������λ��
		if(reming==1)adc_value=xpt2046_read_adc_value(0xD4);//������������
		if(guangming==1)adc_value=xpt2046_read_adc_value(0xA4);//������������
		//sys=0;
	}
	if(sys%30==0)//���һ��ʱ���ȡ�¶�ֵ�����ʱ��Ҫ�����¶ȴ�����ת���¶�ʱ��
	{
		temp_value=ds18b20_read_temperture()*10;//�����¶�ֵС����һλ
		if(temp_value<0)//���¶�
		{
			temp_value=-temp_value;
			wendu_fuhao=29;//��ʾ����	
		}
		else 
		{
			wendu_fuhao=28;//����ʾ
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
//****************************************pwm����segɨ����*************************************************************

void time1() interrupt 3 
{
	static u16 i;
	static u16 j;
	TL1 = 0x91;		//���ö�ʱ��ʼֵ
	TH1 = 0xFf;		//���ö�ʱ��ʼֵ
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

