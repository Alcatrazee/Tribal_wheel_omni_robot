#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h" 
	   		   
#define true 1
#define false 0 
#define bool  uint8_t
#define TRUE  0
#define FALSE -1
//0��ʾд
#define	I2C_Transmitter   0
//����ʾ��
#define	I2C_Receiver      1							 						 
//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB9����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB9���ģʽ
//IO��������	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
uint8_t i2cread(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t i2cwrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data);
void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
bool i2cReadBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif















