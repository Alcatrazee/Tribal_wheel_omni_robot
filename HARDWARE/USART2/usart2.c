#include "includes.h"	
#include "common_fcn.h"


#define data_num 11

u8 USART2_RX_DAT[data_num];
u8 Process_finish_flag=0;
u8 USART2_RX_COUNT;

//global variables
float Ax,Ay,Az;
float Pitch,Roll,Yaw;
float Temperature;



extern OS_FLAG_GRP usart2_flags_group;

void uart2_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIODʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD9����ΪUSART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD10����ΪUSART2
	
	//USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOD9��GPIOD10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOD,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 
	
//USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�

	//USART2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

	
}


void USART2_IRQHandler(void)                	//����1�жϷ������
{
	OSIntEnter();
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		if(Process_finish_flag){
			USART2_RX_DAT[USART2_RX_COUNT] = (USART2->DR);//USART_ReceiveData(USART2);//(USART1->DR);	//��ȡ���յ�������
			USART2_RX_COUNT++;
			if(USART2_RX_DAT[0]!=0x55)
				UART2_Clear();
			if(USART2_RX_COUNT==data_num){
				if(USART2_RX_DAT[0]==0x55){
					Process_finish_flag=0;
				}else {
					UART2_Clear();
				}
			}
		}else{
			USART2_RX_DAT[USART2_RX_COUNT] = (USART2->DR);
		}
  } 
	OSIntExit();
}

 void UART2_Put_Char(unsigned char DataToSend)
{
	while((USART2->SR&0x40)==0);  
    USART2->DR = (u8) DataToSend;  
}

void  UART2_Put2_Char(u8 arr[2]){
	UART2_Put_Char(arr[0]);
	UART2_Put_Char(arr[1]);
}

void UART2_Send_Str(u8 *str){
	while(*str!='\0'){
		UART2_Put_Char(*str);
		str++;
	}
}

void UART2_Send_IMUdat(u8 *dat){
	u8 i;
	for(i=0;i<11;i++){
		UART2_Put_Char(*dat);
		dat++;
	}
}

void UART2_Clear(void){
	u8 count;
	for(count=0;count<11;count++){
		USART2_RX_DAT[count] = 0;
	}
	USART2_RX_COUNT = 0;
}

extern RB_State State;
float continue_Yaw=0;

void Data_Process(void){
	switch(USART2_RX_DAT[1]){
		case 0x51: 
				Ax=(float)((short)(USART2_RX_DAT[3]<<8)|USART2_RX_DAT[2])/32768*4*9.8f;
				Ay=(float)((short)(USART2_RX_DAT[5]<<8)|USART2_RX_DAT[4])/32768*4*9.8f;		
				Az=(float)((short)(USART2_RX_DAT[7]<<8)|USART2_RX_DAT[6])/32768*4*9.8f;		
				Temperature = (float)((short)(USART2_RX_DAT[9]<<8)|USART2_RX_DAT[8])/340+21;
				break;
		case 0x53:
				Roll=	(float)((short)(USART2_RX_DAT[3]<<8)|USART2_RX_DAT[2])/32768*180;		
				Pitch=(float)((short)(USART2_RX_DAT[5]<<8)|USART2_RX_DAT[4])/32768*180;		
				Yaw=	(float)((short)(USART2_RX_DAT[7]<<8)|USART2_RX_DAT[6])/32768*180;		
				Temperature = (float)((short)(USART2_RX_DAT[9]<<8)|USART2_RX_DAT[8])/340+21;
				IMUdata_Serialization();
				State.angle = continue_Yaw;
				break;
	}
}



void IMUdata_Serialization(void){
	float gain=0;
	static float former_angle=0,c_angle=0;
	c_angle = Yaw;
	gain = c_angle-former_angle;
  if(my_abs(gain)>300){
		if(c_angle<0)
				gain = 360-my_abs(gain);
		else 
				gain = my_abs(gain)-360;
	}
	continue_Yaw = continue_Yaw + gain;
	former_angle = c_angle;
}
































