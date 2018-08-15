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
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //GPIOD9复用为USART2
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //GPIOD10复用为USART2
	
	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //GPIOD9与GPIOD10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PA9，PA10

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART2, ENABLE);  //使能串口1 
	
//USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//USART2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	
}


void USART2_IRQHandler(void)                	//串口1中断服务程序
{
	OSIntEnter();
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		if(Process_finish_flag){
			USART2_RX_DAT[USART2_RX_COUNT] = (USART2->DR);//USART_ReceiveData(USART2);//(USART1->DR);	//读取接收到的数据
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
































