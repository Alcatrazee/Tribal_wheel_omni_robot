#include "common_fcn.h"
#include "includes.h"

void uart3_init(u32 bouderrate){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); 
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bouderrate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); 
	
  USART_Cmd(USART3, ENABLE);  

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
}

#define usart3buffersize  14
u8 Res[usart3buffersize];
u8 counter=0;
u16 tracker_pos[2]={0,0};
u16 tracker_area=0;
u8 CircleFound=0;

void USART3_IRQHandler(void)                	
{
	OSIntEnter();
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		Res[counter]=(USART3->DR);//(USART1->DR);	//读取接收到的数据Res[counter]
		counter++;
		if(Res[0]!='P')
			USART3_Clear_Buff();
		if(counter==usart3buffersize){
			USART3_Process();
			USART3_Clear_Buff();
		}
  } 
	
	OSIntExit();
}


u8 rec3_bit=0;
u8 rec_state=0;
// format P Xpos area
//				1  3    4   at max 
//        8 in total
void USART3_Process(void){
	char str_x_pos[3]={0};
	char str_y_pos[3]={0};
	char str_area[3] = {0};
	u8 i=0;
	for(i=0;i<3;i++){
		str_x_pos[i]=Res[i+2];
	}
	tracker_pos[0]=atof(str_x_pos);
	for(i=0;i<3;i++){
		str_y_pos[i]=Res[i+6];
	}
	tracker_pos[1]=atof(str_y_pos);
	for(i=0;i<3;i++){
		str_area[i]=Res[i+10];
	}
	tracker_area=atof(str_area);
	rec_state=1;
	rec3_bit=1;
//	printf("%d\t%d\t%d\r\n",tracker_pos[0],tracker_pos[1],tracker_area);
}

//////////////////////////////////////////////////////////
//function name:	USART3_Clear_Buff											//
//function		 :	clear rec buff												//
//input Para	 :	None																	//
//output       :  None																	//
//////////////////////////////////////////////////////////
void USART3_Clear_Buff(void){
	u8 i;
	for(i=0;i<usart3buffersize;i++){
		Res[i]=0;
	}
	counter=0;
}

//////////////////////////////////////////////////////////
//function name:	UART3_Put_Char												//
//function		 :	Send a letter 												//
//input Para	 :	None																	//
//output       :  None																	//
//////////////////////////////////////////////////////////
 void UART3_Put_Char(unsigned char DataToSend)
{
	while((USART3->SR&0x40)==0);  
    USART3->DR = (u8) DataToSend;  
}







