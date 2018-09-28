#include "common_fcn.h"
#include <stdlib.h>
#include "includes.h"

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

	
}

#define buf_length 13

u8 rec[buf_length];
u8 counter_usart1;
u8 Process_finish_flag_exp_state = 1;
extern u8 action_mode;
extern RB_State State;
extern u8 Motion_flag;
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		rec[counter_usart1] =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		counter_usart1++;
		if(rec[0]=='G'||rec[0]=='V'||rec[0]=='S'||rec[0]=='P'){
			if(rec[counter_usart1-1]=='\n'&&rec[counter_usart1-2]=='\r'){
				Process_finish_flag_exp_state = 0;
			}
		}else{
			Clear();
		}
  } 
}


 void UART1_Put_Char(unsigned char DataToSend)
{
	while((USART1->SR&0x40)==0);  
    USART1->DR = (u8) DataToSend;  
}

void  UART1_Put2_Char(u8 arr[2]){
	UART1_Put_Char(arr[0]);
	UART1_Put_Char(arr[1]);
}

void UART1_Put_String(char *p){
	while(*p!='\0'){
		UART1_Put_Char(*p);
		p++;
	}
}

void UART1_Put_String_with_space(char *p){
	while(*p!='\0'){
		UART1_Put_Char(*p);
		p++;
	}
	UART1_Put_Char(' ');
}

void UART1_Put_Char_with_space(char DataToSend){
	UART1_Put_Char(DataToSend);
	UART1_Put_Char(' ');
}

void UART1_Print_timestamp(void){
	OS_ERR err;
	u32 time = OSTimeGet(&err);
	char temp[10]={0};
	sprintf(temp,"%d",time);
	UART1_Put_String(temp);
	UART1_Put_String("\r\n");
}

void Clear(void){
	u8 i;
	for(i=0;i<buf_length;i++){
		rec[i] = 0;
	}
	counter_usart1 = 0;
}

extern RB_State Exp_State;
u8 rec_bit=0;
extern u8 Motion_ENABLE;
extern float Kp,Kd;
void Process(){
	float cvt_result[usart1_data_num]={0};
	if(rec[0]=='S')
		Motion_ENABLE=!Motion_ENABLE;
	else{
		split_message(cvt_result);
		switch(rec[0]){
			case 'G':	
								Exp_State.frame_X = cvt_result[0];
								Exp_State.frame_Y = cvt_result[1];
								Exp_State.angle = cvt_result[2];							//fill the array of angle
								action_mode = position_mode;
								Motion_ENABLE = 1;
								break;
			case 'V':	
								Exp_State.frame_Vx = cvt_result[0];
								Exp_State.frame_Vy = cvt_result[1];
								Exp_State.angle = cvt_result[2];							//fill the array of angle
								action_mode = velocity_mode;
								Motion_ENABLE = 1;
								break;
			case 'P':
								Kp = cvt_result[0];
								Kd = cvt_result[1];
			printf("%f\t%f\r\n",Kp,Kd);
		}
	}
}

void split_message(float split_result[usart1_data_num]){
	u8 i=0,j=0;
	char *a=0;
	const char *p = " ";
	a = strtok(rec,p);	
	for(i=0;i<4;i++){
		if(strcmp("G",a)&&strcmp("V",a)&&strcmp("P",a)){
			split_result[j] = atof(a);
			j++;
		}
		a = strtok(NULL,p);	
	}
}
//command: Gxxxx xxxx
//				  xpos ypos
//			unit: mm

