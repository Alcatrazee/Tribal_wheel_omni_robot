#include "common_fcn.h"
#include "includes.h"

extern long steps1,steps2,steps3,steps_Y,steps_X;

void EXTI9_5_IRQHandler(void)
{
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line6)!=RESET){
	 if(PCin(7))	{
		steps2++;
	 }else{
		steps2--; 
	 }
	 EXTI_ClearITPendingBit(EXTI_Line6);
	 
	}else	if(EXTI_GetITStatus(EXTI_Line9)!=RESET)
	{
	 if(PCin(8))	{
		steps1--;
	 }else{
		steps1++; 
	 }
	 EXTI_ClearITPendingBit(EXTI_Line9); 
	 
	}else	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)
	{
		printf("on the top now!\r\n");
		EXTI_ClearITPendingBit(EXTI_Line5); 
	}
	
	OSIntExit();
}	

void EXTI0_IRQHandler(void)
{	
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){

		printf("on bottom now!\r\n");
		EXTI_ClearITPendingBit(EXTI_Line0); 
	}
	OSIntExit();
}

void EXTI15_10_IRQHandler(void)
{	
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line11)!=RESET){
	 if(PDin(10))	{
		steps3++;
	 }else{
		steps3--; 
	 }
	 EXTI_ClearITPendingBit(EXTI_Line11); 
  }else if(EXTI_GetITStatus(EXTI_Line14)!=RESET)
	{
	 if(PEin(6))	{
			steps_X--;
	 }else{
			steps_X++; 
	 }
	 EXTI_ClearITPendingBit(EXTI_Line14); 
	}
	OSIntExit();
}	

void EXTI4_IRQHandler(void)
{	
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET){
	 if(PBin(0))	{

	 }else{

	 }
	 EXTI_ClearITPendingBit(EXTI_Line4); 
	}
	OSIntExit();
}

void EXTI1_IRQHandler(void)
{	
	OSIntEnter();
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET){
	 if(PEin(3))	{
			steps_Y--;
	 }else{
			steps_Y++; 
	 }
	 EXTI_ClearITPendingBit(EXTI_Line1); //���LINE6�ϵ��жϱ�־λ 
	}
	OSIntExit();
}




void EXTIX_Init(void)
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOFʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_4 | GPIO_Pin_14|GPIO_Pin_0|GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_3;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOC,GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8 | GPIO_Pin_9);//GPIOF9,F10���øߣ�����
	GPIO_SetBits(GPIOD,GPIO_Pin_11 | GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	GPIO_SetBits(GPIOE,GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_3);
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource1);
	
  /* ����EXTI_Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line6|EXTI_Line9|EXTI_Line11|EXTI_Line4|EXTI_Line14|EXTI_Line1|EXTI_Line5|EXTI_Line0;//LINE0
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�����ش��� 
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
  EXTI_Init(&EXTI_InitStructure);//����
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;//�ⲿ�ж�0
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);//����
	   
}












