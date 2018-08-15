#include "common_fcn.h"
#include "includes.h"
#include "brocaster.h"

#define START_TASK_PRIO		3
#define START_STK_SIZE 		128
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

// task1 is led task
#define LED_TASK_PRIO		6
#define LED_TASK_STK_SIZE 128
OS_TCB LED_TASK_TaskTCB;
CPU_STK LED_TASK_STK[LED_TASK_STK_SIZE];
void Led_task(void *p_arg);

// task2 is print task
#define PRINT_TASK_PRIO	6
#define PRINT_TASK_STK_SIZE 		128
OS_TCB Print_Task_TaskTCB;
CPU_STK PRINT_TASK_STK[PRINT_TASK_STK_SIZE];
void Print_task(void *p_arg);

#define GESTURE_CHANGE_TASK_PRIO		6
#define GESTURE_CHANGE_STK_SIZE 		128
OS_TCB Gesture_Change_TaskTCB;
CPU_STK GESTURE_CHANGE_TASK_STK[GESTURE_CHANGE_STK_SIZE];
void Gesture_Change_task(void *p_arg);

#define CTRL_TASK_PRIO		5
#define CTRL_STK_SIZE 		256
OS_TCB Ctrl_TaskTCB;
CPU_STK CTRL_TASK_STK[CTRL_STK_SIZE];
void CTRL_task(void *p_arg);

void BSP_Init(void);

OS_SEM MySem;
OS_SEM MySem2;
OS_Q MyQueue;
OS_FLAG_GRP usart2_flags_group;

extern RB_State State,Exp_State;
extern float Yaw;
extern u8 USART2_RX_DAT[11];
extern u8 Process_finish_flag;
extern long steps1,steps2,steps3,steps_Y,steps_X;
extern u8 Process_finish_flag_exp_state;

int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	BSP_Init();
	
	OSInit(&err);		    //��ʼ��UCOSIII
	OS_CRITICAL_ENTER();	//�����ٽ���			 
	
	//������ʼ����
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,		//������ƿ�
								 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);      //����UCOSIII
								 
}


//��ʼ����������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
	
	OSSemCreate(&MySem,"My Sem",1,&err);	
//	OSSemCreate(&MySem2,"My Sem 2",1,&err);	
//	OSQCreate(&MyQueue,"My Queue",40,&err);

	OSFlagCreate((OS_FLAG_GRP*)&usart2_flags_group,
						   (CPU_CHAR   *)"usart2_flags_group",
						   (OS_FLAGS    )0,
						   (OS_ERR     *)&err);

	OS_CRITICAL_ENTER();	//�����ٽ���
	//����led����
	OSTaskCreate(  (OS_TCB 	* )&LED_TASK_TaskTCB,		
								 (CPU_CHAR	* )"led task", 		
                 (OS_TASK_PTR )Led_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED_TASK_PRIO,     
                 (CPU_STK   * )&LED_TASK_STK[0],	
                 (CPU_STK_SIZE)LED_TASK_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED_TASK_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
                 (OS_ERR 	* )&err);				
				 
	//����TASK2����	 
	OSTaskCreate((OS_TCB 	* )&Print_Task_TaskTCB,		
				 (CPU_CHAR	* )"print task", 		
                 (OS_TASK_PTR )Print_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )PRINT_TASK_PRIO,     	
                 (CPU_STK   * )&PRINT_TASK_STK[0],	
                 (CPU_STK_SIZE)PRINT_TASK_STK_SIZE/10,	
                 (CPU_STK_SIZE)PRINT_TASK_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			 
	
  // create gesture change task								 
	OSTaskCreate((OS_TCB 	* )&Gesture_Change_TaskTCB,		
				 (CPU_CHAR	* )"Gesture_Change_task", 		
                 (OS_TASK_PTR )Gesture_Change_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )GESTURE_CHANGE_TASK_PRIO,     	
                 (CPU_STK   * )&GESTURE_CHANGE_TASK_STK[0],	
                 (CPU_STK_SIZE)GESTURE_CHANGE_STK_SIZE/10,	
                 (CPU_STK_SIZE)GESTURE_CHANGE_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
							
	OSTaskCreate((OS_TCB 	* )&Ctrl_TaskTCB,		
				 (CPU_CHAR	* )"ctrl task", 		
                 (OS_TASK_PTR )CTRL_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )CTRL_TASK_PRIO,     	
                 (CPU_STK   * )&CTRL_TASK_STK[0],	
                 (CPU_STK_SIZE)CTRL_STK_SIZE/10,	
                 (CPU_STK_SIZE)CTRL_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
								 
//	OSTaskSuspend(&Gesture_Change_TaskTCB,&err);		 
//	OSTaskSuspend(&Print_Task_TaskTCB,&err);		 
//	OSTaskSuspend(&Ctrl_TaskTCB,&err);		 
	
	
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}

void Led_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		led_turn();
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void Print_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	while(1)
	{
		OS_CRITICAL_ENTER();	//�����ٽ���
		Print_IMU_Data();
		OS_CRITICAL_EXIT();	//�˳��ٽ���
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}



void Gesture_Change_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	while(1)
	{
		if(!Process_finish_flag){
			OS_CRITICAL_ENTER();
			Data_Process();
			OS_CRITICAL_EXIT();
			UART2_Clear();
			Process_finish_flag=1;
		}
		if(!Process_finish_flag_exp_state){
			OS_CRITICAL_ENTER();
			Process();
			OS_CRITICAL_EXIT();
			Clear();
			Process_finish_flag_exp_state = 1;
		}
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void CTRL_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		Calculate_State();
		Action();
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

void BSP_Init(void){
	delay_init(168);  															//ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�жϷ�������
	LED_Init();         														//LED��ʼ��	
	uart_init(115200);															//��ʼ�����ڲ�����Ϊ115200
	uart2_init(112500);	
	uart3_init(115200);														  //��ʼ�����ڲ�����Ϊ112500
	PWM_Init(2800-1,6-1);
	stop_all();
	EXTIX_Init();
	printf("peripherals init complete...\r\n");
	Process_finish_flag=1;
}

void IMU_get_offset(){
	if(!Process_finish_flag){
		Data_Process();
		State.angle_offset = State.angle;
		State.angle -=State.angle_offset;
	}
}
