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
#define PRINT_TASK_PRIO	5
#define PRINT_TASK_STK_SIZE 		256
OS_TCB Print_Task_TaskTCB;
CPU_STK PRINT_TASK_STK[PRINT_TASK_STK_SIZE];
void Print_task(void *p_arg);

//	状态更新任务
#define GESTURE_CHANGE_TASK_PRIO		5
#define GESTURE_CHANGE_STK_SIZE 		128
OS_TCB Gesture_Change_TaskTCB;
CPU_STK GESTURE_CHANGE_TASK_STK[GESTURE_CHANGE_STK_SIZE];
void Gesture_Change_task(void *p_arg);

//	控制任务
#define CTRL_TASK_PRIO		5
#define CTRL_STK_SIZE 		256
OS_TCB Ctrl_TaskTCB;
CPU_STK CTRL_TASK_STK[CTRL_STK_SIZE];
void CTRL_task(void *p_arg);

void BSP_Init(void);
void IMU_get_offset(void);											

extern RB_State State,Exp_State;								// 机器人状态以及期望状态结构体
extern State_in_RobotFrame Robot_Velocity;			// 机器人坐标系下的速度结构体
extern u8 Process_finish_flag;									// 串口2处理完成标志位
extern u8 Process_finish_flag_exp_state;				// 串口1处理完成标志位



int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	BSP_Init();
	OSInit(&err);		    													//初始化UCOSIII
	OS_CRITICAL_ENTER();													//进入临界区			 
	
	//创建开始任务
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,		//任务控制块
								 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区	 
	OSStart(&err);      //开启UCOSIII
								 
}


//开始任务任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	

	OS_CRITICAL_ENTER();	//进入临界区
	//创建led任务
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
				 
	//创建TASK2任务	 
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
								 
	OS_CRITICAL_EXIT();	//退出临界区
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
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
		OS_CRITICAL_ENTER();	//进入临界区
		//Print_IMU_Data();
		//printf("%f\t%f\r\n",Exp_State.frame_Vx,Exp_State.frame_Vy);
		printf("%f\t%f\t%f\t%f\t%f\t%f\r\n",State.frame_Vx,State.frame_Vy,State.angle,Robot_Velocity.frame_Vx,Robot_Velocity.frame_Vy,Robot_Velocity.omega);
		OS_CRITICAL_EXIT();	//退出临界区
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
		OSTimeDlyHMSM(0,0,0,2,OS_OPT_TIME_HMSM_STRICT,&err);
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
	delay_init(168);  															//时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断分组配置
	LED_Init();         														//LED初始化	
	uart_init(115200);															//初始化串口波特率为115200
	uart2_init(112500);															//初始化串口波特率为115200
	uart3_init(115200);														  //初始化串口波特率为115200
	PWM_Init(1400-1,3-1);														//初始化PWM频率为168000000/1400/3=40khz
	stop_all();																			//stop all motor
	EXTIX_Init();																		//初始化所有外部中断，包括五个编码器中断和2个未使用中断
	delay_ms(2000);																	//零漂前固定机器人
	IMU_get_offset();																//IMU获取零漂	
	printf("peripherals init complete...\r\n");			//完成初始化标志
	Process_finish_flag=1;													//恢复串口二处理完成标志位
	delay_ms(1000);																	//真正操作前等待1s给IMU准备好
}

void IMU_get_offset(void){
	while(Process_finish_flag);
	Data_Process();
	State.angle_offset = State.angle;
	State.angle -=State.angle_offset;

}
