#include "encoder.h"
#include "stm32f4xx.h"
#include "tim.h"//包含tim头文件
#include "usart.h"

long EncoderData_L_Behind, EncoderData_R_Behind, EncoderData_L_Front, EncoderData_R_Front;//编码器数值
int Direction_L_Behind,Direction_R_Behind, Direction_L_Front,Direction_R_Front;//电机方向

//如果编码器有时候捕获的数值会跳变的话，可以在cubemx的编码器设置里面加上滤波，可以设置1~16的值


/*******************实际运行时读取编码器数值************************/   //当两个轮子转向不一致的时候，要同时调这里编码器捕获脉冲的正负和control.h里的PWM通道宏定义
void Read_Encoder(void)//读取电机脉冲
{
		//后轮
		Direction_L_Behind = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);  
		EncoderData_L_Behind = __HAL_TIM_GET_COUNTER(&htim3);	
		Direction_R_Behind = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4); 
		EncoderData_R_Behind = __HAL_TIM_GET_COUNTER(&htim4);
		//前轮
		Direction_L_Front = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);  
		EncoderData_L_Front = __HAL_TIM_GET_COUNTER(&htim1);	
		Direction_R_Front = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2); 
		EncoderData_R_Front = __HAL_TIM_GET_COUNTER(&htim2);
	
//		printf("Direction_L_Behind:%d,,Direction_R_Behind:%d,,Direction_L_Front:%d,,Direction_R_Front:%d\r\n",Direction_L_Behind,Direction_R_Behind, Direction_L_Front,Direction_R_Front);//方向

	
		//后轮
		if(Direction_L_Behind == 0)
			EncoderData_L_Behind = - EncoderData_L_Behind;
		else if(Direction_L_Behind == 1 && EncoderData_L_Behind > 4000)
			EncoderData_L_Behind = - (__HAL_TIM_GET_COUNTER(&htim3) - 65536);
		else
			EncoderData_L_Behind = 0;
		
		if(Direction_R_Behind == 0)
			EncoderData_R_Behind = - EncoderData_R_Behind;
		else if(Direction_R_Behind == 1 && EncoderData_R_Behind > 4000)
			EncoderData_R_Behind = - (__HAL_TIM_GET_COUNTER(&htim4) - 65536);
		else
			EncoderData_R_Behind = 0;
		
		
		//前轮
		if(Direction_L_Front == 0)
			EncoderData_L_Front = EncoderData_L_Front;
		else if(Direction_L_Front == 1 && EncoderData_L_Front > 4000)
			EncoderData_L_Front = __HAL_TIM_GET_COUNTER(&htim1) - 65536;
		else
			EncoderData_L_Front = 0;
		
//		if(Direction_R_Front == 0)
//			EncoderData_R_Front = EncoderData_R_Front;
//		else if(Direction_R_Front == 1 && EncoderData_R_Front > 4000)
//			EncoderData_R_Front = __HAL_TIM_GET_COUNTER(&htim2) - 65536;
//		else
//			EncoderData_R_Front = 0;
		if(EncoderData_R_Front!=0)
			EncoderData_R_Front = ~EncoderData_R_Front;
			
		
//		printf("EncoderData_L_Behind:%4ld,,EncoderData_R_Behind:%4ld,,EncoderData_L_Front:%4ld,,EncoderData_R_Front:%4ld\r\n",EncoderData_L_Behind,EncoderData_R_Behind,EncoderData_L_Front,EncoderData_R_Front);

		
		__HAL_TIM_SetCounter(&htim3,0);//清空计数器
		__HAL_TIM_SetCounter(&htim4,0);//清空计数器
		
		__HAL_TIM_SetCounter(&htim1,0);//清空计数器
		__HAL_TIM_SetCounter(&htim2,0);//清空计数器
		
}
