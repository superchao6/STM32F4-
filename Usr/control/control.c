#include "control.h"
#include "usart.h"
#include "tim.h"
#include "pid.h"
#include "encoder.h"
#include "protocol.h"

int  PWM_L_Behind = 0, PWM_R_Behind = 0, PWM_L_Front = 0, PWM_R_Front = 0;
extern long EncoderData_L_Behind,EncoderData_R_Behind, EncoderData_L_Front,EncoderData_R_Front;
extern _pid pid_speed_L_Behind, pid_speed_R_Behind, pid_speed_L_Front, pid_speed_R_Front;    

unsigned char location_control_count = 0;  //执行频率不需要那么高的用这个事件计数，用在中断中


//左后轮 L_Behind
void Set_Motor_L_Behind_Forward(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
}
void Set_Motor_L_Behind_Back(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
}
void Set_Motor_L_Behind_Stop(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_RESET);
}

float speed_pid_L_Behind_control(void)  
{
    float cont_val = 0.0;                       // 当前控制值
    double actual_speed = 0;
//	  actual_speed = (EncoderData_L_Behind*1000*60)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);  //rpm
	  actual_speed = EncoderData_L_Behind;  //乱计算的rpm
		
	
//		USART2Printf("%lf\r\n",actual_speed);
	
    cont_val = speed_pid_realize(&pid_speed_L_Behind, actual_speed);    // 进行 PID 计算
	
		if(cont_val>2000)
			cont_val = 2000;
		else if(cont_val<-2000)
			cont_val = -2000;
//	printf("cont_val:%.2f",cont_val);
		
		return cont_val;
}

void Set_PWM_L_Behind(int PWM_L_Behind){
	if(PWM_L_Behind == 0){
		__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,PWM_L_Behind);
//		Set_Motor_L_Stop();
	}else if(PWM_L_Behind > 0){
		Set_Motor_L_Behind_Back();
		__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,PWM_L_Behind);
	}else{
		Set_Motor_L_Behind_Forward();
		__HAL_TIM_SetCompare(&htim13,TIM_CHANNEL_1,-PWM_L_Behind);
	}
}


//右后轮
void Set_Motor_R_Behind_Forward(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_SET);
}
void Set_Motor_R_Behind_Back(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_3,GPIO_PIN_RESET);
}
void Set_Motor_R_Behind_Stop(void){
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_RESET);
}

float speed_pid_R_Behind_control(void)  
{
    float cont_val = 0.0;                       // 当前控制值
    double actual_speed = 0;
//	  actual_speed = (EncoderData_R_Behind*1000*60)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);  //rpm
		
	  actual_speed = EncoderData_R_Behind;  //乱计算的rpm
	
//		USART2Printf("%lf\r\n",actual_speed);
	
    cont_val = speed_pid_realize(&pid_speed_R_Behind, actual_speed);    // 进行 PID 计算
	
		if(cont_val>2000)
			cont_val = 2000;
		else if(cont_val<-2000)
			cont_val = -2000;
		
		return cont_val;
}

void Set_PWM_R_Behind(int PWM_R_Behind){
	if(PWM_R_Behind == 0){
		__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,PWM_R_Behind);
	}else if(PWM_R_Behind > 0){
		Set_Motor_R_Behind_Forward();
		__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,PWM_R_Behind);
	}else{
		Set_Motor_R_Behind_Back();
		__HAL_TIM_SetCompare(&htim14,TIM_CHANNEL_1,-PWM_R_Behind);
	}
}


//左前轮 L_Front
void Set_Motor_L_Front_Forward(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
}
void Set_Motor_L_Front_Back(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
}
void Set_Motor_L_Front_Stop(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2|GPIO_PIN_3,GPIO_PIN_RESET);
}

float speed_pid_L_Front_control(void)  
{
    float cont_val = 0.0;                       // 当前控制值
    double actual_speed = 0;
//	  actual_speed = (EncoderData_L_Front*1000*60)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);  //rpm
	
	  actual_speed = EncoderData_L_Front;  //乱计算的rpm
	
//		USART2Printf("%lf\r\n",actual_speed);
	
    cont_val = speed_pid_realize(&pid_speed_L_Front, actual_speed);    // 进行 PID 计算
	
		if(cont_val>2000)
			cont_val = 2000;
		else if(cont_val<-2000)
			cont_val = -2000;
		
		int temp = actual_speed;

		return cont_val;
}

void Set_PWM_L_Front(int PWM_L_Front){
	if(PWM_L_Front == 0){
		__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,PWM_L_Front);
//		Set_Motor_L_Stop();
	}else if(PWM_L_Front > 0){
		Set_Motor_L_Front_Back();
		__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,PWM_L_Front);
	}else{
		Set_Motor_L_Front_Forward();
		__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,-PWM_L_Front);
	}
}


//右前轮
void Set_Motor_R_Front_Forward(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
}
void Set_Motor_R_Front_Back(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
}
void Set_Motor_R_Front_Stop(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4|GPIO_PIN_5,GPIO_PIN_RESET);
}

float speed_pid_R_Front_control(void)  
{
    float cont_val = 0.0;                       // 当前控制值
    double actual_speed = 0;
//	  actual_speed = (EncoderData_R_Front*1000*60)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*SPEED_PID_PERIOD);  //rpm
		
	  actual_speed = EncoderData_R_Front;  //乱计算的rpm
	
		USART2Printf("%lf\r\n",actual_speed);
	
    cont_val = speed_pid_realize(&pid_speed_R_Front, actual_speed);    // 进行 PID 计算
	
		if(cont_val>2000)
			cont_val = 2000;
		else if(cont_val<-2000)
			cont_val = -2000;
		
		return cont_val;
}

void Set_PWM_R_Front(int PWM_R_Front){
	if(PWM_R_Front == 0){
		__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,PWM_R_Front);
	}else if(PWM_R_Front > 0){
		Set_Motor_R_Front_Back();
		__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,PWM_R_Front);
	}else{
		Set_Motor_R_Front_Forward();
		__HAL_TIM_SetCompare(&htim11,TIM_CHANNEL_1,-PWM_R_Front);
	}
}



void Speed_control(void)
{
		//mpu_dmp_get_data放在这会影响到编码器脉冲的获取。
	
		PWM_L_Behind = speed_pid_L_Behind_control();    
		Set_PWM_L_Behind(PWM_L_Behind);
	
		PWM_R_Behind = speed_pid_R_Behind_control();    
		Set_PWM_R_Behind(PWM_R_Behind);
	
		PWM_L_Front = speed_pid_L_Front_control(); 
		Set_PWM_L_Front(PWM_L_Front);   
	
		PWM_R_Front = speed_pid_R_Front_control();  
		Set_PWM_R_Front(PWM_R_Front);		

}

