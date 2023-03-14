#include "pid.h"
#include "usart.h"
#include "control.h"
//定义全局变量

_pid pid_speed_L_Behind, pid_speed_R_Behind, pid_speed_L_Front, pid_speed_R_Front;    

extern int  PWM_L_Behind, PWM_R_Behind, PWM_L_Front, PWM_R_Front;
extern int EncoderData_L_Behind,EncoderData_L_Behind, EncoderData_L_Front,EncoderData_L_Front;
extern _pid pid_speed_L_Behind, pid_speed_R_Behind, pid_speed_L_Front, pid_speed_R_Front;    


/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init()
{
  	/* 速度相关初始化参数 */
    pid_speed_L_Behind.target_val=0.0;				
    pid_speed_L_Behind.actual_val=0.0;
    pid_speed_L_Behind.err=0.0;
    pid_speed_L_Behind.err_last=0.0;
    pid_speed_L_Behind.integral=0.0;
  
		pid_speed_L_Behind.Kp = 73.8;
		pid_speed_L_Behind.Ki = 20;
		pid_speed_L_Behind.Kd = 32;
  
	
  	/* 速度相关初始化参数 */
    pid_speed_R_Behind.target_val=0.0;				
    pid_speed_R_Behind.actual_val=0.0;
    pid_speed_R_Behind.err=0.0;
    pid_speed_R_Behind.err_last=0.0;
    pid_speed_R_Behind.integral=0.0;
    
		pid_speed_R_Behind.Kp = 73.8;
		pid_speed_R_Behind.Ki = 20;
		pid_speed_R_Behind.Kd = 32;
		
		
		/* 速度相关初始化参数 */
    pid_speed_L_Front.target_val=0.0;				
    pid_speed_L_Front.actual_val=0.0;
    pid_speed_L_Front.err=0.0;
    pid_speed_L_Front.err_last=0.0;
    pid_speed_L_Front.integral=0.0;
    
		pid_speed_L_Front.Kp = 73.8;
		pid_speed_L_Front.Ki = 20;
		pid_speed_L_Front.Kd = 32;
		
		
		/* 速度相关初始化参数 */
    pid_speed_R_Front.target_val=0.0;				
    pid_speed_R_Front.actual_val=0.0;
    pid_speed_R_Front.err=0.0;
    pid_speed_R_Front.err_last=0.0;
    pid_speed_R_Front.integral=0.0;
    
		pid_speed_R_Front.Kp = 70.0;
		pid_speed_R_Front.Ki = 20;
		pid_speed_R_Front.Kd = 32;
		
}

/**
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // 设置当前的目标值
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // 设置比例系数 P
		pid->Ki = i;    // 设置积分系数 I
		pid->Kd = d;    // 设置微分系数 D
}

/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
  */
float speed_pid_realize(_pid *pid, float actual_val)
{

		/*计算目标值与实际值的误差*/
    pid->err = pid->target_val-actual_val;
	
    pid->integral += pid->err;    // 误差累积
	
	
	  /*积分限幅*/
	   	if (pid->integral >= 1200) {pid->integral = 1200;}
      else if (pid->integral < -1200)  {pid->integral = -1200;}

		/*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*误差传递*/
    pid->err_last=pid->err;
//    
//		USART2Printf("actual_val:%.2f\t target_val:%.2f\r\n",actual_val,pid->target_val);
//		USART2Printf("pid->Kp:%.2f\t pid->Ki:%.2f\r\n",pid->Kp,pid->Ki);	
//		USART2Printf("pid->Kp:%.2f\t target_val:%.2f\r\n",pid->Kp,pid->target_val);	
		/*返回当前实际值*/
    return pid->actual_val;
}

