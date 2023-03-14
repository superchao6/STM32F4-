#include "pid.h"
#include "usart.h"
#include "control.h"
//����ȫ�ֱ���

_pid pid_speed_L_Behind, pid_speed_R_Behind, pid_speed_L_Front, pid_speed_R_Front;    

extern int  PWM_L_Behind, PWM_R_Behind, PWM_L_Front, PWM_R_Front;
extern int EncoderData_L_Behind,EncoderData_L_Behind, EncoderData_L_Front,EncoderData_L_Front;
extern _pid pid_speed_L_Behind, pid_speed_R_Behind, pid_speed_L_Front, pid_speed_R_Front;    


/**
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */
void PID_param_init()
{
  	/* �ٶ���س�ʼ������ */
    pid_speed_L_Behind.target_val=0.0;				
    pid_speed_L_Behind.actual_val=0.0;
    pid_speed_L_Behind.err=0.0;
    pid_speed_L_Behind.err_last=0.0;
    pid_speed_L_Behind.integral=0.0;
  
		pid_speed_L_Behind.Kp = 73.8;
		pid_speed_L_Behind.Ki = 20;
		pid_speed_L_Behind.Kd = 32;
  
	
  	/* �ٶ���س�ʼ������ */
    pid_speed_R_Behind.target_val=0.0;				
    pid_speed_R_Behind.actual_val=0.0;
    pid_speed_R_Behind.err=0.0;
    pid_speed_R_Behind.err_last=0.0;
    pid_speed_R_Behind.integral=0.0;
    
		pid_speed_R_Behind.Kp = 73.8;
		pid_speed_R_Behind.Ki = 20;
		pid_speed_R_Behind.Kd = 32;
		
		
		/* �ٶ���س�ʼ������ */
    pid_speed_L_Front.target_val=0.0;				
    pid_speed_L_Front.actual_val=0.0;
    pid_speed_L_Front.err=0.0;
    pid_speed_L_Front.err_last=0.0;
    pid_speed_L_Front.integral=0.0;
    
		pid_speed_L_Front.Kp = 73.8;
		pid_speed_L_Front.Ki = 20;
		pid_speed_L_Front.Kd = 32;
		
		
		/* �ٶ���س�ʼ������ */
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
  * @brief  ����Ŀ��ֵ
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ��
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // ���õ�ǰ��Ŀ��ֵ
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // ���ñ���ϵ�� P
		pid->Ki = i;    // ���û���ϵ�� I
		pid->Kd = d;    // ����΢��ϵ�� D
}

/**
  * @brief  �ٶ�PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float speed_pid_realize(_pid *pid, float actual_val)
{

		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = pid->target_val-actual_val;
	
    pid->integral += pid->err;    // ����ۻ�
	
	
	  /*�����޷�*/
	   	if (pid->integral >= 1200) {pid->integral = 1200;}
      else if (pid->integral < -1200)  {pid->integral = -1200;}

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
//    
//		USART2Printf("actual_val:%.2f\t target_val:%.2f\r\n",actual_val,pid->target_val);
//		USART2Printf("pid->Kp:%.2f\t pid->Ki:%.2f\r\n",pid->Kp,pid->Ki);	
//		USART2Printf("pid->Kp:%.2f\t target_val:%.2f\r\n",pid->Kp,pid->target_val);	
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}

