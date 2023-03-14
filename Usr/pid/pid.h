#ifndef __PID_H
#define	__PID_H
#include "stm32f4xx.h"
#include "usart.h"
//#include "bsp_debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>





typedef struct
{
    float target_val;           //Ŀ��ֵ
    float actual_val;        		//ʵ��ֵ
    float err;             			//����ƫ��ֵ
    float err_last;          		//������һ��ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
}_pid;

 void PID_param_init(void);
 void set_pid_target(_pid *pid, float temp_val);
 float get_pid_target(_pid *pid);
 void set_p_i_d(_pid *pid, float p, float i, float d);

float speed_pid_realize(_pid *pid, float actual_val);

#endif
