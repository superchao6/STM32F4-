#ifndef __CONTROL_H
#define __CONTROL_H

#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "tim.h"
#include "main.h"


/* ���Ƚ�ֵ */
#define PWM_MAX_PERIOD_COUNT              (PWM_PERIOD_COUNT - 30)    //���PWMŪ�������ģ�һЩ������ͻ�������⣨Ӳ���ϵ�ԭ��
#define PWM2_MAX_PERIOD_COUNT              (PWM2_PERIOD_COUNT - 30)


/****************��ߵ�����ų�ʼ��**************/
/* ʹ����� */
#define MOTOR_L_ENABLE()      HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);  

/* ������� */
#define MOTOR_L_DISABLE()     HAL_TIM_PWM_Stop(&htim13,TIM_CHANNEL_1);



/****************�ұߵ�����ų�ʼ��**************/      //����������ת��һ�µ�ʱ��Ҫͬʱ������ĺ궨���encode.c�������������������
/* �����ٶȣ�ռ�ձȣ�2 */     
//#define SET_R_PWM(pwm_r)       __HAL_TIM_SET_COMPARE(&htim14,TIM_CHANNEL_1,pwm_r)    // ���ñȽϼĴ�����ֵ 

/* ʹ�����2 */
#define MOTOR_R_ENABLE()                  HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);   

/* �������2 */
#define MOTOR_R_DISABLE()                 HAL_TIM_PWM_Stop(&htim14,TIM_CHANNEL_1);   
    


/* ����������ֱ��� */
#define ENCODER_RESOLUTION                     12		//����������

/* ������Ƶ֮����ֱܷ��� */
#define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 4)  /* 4��Ƶ����ֱܷ��� */

/* ���ٵ�����ٱ� */
#define REDUCTION_RATIO  34

/*��sysTick�����PID��������ڣ��Ժ���Ϊ��λ*/
#define SPEED_PID_PERIOD  20    //���Ҫ����ʱ��7���ж�����
#define TARGET_SPEED_MAX  100  //

/*���Ӱ뾶*/
#define RADIUS 	3.35    
#define Perimeter 	(RADIUS*2*3.14) 

//�����
void Set_Motor_L_Behind_Forward(void);
void Set_Motor_L_Behind_Back(void);
void Set_Motor_L_Behind_Stop(void);
float speed_pid_L_Behind_control(void);
void Set_PWM_L_Behind(int PWM_L_Behind);
//�Һ���
void Set_Motor_R_Behind_Forward(void);
void Set_Motor_R_Behind_Back(void);
void Set_Motor_R_Behind_Stop(void);
float speed_pid_R_Behind_control(void);
void Set_PWM_R_Behind(int PWM_R_Behind);
//��ǰ��
void Set_Motor_L_Front_Forward(void);
void Set_Motor_L_Front_Back(void);
void Set_Motor_L_Front_Stop(void);
float speed_pid_L_Front_control(void);
void Set_PWM_L_Front(int PWM_L_Front);
//��ǰ��
void Set_Motor_R_Front_Forward(void);
void Set_Motor_R_Front_Back(void);
void Set_Motor_R_Front_Stop(void);
float speed_pid_R_Front_control(void);
void Set_PWM_R_Front(int PWM_R_Front);


void Speed_control(void);

#endif 

