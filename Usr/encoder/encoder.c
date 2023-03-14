#include "encoder.h"
#include "stm32f4xx.h"
#include "tim.h"//����timͷ�ļ�
#include "usart.h"

long EncoderData_L_Behind, EncoderData_R_Behind, EncoderData_L_Front, EncoderData_R_Front;//��������ֵ
int Direction_L_Behind,Direction_R_Behind, Direction_L_Front,Direction_R_Front;//�������

//�����������ʱ�򲶻����ֵ������Ļ���������cubemx�ı�����������������˲�����������1~16��ֵ


/*******************ʵ������ʱ��ȡ��������ֵ************************/   //����������ת��һ�µ�ʱ��Ҫͬʱ��������������������������control.h���PWMͨ���궨��
void Read_Encoder(void)//��ȡ�������
{
		//����
		Direction_L_Behind = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);  
		EncoderData_L_Behind = __HAL_TIM_GET_COUNTER(&htim3);	
		Direction_R_Behind = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4); 
		EncoderData_R_Behind = __HAL_TIM_GET_COUNTER(&htim4);
		//ǰ��
		Direction_L_Front = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);  
		EncoderData_L_Front = __HAL_TIM_GET_COUNTER(&htim1);	
		Direction_R_Front = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2); 
		EncoderData_R_Front = __HAL_TIM_GET_COUNTER(&htim2);
	
//		printf("Direction_L_Behind:%d,,Direction_R_Behind:%d,,Direction_L_Front:%d,,Direction_R_Front:%d\r\n",Direction_L_Behind,Direction_R_Behind, Direction_L_Front,Direction_R_Front);//����

	
		//����
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
		
		
		//ǰ��
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

		
		__HAL_TIM_SetCounter(&htim3,0);//��ռ�����
		__HAL_TIM_SetCounter(&htim4,0);//��ռ�����
		
		__HAL_TIM_SetCounter(&htim1,0);//��ռ�����
		__HAL_TIM_SetCounter(&htim2,0);//��ռ�����
		
}
