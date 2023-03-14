/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "pid.h"
#include "control.h"
#include "pid.h"
#define USART2_SENDBUFF_MAX_BYTES	100U	//����1���ͻ�������С ��λ�ֽ�
uint8_t  USART2_RecieveBuffer;		//�ݴ���յ����ַ�
uint8_t  USART2_Buffer[255];//�յ������ݴ�Ŵ�
uint8_t  USART2_RxLine=0; //��¼�������ݳ���

uint8_t pid_data;

uint8_t  USART3_RecieveBuffer;		//�ݴ���յ����ַ�
uint8_t  USART3_Buffer[255];//�յ������ݴ�Ŵ�
uint8_t  USART3_RxLine=0; //��¼�������ݳ���

uint8_t USART2_START = 0;

uint8_t USART2_flag = 0;
uint8_t USART3_flag = 0;

extern uint8_t  ESP32_Buffer[255];

extern int EncoderData_L_Behind,EncoderData_L_Behind, EncoderData_L_Front,EncoderData_L_Front;
extern _pid pid_speed_L_Behind, pid_speed_R_Behind, pid_speed_L_Front, pid_speed_R_Front;    

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_RecieveBuffer, 1);		//һ���ǵ��ٴδ��ж�
  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&USART3_RecieveBuffer, 1);//����3���жϿ���
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PD5     ------> USART2_TX
    PD6     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5|GPIO_PIN_6);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  UART ��printf����
  * @param  format	������ַ���
  * @retval ����д����ַ�����
  */
int USART2Printf(const char* format, ...)
{
	static char sendBuff[USART2_SENDBUFF_MAX_BYTES] = { 0 };//���ͻ�����
	int bytes = 0;
	va_list list;

	va_start(list, format);
	bytes = vsprintf(sendBuff, format, list);//��ʽ������
	va_end(list);
	/* ����֮ǰ�����־λ */
//	CLEAR_BIT(huart2.Instance->SR, USART_SR_TC_Msk);//��TCλд��0�����TCλ
	HAL_UART_Transmit(&huart2, (void*)sendBuff, bytes, 0xff);//����ʽ�������ݣ����͵ȴ�ʱ��Ϊ���ȴ�ʱ��

	return bytes;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
			USART2_RxLine++;                      //ÿ���յ�һ�����ݣ�����ص����ݳ��ȼ�1
			USART2_Buffer[USART2_RxLine-1]=USART2_RecieveBuffer;  //��ÿ�ν��յ������ݱ��浽��������
			
//			printf("USART2_RecieveBuffer = %c\r\n",USART2_RecieveBuffer);
			if(USART2_RxLine == 1)
				pid_data = USART2_RecieveBuffer;
			else if(USART2_RecieveBuffer==0x21)            //���ս�����־λ��������ݿ����Զ��壬����ʵ����������ֻ��ʾ��ʹ�ã���һ����0x21
			{
//					printf("RXLen=%d\r\n",USART2_RxLine);
//					for(int i=0;i<USART2_RxLine;i++)
//							printf("UART DataBuff[%d] = %c\r\n",i,USART2_Buffer[i]);

					USART_PID_Adjust(4);//���ݽ����Ͳ�����ֵ����
					memset(USART2_Buffer,0,sizeof(USART2_Buffer));  //��ջ�������
					USART2_RxLine=0;  //��ս��ճ���
			}
			USART2_Buffer[0]=0;
			HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_RecieveBuffer, 1); //ÿ����һ�����ݣ��ʹ�һ�δ����жϽ��գ�����ֻ�����һ�����ݾ�ֹͣ����
	}
	
	if(huart->Instance == USART3)
	{
//		UNUSED(huart);
		USART3_Buffer[USART3_RxLine++] = USART3_RecieveBuffer;
		if (USART3_RxLine >= 255) //�����Լ�����Ҫ�޸ģ������ݽ��ճ������λ�ʹ���
		{
			USART3_RxLine=0;
			memset(USART3_Buffer,0,sizeof(USART3_Buffer)); 
			printf("USART3 ERROR!\r\n");
		}
		else if (USART3_RecieveBuffer=='}')  //�������һ�ֽڣ�����ʵ������޸�
		{
//			HAL_UART_Transmit_IT(&huart3, (uint8_t *)USART3_Buffer,USART3_RxLine);  //������յ������ݶ��ԣ��ͽ����ݷ��ͳ�ȥ

//			printf("%s\r\n",USART3_Buffer);
//			memset(USART3_Buffer,0x00,sizeof(USART3_Buffer)); 
			USART3_RxLine=0;	
			USART3_flag = 1;
		}
//		USART3_RecieveBuffer=0;
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&USART3_RecieveBuffer, 1);		//һ���ǵ��ٴδ��ж�
	}
}

/*
 * ������DataBuff�е�����
 * ���ؽ����õ�������
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // ��¼����λ��ʼ�ĵط�
    uint8_t data_End_Num = 0; // ��¼����λ�����ĵط�
    uint8_t data_Num = 0; // ��¼����λ��
    uint8_t minus_Flag = 0; // �ж��ǲ��Ǹ���
    float data_return = 0; // �����õ�������
	
    for(uint8_t i=0;i<10;i++) // ���ҵȺź͸�̾�ŵ�λ��
    {
        if(USART2_Buffer[i] == '=') data_Start_Num = i + 1; // +1��ֱ�Ӷ�λ��������ʼλ
        if(USART2_Buffer[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(USART2_Buffer[data_Start_Num] == '-') // ����Ǹ���
    {
        data_Start_Num += 1; // ����һλ������λ
        minus_Flag = 1; // ����flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // ���ݹ�4λ
    {
        data_return = (USART2_Buffer[data_Start_Num]-48)  + (USART2_Buffer[data_Start_Num+2]-48)*0.1f + (USART2_Buffer[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // ���ݹ�5λ
    {
        data_return = (USART2_Buffer[data_Start_Num]-48)*10 + (USART2_Buffer[data_Start_Num+1]-48) + (USART2_Buffer[data_Start_Num+3]-48)*0.1f + (USART2_Buffer[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // ���ݹ�6λ
    {
        data_return = (USART2_Buffer[data_Start_Num]-48)*100 + (USART2_Buffer[data_Start_Num+1]-48)*10 + (USART2_Buffer[data_Start_Num+2]-48) + (USART2_Buffer[data_Start_Num+4]-48)*0.1f + (USART2_Buffer[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
//    printf("data=%.2f\r\n",data_return); 
    return data_return;
};

/*
 * ���ݴ�����Ϣ����PID����
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // ��Ž��յ�������
//    printf("data=%.2f\r\n",data_Get);
		if(Motor_n == 0) // �����
    {
//				printf("Motor_n = 0\r\n");
        if(pid_data=='P'){ // 
            pid_speed_L_Behind.Kp = data_Get;
						printf("P0=%.2f\r\n",data_Get);
				}
        else if(pid_data=='I'){ // 
            pid_speed_L_Behind.Ki = data_Get;
						printf("I0=%.2f\r\n",data_Get);
				}
        else if(pid_data=='D'){ // 
            pid_speed_L_Behind.Kd = data_Get;
						printf("D0=%.2f\r\n",data_Get);
				}
        else if(pid_data=='V'){ //Ŀ���ٶ�
            set_pid_target(&pid_speed_L_Behind,data_Get);
						printf("V0=%.2f\r\n",data_Get);
				}
    }
		else if(Motor_n == 1) // �Һ���
    {
				printf("Motor_n = 1\r\n");
        if(pid_data=='P'){ // 
            pid_speed_R_Behind.Kp = data_Get;
						printf("P1=%.2f\r\n",data_Get);
				}
        else if(pid_data=='I'){ // 
            pid_speed_R_Behind.Ki = data_Get;
						printf("I1=%.2f\r\n",data_Get);
				}
        else if(pid_data=='D'){ // 
            pid_speed_R_Behind.Kd = data_Get;
						printf("D1=%.2f\r\n",data_Get);
				}
        else if(pid_data=='V'){ //
            set_pid_target(&pid_speed_R_Behind,data_Get);
						printf("V1=%.2f\r\n",data_Get);
				}
    }
		else if(Motor_n == 2) // ��ǰ���
    {
				printf("Motor_n = 2\r\n");
        if(pid_data=='P'){ //
            pid_speed_L_Front.Kp = data_Get;
						printf("P2=%.2f\r\n",data_Get);
				}
        else if(pid_data=='I'){ // 
            pid_speed_L_Front.Ki = data_Get;
						printf("I2=%.2f\r\n",data_Get);
				}
        else if(pid_data=='D'){ // 
            pid_speed_L_Front.Kd = data_Get;
						printf("D2=%.2f\r\n",data_Get);
				}
        else if(pid_data=='V'){ //Ŀ���ٶ�
            set_pid_target(&pid_speed_L_Front,data_Get);
						printf("V2=%.2f\r\n",data_Get);
				}
    }
		else if(Motor_n == 3) // ��ǰ���
    {
				printf("Motor_n = 3\r\n");
        if(pid_data=='P'){ // 
            pid_speed_R_Front.Kp = data_Get;
						printf("P3=%.2f\r\n",data_Get);
				}
        else if(pid_data=='I'){ //
            pid_speed_R_Front.Ki = data_Get;
						printf("I3=%.2f\r\n",data_Get);
				}
        else if(pid_data=='D'){ // 
            pid_speed_R_Front.Kd = data_Get;
						printf("D3=%.2f\r\n",data_Get);
				}
        else if(pid_data=='V'){ //Ŀ���ٶ�
            set_pid_target(&pid_speed_R_Front,data_Get);
						printf("V3=%.2f\r\n",data_Get);
				}
    }
		else if(Motor_n == 4)
		{
			set_pid_target(&pid_speed_L_Behind,data_Get);
			set_pid_target(&pid_speed_R_Behind,data_Get);
			set_pid_target(&pid_speed_L_Front,data_Get);
			set_pid_target(&pid_speed_R_Front,data_Get);
		}
}


/* USER CODE END 1 */
