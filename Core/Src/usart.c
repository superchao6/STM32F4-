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
#define USART2_SENDBUFF_MAX_BYTES	100U	//串口1发送缓冲区大小 单位字节
uint8_t  USART2_RecieveBuffer;		//暂存接收到的字符
uint8_t  USART2_Buffer[255];//收到的数据存放处
uint8_t  USART2_RxLine=0; //记录接收数据长度

uint8_t pid_data;

uint8_t  USART3_RecieveBuffer;		//暂存接收到的字符
uint8_t  USART3_Buffer[255];//收到的数据存放处
uint8_t  USART3_RxLine=0; //记录接收数据长度

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
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_RecieveBuffer, 1);		//一定记得再次打开中断
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
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&USART3_RecieveBuffer, 1);//串口3的中断开启
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
  * @brief  UART 仿printf发送
  * @param  format	输出的字符串
  * @retval 返回写入的字符总数
  */
int USART2Printf(const char* format, ...)
{
	static char sendBuff[USART2_SENDBUFF_MAX_BYTES] = { 0 };//发送缓冲区
	int bytes = 0;
	va_list list;

	va_start(list, format);
	bytes = vsprintf(sendBuff, format, list);//格式化输入
	va_end(list);
	/* 发送之前清除标志位 */
//	CLEAR_BIT(huart2.Instance->SR, USART_SR_TC_Msk);//往TC位写入0来清除TC位
	HAL_UART_Transmit(&huart2, (void*)sendBuff, bytes, 0xff);//阻塞式发送数据，发送等待时间为最大等待时间

	return bytes;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
			USART2_RxLine++;                      //每接收到一个数据，进入回调数据长度加1
			USART2_Buffer[USART2_RxLine-1]=USART2_RecieveBuffer;  //把每次接收到的数据保存到缓存数组
			
//			printf("USART2_RecieveBuffer = %c\r\n",USART2_RecieveBuffer);
			if(USART2_RxLine == 1)
				pid_data = USART2_RecieveBuffer;
			else if(USART2_RecieveBuffer==0x21)            //接收结束标志位，这个数据可以自定义，根据实际需求，这里只做示例使用，不一定是0x21
			{
//					printf("RXLen=%d\r\n",USART2_RxLine);
//					for(int i=0;i<USART2_RxLine;i++)
//							printf("UART DataBuff[%d] = %c\r\n",i,USART2_Buffer[i]);

					USART_PID_Adjust(4);//数据解析和参数赋值函数
					memset(USART2_Buffer,0,sizeof(USART2_Buffer));  //清空缓存数组
					USART2_RxLine=0;  //清空接收长度
			}
			USART2_Buffer[0]=0;
			HAL_UART_Receive_IT(&huart2, (uint8_t *)&USART2_RecieveBuffer, 1); //每接收一个数据，就打开一次串口中断接收，否则只会接收一个数据就停止接收
	}
	
	if(huart->Instance == USART3)
	{
//		UNUSED(huart);
		USART3_Buffer[USART3_RxLine++] = USART3_RecieveBuffer;
		if (USART3_RxLine >= 255) //根据自己的需要修改，当数据接收超多多少位就错了
		{
			USART3_RxLine=0;
			memset(USART3_Buffer,0,sizeof(USART3_Buffer)); 
			printf("USART3 ERROR!\r\n");
		}
		else if (USART3_RecieveBuffer=='}')  //数据最后一字节，根据实际情况修改
		{
//			HAL_UART_Transmit_IT(&huart3, (uint8_t *)USART3_Buffer,USART3_RxLine);  //如果接收到的数据都对，就将数据发送出去

//			printf("%s\r\n",USART3_Buffer);
//			memset(USART3_Buffer,0x00,sizeof(USART3_Buffer)); 
			USART3_RxLine=0;	
			USART3_flag = 1;
		}
//		USART3_RecieveBuffer=0;
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&USART3_RecieveBuffer, 1);		//一定记得再次打开中断
	}
}

/*
 * 解析出DataBuff中的数据
 * 返回解析得到的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
	
    for(uint8_t i=0;i<10;i++) // 查找等号和感叹号的位置
    {
        if(USART2_Buffer[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(USART2_Buffer[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(USART2_Buffer[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (USART2_Buffer[data_Start_Num]-48)  + (USART2_Buffer[data_Start_Num+2]-48)*0.1f + (USART2_Buffer[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (USART2_Buffer[data_Start_Num]-48)*10 + (USART2_Buffer[data_Start_Num+1]-48) + (USART2_Buffer[data_Start_Num+3]-48)*0.1f + (USART2_Buffer[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (USART2_Buffer[data_Start_Num]-48)*100 + (USART2_Buffer[data_Start_Num+1]-48)*10 + (USART2_Buffer[data_Start_Num+2]-48) + (USART2_Buffer[data_Start_Num+4]-48)*0.1f + (USART2_Buffer[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
//    printf("data=%.2f\r\n",data_return); 
    return data_return;
};

/*
 * 根据串口信息进行PID调参
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
//    printf("data=%.2f\r\n",data_Get);
		if(Motor_n == 0) // 左后电机
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
        else if(pid_data=='V'){ //目标速度
            set_pid_target(&pid_speed_L_Behind,data_Get);
						printf("V0=%.2f\r\n",data_Get);
				}
    }
		else if(Motor_n == 1) // 右后电机
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
		else if(Motor_n == 2) // 左前电机
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
        else if(pid_data=='V'){ //目标速度
            set_pid_target(&pid_speed_L_Front,data_Get);
						printf("V2=%.2f\r\n",data_Get);
				}
    }
		else if(Motor_n == 3) // 右前电机
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
        else if(pid_data=='V'){ //目标速度
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
