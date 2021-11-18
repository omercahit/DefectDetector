#include <stm32f4xx.h>
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_adc.h"
#include "misc.h"
#include "stdio.h"

#define STRING_SIZE 100

char message[STRING_SIZE];

/*
** Rotates the servo to the given angle between 0-180 degrees
** 
** 600 corresponds to the %3 duty cycle which is 0 degrees for this specific servo (MG996R)
** 2300 corresponds to the %11.5 duty cycle which is 180 degrees for this specific servo (MG996R)
*/
void rotateServo(int degree){	
	int val = (degree*(1700/180))+600;
	
	if(degree <= 0){
		TIM_SetCompare4(TIM4, 600);
	} else if(degree >= 180){
		TIM_SetCompare4(TIM4, 2300);
	} else {
		TIM_SetCompare4(TIM4, val);
	}
}

void USART1_puts(volatile char *s){
	while(*s){
		// busy wait until data register is empty
		while(!(USART1->SR & 0x00000040));
		USART_SendData(USART1, *(s++));
	 }	
}

/*
** If UART1 recieves 'f', servo rotates to close position
** If UART1 recieves 'o', servo rotates to open position
*/
void USART1_IRQHandler(void) {
	//Check if interrupt was because data is received
  if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
		//Do your stuff here
		/*switch(USART_ReceiveData(USART1)){
			case '0':
				rotateServo(45);
				break;
			case '1':
				rotateServo(0);
				break;
		}*/
		if (USART_ReceiveData(USART1)=='0'){
		rotateServo(45);
		}
		else if (USART_ReceiveData(USART1)=='1'){
		rotateServo(5);
		}
		else {
		//USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		NVIC_SystemReset();
		}
		sprintf(message, "%c\r\n", USART_ReceiveData(USART1));
		USART1_puts(message); // send recieved char for debugging purposes
		//Clear interrupt flag
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}

int main(){
	GPIO_InitTypeDef rxtxInit, pwmPinInit;
	USART_InitTypeDef usartInit;
	NVIC_InitTypeDef nvicInit;
	TIM_TimeBaseInitTypeDef timInit;
	TIM_OCInitTypeDef pwmInit;

	GPIO_StructInit(&rxtxInit);
	GPIO_StructInit(&pwmPinInit);
	USART_StructInit(&usartInit);
	
	//ENABLE CLOCKS
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	//USART B6TX, B7RX PIN CONFIG 
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	rxtxInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	rxtxInit.GPIO_Mode = GPIO_Mode_AF;
	rxtxInit.GPIO_OType = GPIO_OType_PP;
	rxtxInit.GPIO_PuPd = GPIO_PuPd_UP;
	rxtxInit.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &rxtxInit);
	
	//USART CONFIG
	usartInit.USART_BaudRate = 115200;
	USART_Init(USART1, &usartInit);
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	//NVIC CONFIG OF USART
	nvicInit.NVIC_IRQChannel = USART1_IRQn;
	nvicInit.NVIC_IRQChannelCmd = ENABLE;
	nvicInit.NVIC_IRQChannelPreemptionPriority = 1;
	nvicInit.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvicInit);
	
	//TIM4 CONFIG
	timInit.TIM_Prescaler = ((SystemCoreClock /2) / 1000000) - 1; // 1MHz tick
	timInit.TIM_CounterMode = TIM_CounterMode_Up;
	timInit.TIM_Period = 19999; // 50Hz update
	timInit.TIM_ClockDivision = TIM_CKD_DIV1;
	timInit.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &timInit);

	//D15 PWM PIN CONFIG
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	pwmPinInit.GPIO_Pin = GPIO_Pin_15;
	pwmPinInit.GPIO_Mode = GPIO_Mode_AF;
	pwmPinInit.GPIO_OType = GPIO_OType_PP;
	pwmPinInit.GPIO_PuPd = GPIO_PuPd_NOPULL;
	pwmPinInit.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &pwmPinInit);
	
	//PWM CONFIG
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1; 
	pwmInit.TIM_OutputState = TIM_OutputState_Enable;
	pwmInit.TIM_Pulse = 0;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM4, &pwmInit);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_Cmd(TIM4, ENABLE);
	
	rotateServo(31);
	while(1);
}
