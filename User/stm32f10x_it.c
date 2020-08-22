/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "my_own.h"

extern uchar rcount;
extern uchar rbyte1,rbyte2,rbyte3,comdok;
extern uchar basictime;
extern uchar eepromupdate,oswctlcomd,readstcomd,readeepromcomd;
extern uchar oswst1,oswst2;
extern uchar adcount;
extern uchar switchover;
extern uchar swtime;
extern uchar flag;
extern uchar exitst;

void d_ms(u16 time)
{    
	u16 i=0;  
	while(time--)
	{
		i=12000;  
		while(i--) ;    
	}
}

void EXTI15_10_IRQHandler(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	if (EXTI_GetITStatus(EXTI_Line13) != RESET) 
		{
			NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
			NVIC_Init(&NVIC_InitStructure);
			d_ms(100);
			oswctlcomd=1;
			if (exitst==0){exitst=1;}
			else {exitst=0;}
			if (exitst==0){rbyte2=0x00;rbyte3=0x00;oswst1=0x00;oswst2=0x00;}
			else {rbyte2=0xff;rbyte3=0xff;oswst1=0xff;oswst2=0xff;}
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
			EXTI_ClearITPendingBit(EXTI_Line13);
	}
}

void DEBUG_USART_IRQHandler(void)
{
	uint8_t tem;
	if (USART_GetITStatus(DEBUG_USARTx,USART_IT_RXNE)!=RESET) {
	tem = USART_ReceiveData( DEBUG_USARTx );
	if (rcount==0)
	{
		rbyte1=tem;rcount=1;
		if(rbyte1==0x00)//sn:00
		{comdok=1;}
		else
		{comdok=0;}
	}
	else if(rcount==1)   //2nd:12-5-osw
	{rbyte2=tem;rcount=2;} //2nd:12-5-osw
	else if(rcount==2)	  //3st:4-1-osw
	{rbyte3=tem;rcount=3;} //3st:4-1-osw 	
	else if(rcount==3)	  //4th:pluse width
	{basictime=tem;rcount=4;}  //4th:pluse width			
	else if(rcount==4)	  //4th:pluse width
	{if((tem==0x55)&& comdok)  //save
	{eepromupdate=1;oswctlcomd=0;readstcomd=0;readeepromcomd=0;rcount=0;}//save
	else if((tem==0xaa)&& comdok)  //ctl
	{eepromupdate=0;oswctlcomd=1;readstcomd=0;readeepromcomd=0;rcount=0;oswst1=rbyte2;oswst2=rbyte3;}  // ctl
	else if((tem==0xbb)&& comdok)  //rdst
	{eepromupdate=0;oswctlcomd=0;readstcomd=1;readeepromcomd=0;rcount=0;}  // rdst
	else if((tem==0xee)&& comdok)  //rdeeprom
	{eepromupdate=0;oswctlcomd=0;readstcomd=0;readeepromcomd=1;rcount=0;}  // rdeeprom
	else rcount=0;
	} //4th:pluse width	
			   else ;
}
	}
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */

void open_all(void)
{
	GPIO_SetBits(IN1N);	
	GPIO_SetBits(IN2N);	
	GPIO_SetBits(IN3N);	
	GPIO_SetBits(IN4N);	
	GPIO_SetBits(IN5N);	
	GPIO_SetBits(IN6N);	
	GPIO_SetBits(IN7N);	
	GPIO_SetBits(IN8N);	
	GPIO_SetBits(IN9N);	
	GPIO_SetBits(IN10N);	
	GPIO_SetBits(IN11N);	
	GPIO_SetBits(IN12N);	
	GPIO_SetBits(IN1P);	
	GPIO_SetBits(IN2P);	
	GPIO_SetBits(IN3P);	
	GPIO_SetBits(IN4P);	
	GPIO_SetBits(IN5P);	
	GPIO_SetBits(IN6P);	
	GPIO_SetBits(IN7P);	
	GPIO_SetBits(IN8P);	
	GPIO_SetBits(IN9P);	
	GPIO_SetBits(IN10P);	
	GPIO_SetBits(IN11P);	
	GPIO_SetBits(IN12P);	
}

void close_all(void)
{
	GPIO_ResetBits(IN1N);	
	GPIO_ResetBits(IN2N);	
	GPIO_ResetBits(IN3N);	
	GPIO_ResetBits(IN4N);	
	GPIO_ResetBits(IN5N);	
	GPIO_ResetBits(IN6N);	
	GPIO_ResetBits(IN7N);	
	GPIO_ResetBits(IN8N);	
	GPIO_ResetBits(IN9N);	
	GPIO_ResetBits(IN10N);	
	GPIO_ResetBits(IN11N);	
	GPIO_ResetBits(IN12N);	
	GPIO_ResetBits(IN1P);	
	GPIO_ResetBits(IN2P);	
	GPIO_ResetBits(IN3P);	
	GPIO_ResetBits(IN4P);	
	GPIO_ResetBits(IN5P);	
	GPIO_ResetBits(IN6P);	
	GPIO_ResetBits(IN7P);	
	GPIO_ResetBits(IN8P);	
	GPIO_ResetBits(IN9P);	
	GPIO_ResetBits(IN10P);	
	GPIO_ResetBits(IN11P);	
	GPIO_ResetBits(IN12P);	
}

void BASIC_TIM_IRQHandler(void)
{
	if ( TIM_GetITStatus( TIM2, TIM_IT_Update) != RESET ) 
	{
		adcount++;
		if(adcount==4)
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_12);
		}
		else if (adcount==3)
		{
			TIM_Cmd(TIM2, ENABLE);
			GPIO_SetBits(GPIOA, GPIO_Pin_11);
		}
		else if (adcount==basictime)
		{
			TIM_Cmd(TIM2, DISABLE);
			adcount=0;
			GPIO_SetBits(GPIOA, GPIO_Pin_11);
			open_all();
			open_all();
			open_all();
			GPIO_ResetBits(GPIOA, GPIO_Pin_12);
			swtime=swtime+1;
			switchover=1;
			GPIO_ResetBits(GPIOA, GPIO_Pin_11);
		}
		else;
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
	}
}
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
