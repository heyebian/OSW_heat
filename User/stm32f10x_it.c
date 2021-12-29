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
extern uchar rbyte1,rbyte2,rbyte3,rbyte4,comdok;
extern uchar basictime;
extern uchar oswctlcomd,readstcomd,readbitcomd,readoswcmd,readtem;
extern uchar oswst1,oswst2;
extern uchar adcount;
extern uchar switchover;
extern uchar swtime;
extern uchar panel_number;
extern uchar temp[3];
extern uchar oswctr1,oswctr2,oswctr3,oswctr4,oswctr5,oswctr6,oswctr7,oswctr8,oswctr9,oswctr10,oswctr11,oswctr12;
extern uchar sbyte1,sbyte2,sbyte3;
extern uchar heat_time;
extern uint16_t ADC_ConvertedValue;
extern uint16_t ADC_summer_0_14,ADC_summer_15_29;
extern uchar adc_counter;
uchar now_st,next_st;
uint16_t start_heat;
extern uint16_t adc_5value_sum;
extern uchar adc_num;
extern uint16_t adc_5value_aver;

void d_ms(u16 time)
{    
	u16 i=0;  
	while(time--)
	{
		i=1000;  
		while(i--) ;    
	}
}

void Usart_SendByte_IN( USART_TypeDef * pUSARTx, uchar ch)
{

	USART_SendData(pUSARTx,ch);

	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

void command_process_IN()
{
	if(switchover==1)
	{if((rbyte2 & 0x08)==0)	 //12
        oswctr12=0;
	else oswctr12=1;
	if((rbyte2 & 0x04)==0)//11
        oswctr11=0;
	else oswctr11=1;
	if((rbyte2 & 0x02)==0)//10
        oswctr10=0;
	else oswctr10=1;
	if((rbyte2 & 0x01)==0)//9
        oswctr9=0;
	else oswctr9=1;
	if((rbyte3 & 0x80)==0)//8
        oswctr8=0;
	else oswctr8=1;
	if((rbyte3 & 0x40)==0)//7
        oswctr7=0;
	else oswctr7=1;
	if((rbyte3 & 0x20)==0)//6
        oswctr6=0;
	else oswctr6=1;
	if((rbyte3 & 0x10)==0)//5
        oswctr5=0;
	else oswctr5=1;
	if((rbyte3 & 0x08)==0)//4
        oswctr4=0;
	else oswctr4=1;
	if((rbyte3 & 0x04)==0)//3
        oswctr3=0;
	else oswctr3=1;
	if((rbyte3 & 0x02)==0)//2
        oswctr2=0;
	else oswctr2=1;
	if((rbyte3 & 0x01)==0)//1
        oswctr1=0;
	else oswctr1=1;

	if(oswctr12)  //no.12
	{GPIO_SetBits(IN12P);}
	else  //no.12
	{GPIO_SetBits(IN12N);}
	if(oswctr11)  //no.11
	{GPIO_SetBits(IN11P);}
	else  //no.11
	{GPIO_SetBits(IN11N);}
	if(oswctr10)  //no.10
	{GPIO_SetBits(IN10P);}
	else  //no.10
	{GPIO_SetBits(IN10N);}
	if(oswctr9)  //no.9
	{GPIO_SetBits(IN9P);}
	else  //no.9
	{GPIO_SetBits(IN9N);}
	if(oswctr8)  //no.8
	{GPIO_SetBits(IN8P);}
	else  //no.8
	{GPIO_SetBits(IN8N);}
	if(oswctr7)  //no.7
	{GPIO_SetBits(IN7P);}
	else  //no.7
	{GPIO_SetBits(IN7N);}
	if(oswctr6)  //no.6
	{GPIO_SetBits(IN6P);}
	else  //no.6
	{GPIO_SetBits(IN6N);}
	if(oswctr5)  //no.5
	{GPIO_SetBits(IN5P);}
	else  //no.5
	{GPIO_SetBits(IN5N);}
	if(oswctr4)  //no.4
	{GPIO_SetBits(IN4P);}
	else  //no.4
	{GPIO_SetBits(IN4N);}
	if(oswctr3)  //no.3
	{GPIO_SetBits(IN3P);}
	else  //no.3
	{GPIO_SetBits(IN3N);}
	if(oswctr2)  //no.2
	{GPIO_SetBits(IN2P);}
	else  //no.2
	{GPIO_SetBits(IN2N);}
	if(oswctr1)  //no.1
	{GPIO_SetBits(IN1P);}
	else  //no.1
	{GPIO_SetBits(IN1N);}
	
	GPIO_SetBits(GPIOA, GPIO_Pin_11);
	//GPIO_SetBits(GPIOA, GPIO_Pin_12);
	TIM_Cmd(TIM2, ENABLE);
	
	switchover=0;
  }
	else;
	oswctlcomd=0;
 }

//EXTI
//Add the state for each entry
void EXTI15_10_IRQHandler(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	if (EXTI_GetITStatus(EXTI_Line12) != RESET) 
		{
			//Close the EXIT to avoid the shake
			EXTI_InitStructure.EXTI_Line = EXTI_Line12;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			EXTI_InitStructure.EXTI_LineCmd = DISABLE;
			EXTI_Init(&EXTI_InitStructure);
			d_ms(100);
			
			oswctlcomd=1;
			now_st = oswst2 & 0x3f; //get the current state
			if (now_st == 63) //The maxium is 63
			{next_st = 0;}
			else {next_st = now_st+1;}
			//According to bit state, get every switch state
			rbyte2=(next_st & 0x3C)>>2;
			rbyte3=(next_st & 0x3F)|(next_st << 6);
			//Changed the state
			if(oswctlcomd==1)
			{command_process_IN();}
			//Open the EXIT
			EXTI_InitStructure.EXTI_Line = EXTI_Line12;
			EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);
			EXTI_ClearITPendingBit(EXTI_Line12);
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
		if(rbyte1==panel_number)//Judge whether the pannel number is true
		{comdok=1;}
		else
		{comdok=0;}
	}
	else if(rcount==1)   
	{rbyte2=tem;rcount=2;} 
	else if(rcount==2)	  
	{rbyte3=tem;rcount=3;} 
	else if(rcount==3)	  //4th:check_bit
	{rbyte4=tem;rcount=4;}  //4th:check_bit		
	else if(rcount==4)
	{
	/*
	//Xcom
	if((tem==0x00)&& comdok && (rbyte4==0x05))  //ctl_bit
	{rbyte2=(rbyte3 & 0x3C)>>2;rbyte3=(rbyte3 & 0x3F)|(rbyte3 << 6);
	oswctlcomd=1;readstcomd=0;readbitcomd=0;readoswcmd=0;rcount=0;}
	else if((tem==0x11)&& comdok &&(rbyte4 ==0x05))  //read_bit
	{oswctlcomd=0;readstcomd=0;readbitcomd=1;readoswcmd=0;rcount=0;}
	else if((tem==0x33)&& comdok &&(rbyte4 ==0x05))  //ctl_single
	{oswctlcomd=1;readstcomd=0;readbitcomd=0;readoswcmd=0;rcount=0;}
	else if((tem==0x44)&& comdok &&(rbyte4 ==0x05))  //read_single
	{oswctlcomd=0;readstcomd=1;readbitcomd=0;readoswcmd=0;rcount=0;}
	//OSW_12
	else if((tem==0x55)&& comdok &&(rbyte4 ==0x05))  //save
	{rcount=0;}
	else if((tem==0xaa)&& comdok &&(rbyte4 ==0x05))  //ctl_single
	{rbyte3=(rbyte3&0x0F)|((rbyte2&0x0F)<<4);rbyte2=rbyte2>>4;
	oswctlcomd=1;readstcomd=0;readbitcomd=0;readoswcmd=0;rcount=0;}
	else if((tem==0xbb)&& comdok &&(rbyte4 ==0x05))  //read_single
	{oswctlcomd=0;readstcomd=0;readbitcomd=0;readoswcmd=1;rcount=0;}
	else if((tem==0xee)&& comdok &&(rbyte4 ==0x05))  //read_single
	{oswctlcomd=0;readstcomd=0;readbitcomd=0;readoswcmd=1;rcount=0;}
	
	else rcount=0;
	*/
	
	if((tem==0x00)&& comdok)  //ctl_bit
	{rbyte2=(rbyte3 & 0x3C)>>2;rbyte3=(rbyte3 & 0x3F)|(rbyte3 << 6);
	oswctlcomd=1;readstcomd=0;readbitcomd=0;readoswcmd=0;rcount=0;}
	else if((tem==0x11)&& comdok)  //read_bit
	{oswctlcomd=0;readstcomd=0;readbitcomd=1;readoswcmd=0;rcount=0;}
	else if((tem==0x33)&& comdok)  //ctl_single
	{oswctlcomd=1;readstcomd=0;readbitcomd=0;readoswcmd=0;rcount=0;}
	else if((tem==0x44)&& comdok)  //read_single
	{oswctlcomd=0;readstcomd=1;readbitcomd=0;readoswcmd=0;rcount=0;}
	//OSW_12
	else if((tem==0x55)&& comdok)  //save
	{rcount=0;}
	else if((tem==0xaa)&& comdok)  //ctl_single
	{rbyte3=(rbyte3&0x0F)|((rbyte2&0x0F)<<4);rbyte2=rbyte2>>4;
	oswctlcomd=1;readstcomd=0;readbitcomd=0;readoswcmd=0;rcount=0;}
	else if((tem==0xbb)&& comdok)  //read_single
	{oswctlcomd=0;readstcomd=0;readbitcomd=0;readoswcmd=1;rcount=0;}
	else if((tem==0xee)&& comdok)  //read_single
	{oswctlcomd=0;readstcomd=0;readbitcomd=0;readoswcmd=1;rcount=0;}
	else if((tem==0xdd)&& comdok)  //read_temperature
	{readtem=1;rcount=0;}
	
	else rcount=0;
	
	if(oswctlcomd==1)
	{command_process_IN();}

	else if(readstcomd==1)
	{
		sbyte1=panel_number;sbyte2=oswst1;sbyte3=oswst2;
		Usart_SendByte_IN(DEBUG_USARTx,sbyte1);
		Usart_SendByte_IN(DEBUG_USARTx,sbyte2);
		Usart_SendByte_IN(DEBUG_USARTx,sbyte3);
		Usart_SendByte_IN(DEBUG_USARTx,0x05);
		readstcomd=0;
	} 
	else if(readbitcomd==1)
	{
		sbyte1=panel_number;sbyte2=oswst1;sbyte3=oswst2&0x3F;
		Usart_SendByte_IN(DEBUG_USARTx,sbyte1);
		Usart_SendByte_IN(DEBUG_USARTx,sbyte3);
		Usart_SendByte_IN(DEBUG_USARTx,0x05);
		readbitcomd=0;
	} 
	else if(readoswcmd==1)
	{
		sbyte1=panel_number;
		sbyte2=((oswst1&0x0F)<<4)|((oswst2&0xF0)>>4);sbyte3=oswst2&0x0F;
		Usart_SendByte_IN(DEBUG_USARTx,sbyte1);
		Usart_SendByte_IN(DEBUG_USARTx,sbyte2);
		Usart_SendByte_IN(DEBUG_USARTx,sbyte3);
		Usart_SendByte_IN(DEBUG_USARTx,0x05);
		readoswcmd=0;
	} 
	else if(readtem==1)
	{
		sbyte1=ADC_ConvertedValue>>8;
		sbyte2=ADC_ConvertedValue;
		Usart_SendByte_IN(DEBUG_USARTx,sbyte1);
		Usart_SendByte_IN(DEBUG_USARTx,sbyte2);
		readoswcmd=0;
	} 
	
	
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

void close_all(void)
{
	if (1) {
			GPIO_ResetBits(IN12N);	
			GPIO_ResetBits(IN12P);	
	}
	if (1) {
			GPIO_ResetBits(IN11N);	
			GPIO_ResetBits(IN11P);	
	}
	if (1) {
			GPIO_ResetBits(IN10N);	
			GPIO_ResetBits(IN10P);	
	}
	if (1) {
			GPIO_ResetBits(IN9N);	
			GPIO_ResetBits(IN9P);	
	}
	if (1) {
			GPIO_ResetBits(IN8N);	
			GPIO_ResetBits(IN8P);	
	}
	if (1) {
			GPIO_ResetBits(IN7N);	
			GPIO_ResetBits(IN7P);	
	}
	if (1) {
			GPIO_ResetBits(IN6N);	
			GPIO_ResetBits(IN6P);	
	}
	if (1) {
			GPIO_ResetBits(IN5N);	
			GPIO_ResetBits(IN5P);	
	}
	if (1) {
			GPIO_ResetBits(IN4N);	
			GPIO_ResetBits(IN4P);	
	}
	if (1) {
			GPIO_ResetBits(IN3N);	
			GPIO_ResetBits(IN3P);	
	}
	if (1) {
			GPIO_ResetBits(IN2N);	
			GPIO_ResetBits(IN2P);	
	}
	if (1) {
			GPIO_ResetBits(IN1N);	
			GPIO_ResetBits(IN1P);	
	}

}

void FLASH_WriteByte_1(uint32_t addr , uchar *p)
{
	uint32_t HalfWord;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	FLASH_ErasePage(addr);
	HalfWord=*(p++);
	HalfWord|=*(p++)<<8;
	FLASH_ProgramHalfWord(addr, HalfWord);
	FLASH_Lock();
}

void BASIC_TIM_IRQHandler(void)
{
	if ( TIM_GetITStatus( TIM2, TIM_IT_Update) != RESET ) 
	{
		
		adcount++;
		if (adcount==basictime)
		{
			TIM_Cmd(TIM2, DISABLE);
			adcount=0;
			close_all();
			GPIO_ResetBits(GPIOA, GPIO_Pin_11);
			//GPIO_ResetBits(GPIOA, GPIO_Pin_12);
			swtime=swtime+1;
			switchover=1;
			GPIO_ResetBits(GPIOA, GPIO_Pin_11);
			oswst1=rbyte2;
			oswst2=rbyte3;
			temp[0]=oswst1;
			temp[1]=oswst2;
			FLASH_WriteByte_1(((uint32_t)0x08009000),temp);
		}
		else;
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);

	}
}

void TIM3_IRQHandler(void)
{
	if ( TIM_GetITStatus( TIM3, TIM_IT_Update) != RESET ) 
	{

	if(switchover==1)
	{
	GPIO_SetBits(GPIOA, GPIO_Pin_11);
	if((oswst1 & 0x08)==0)	 //12
        oswctr12=0;
	else oswctr12=1;		
	if((oswst1 & 0x04)==0)//11
        oswctr11=0;
	else oswctr11=1;
	if((oswst1 & 0x02)==0)//10
        oswctr10=0;
	else oswctr10=1;
	if((oswst1 & 0x01)==0)//9
        oswctr9=0;
	else oswctr9=1;
	if((oswst2 & 0x80)==0)//8
        oswctr8=0;
	else oswctr8=1;
	if((oswst2 & 0x40)==0)//7
        oswctr7=0;
	else oswctr7=1;
	if((oswst2 & 0x20)==0)//6
        oswctr6=0;
	else oswctr6=1;
	if((oswst2 & 0x10)==0)//5
        oswctr5=0;
	else oswctr5=1;
	if((oswst2 & 0x08)==0)//4
        oswctr4=0;
	else oswctr4=1;
	if((oswst2 & 0x04)==0)//3
        oswctr3=0;
	else oswctr3=1;
	if((oswst2 & 0x02)==0)//2
        oswctr2=0;
	else oswctr2=1;
	if((oswst2 & 0x01)==0)//1
        oswctr1=0;
	else oswctr1=1;

	if(oswctr12)  //no.12
	{GPIO_SetBits(IN12P);GPIO_ResetBits(IN12N);}
	else  //no.12
	{GPIO_SetBits(IN12N);GPIO_ResetBits(IN12P);}
	if(oswctr11)  //no.11
	{GPIO_SetBits(IN11P);GPIO_ResetBits(IN11N);}
	else  //no.11
	{GPIO_SetBits(IN11N);GPIO_ResetBits(IN11P);}
	if(oswctr10)  //no.10
	{GPIO_SetBits(IN10P);GPIO_ResetBits(IN10N);}
	else  //no.10
	{GPIO_SetBits(IN10N);GPIO_ResetBits(IN10P);}
	if(oswctr9)  //no.9
	{GPIO_SetBits(IN9P);GPIO_ResetBits(IN9N);}
	else  //no.9
	{GPIO_SetBits(IN9N);GPIO_ResetBits(IN9P);}
	if(oswctr8)  //no.8
	{GPIO_SetBits(IN8P);GPIO_ResetBits(IN8N);}
	else  //no.8
	{GPIO_SetBits(IN8N);GPIO_ResetBits(IN8P);}
	if(oswctr7)  //no.7
	{GPIO_SetBits(IN7P);GPIO_ResetBits(IN7N);}
	else  //no.7
	{GPIO_SetBits(IN7N);GPIO_ResetBits(IN7P);}
	if(oswctr6)  //no.6
	{GPIO_SetBits(IN6P);GPIO_ResetBits(IN6N);}
	else  //no.6
	{GPIO_SetBits(IN6N);GPIO_ResetBits(IN6P);}
	if(oswctr5)  //no.5
	{GPIO_SetBits(IN5P);GPIO_ResetBits(IN5N);}
	else  //no.5
	{GPIO_SetBits(IN5N);GPIO_ResetBits(IN5P);}
	if(oswctr4)  //no.4
	{GPIO_SetBits(IN4P);GPIO_ResetBits(IN4N);}
	else  //no.4
	{GPIO_SetBits(IN4N);GPIO_ResetBits(IN4P);}
	if(oswctr3)  //no.3
	{GPIO_SetBits(IN3P);GPIO_ResetBits(IN3N);}
	else  //no.3
	{GPIO_SetBits(IN3N);GPIO_ResetBits(IN3P);}
	if(oswctr2)  //no.2
	{GPIO_SetBits(IN2P);GPIO_ResetBits(IN2N);}
	else  //no.2
	{GPIO_SetBits(IN2N);GPIO_ResetBits(IN2P);}
	if(oswctr1)  //no.1
	{GPIO_SetBits(IN1P);GPIO_ResetBits(IN1N);}
	else  //no.1
	{GPIO_SetBits(IN1N);GPIO_ResetBits(IN1P);}

	TIM_Cmd(TIM4, ENABLE);
	
 }
 

	TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);

	}
}

void TIM4_IRQHandler(void)
{
	TIM_Cmd(TIM4, DISABLE); 
	close_all();
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
}

void ADC1_2_IRQHandler(void)
{
	if (ADC_GetITStatus(ADC1,ADC_IT_EOC)==SET) {
		ADC_ConvertedValue = ADC_GetConversionValue(ADC1);
		if (adc_counter < 30)
		{
			// compute the ambient temperature
			if (adc_counter == 2)
			{
				start_heat = ADC_summer_0_14 / 2;
				if (start_heat > 1850){TIM_Cmd(TIM3, ENABLE);}
			}
			if (adc_counter < 15)
			{ADC_summer_0_14 += ADC_ConvertedValue;}
			else
			{ADC_summer_15_29 += ADC_ConvertedValue;}
			adc_counter += 1;
			sbyte1=ADC_ConvertedValue>>8;
			sbyte2=ADC_ConvertedValue;
			Usart_SendByte_IN(DEBUG_USARTx,sbyte1);
			Usart_SendByte_IN(DEBUG_USARTx,sbyte2);
		}
		if (adc_counter > 30)
		{
			//compute the current temperature
			adc_5value_sum += ADC_ConvertedValue;
			adc_num += 1;
			if (adc_num == 5)
			{
				adc_5value_aver = adc_5value_sum / 5;
				adc_5value_sum = 0;
				adc_num =0;
			}
		}
		ADC_Cmd(ADC1, DISABLE);
	}
	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
}
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
