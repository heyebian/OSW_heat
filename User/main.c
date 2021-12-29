#include "stm32f10x.h"
#include "my_own.h"

//Modified in 12/28
//Apply to STM32F103CBU6
#define WRITE_START_ADDR ((uint32_t)0x08009000)
#define WRITE_END_ADDR ((uint32_t)0x0800C000)

uchar oswctlcomd,readstcomd,readbitcomd,readoswcmd,switchover,readtem; //Command

uchar oswctr1,oswctr2,oswctr3,oswctr4,oswctr5,oswctr6,oswctr7,oswctr8,oswctr9,oswctr10,oswctr11,oswctr12; //Control every single switch
uchar rcount; //Receive words' number
uchar rbyte1,rbyte2,rbyte3,rbyte4; //same to the readme
uchar sbyte1,sbyte2,sbyte3;
uchar oswst1,oswst2;
uchar adcount;
uchar swtime;
uchar basictime;
uchar comdok;
uchar temp[3]={0}; //flash temp
uchar panel_number;
uchar xcom_flag;//judge the command is from the XCOM or the SW-12 programm
uchar heat_time;//save the current heat time
uint16_t ADC_ConvertedValue; //save the ADC_converted value
uchar add_flag;
uchar heat_time_new; //Judge whether the heat time should be changed
uchar adc_counter; //Cal the first 30 adc_converted process
uchar heat_time_start; //Save the first heat time 
uint16_t ADC_summer_0_14,ADC_summer_15_29; //to cal the ambient temperature
uint16_t adc_5value_sum; //to cal the current temperature
uint16_t adc_5value_aver;
uchar adc_num; //every 5 adc results' aver as the current adc value
uchar heat_time_max,heat_time_min;

//Set the main frequency as 64M
//Without the out crystal oscillator
//Using the RC oscillator
void RCC_Configuration(void)
{
	__IO uint32_t HSIStartUpStatus = 0;
	
	RCC_DeInit();		
	RCC_HSICmd(ENABLE);
	HSIStartUpStatus = RCC->CR & RCC_CR_HSIRDY;
	
	if (HSIStartUpStatus == RCC_CR_HSIRDY) {
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
		RCC_PLLCmd(ENABLE);
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {}
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while (RCC_GetSYSCLKSource() != 0x08) {}
}
}

//Set the basic TIM config
void BASIC_TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//The single switch pulse
	//The full pulse = Single Pulse * basictime
	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Period=20;//20us
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64M/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM2, DISABLE);
	
	//The heating cycle
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseStructure.TIM_Period=2000;//2ms
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM3, DISABLE);
		
	//The heat time
	TIM_DeInit(TIM4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_InternalClockConfig(TIM4);
	TIM_TimeBaseStructure.TIM_Period=0x48;//20us
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM4, DISABLE);
	
}

//Write Halfword(2 bytes) to the Flash
void FLASH_WriteByte(uint32_t addr , uchar *p)
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

//Read Halfword from the Flash
void FLASH_ReadByte(uint32_t addr , uchar *p )
{
	uint8_t n;

	n=2;
	while(n--)
	{*(p++)=*((uchar*)addr++);}
}

//Set the NVIC config
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //Choose the Group 2(2 bit PreemptionPriority and 2 bit SubPriority)
	
  //Set the USART order
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	//Set the TIM2 order
	NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQ ;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	//Set the TIM3 order
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn ;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	//Set the TIM4 order
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	//Set the ADC order
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
	
	//Set the EXIT order
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;

	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}


//Set the EXTI config
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO),ENABLE);

	NVIC_Configuration();
	
	//GPIOA 12 as the EXTI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,	GPIO_PinSource12);
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}


//Set the USART config
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;


	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);


	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);


	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;

	USART_InitStructure.USART_StopBits = USART_StopBits_1;

	USART_InitStructure.USART_Parity = USART_Parity_No ;

	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(DEBUG_USARTx, &USART_InitStructure);

	NVIC_Configuration();

	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	USART_Cmd(DEBUG_USARTx, ENABLE);
}

//Set the ADC config
void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

	ADC_InitStructure.ADC_ScanConvMode = DISABLE ;

	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;

	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	
	ADC_TempSensorVrefintCmd(ENABLE);

	ADC_Cmd(ADC1, ENABLE);

	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


//Sent Byte to the Usart
void Usart_SendByte( USART_TypeDef * pUSARTx, uchar ch)
{

	USART_SendData(pUSARTx,ch);

	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}


//Set the GPIO config
void GPIO_CONFIG(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	//SW1
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//SW2
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//SW3
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//SW4
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//SW5
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SW6
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SW7
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SW8
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SW9
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SW10
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//SW11
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//SW12
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//ENA
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	PWR_BackupAccessCmd(ENABLE);
	RCC_LSEConfig(RCC_LSE_OFF);
	BKP_TamperPinCmd(DISABLE);
	BKP_ITConfig(DISABLE); 
	PWR_BackupAccessCmd(DISABLE);

	
}

void delay_ms(u16 time)
{    
	u16 i=0;  
	while(time--)
	{
		i=3600;  
		while(i--) ;    
	}
}

void delay_us(u16 time)
{    
	u16 i=0;  
	while(time--)
	{
		i=14;  
		while(i--) ;    
	}
}


//Reset the TIM4 (heat time)
void TIM4_redo(uchar add_f, uchar heat_t)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIM4);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		TIM_InternalClockConfig(TIM4);
		TIM_TimeBaseStructure.TIM_Period=(add_f-0x05)*10+heat_t;//20us
		TIM_TimeBaseStructure.TIM_Prescaler= 63;//64/64
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
		TIM_ARRPreloadConfig(TIM4, DISABLE);
		Usart_SendByte(DEBUG_USARTx,heat_t);
}
 
void init(void)
{
	panel_number = 0x00;
	RCC_Configuration();
	GPIO_CONFIG();
	BASIC_TIM_Config();
	EXTI_Key_Config();
	USART_Config();
	ADC_Config();
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	oswctlcomd=0;readstcomd=0;readbitcomd=0;readtem=0;
	switchover=1;swtime=0;
	rcount=0;comdok=0;    
	adcount=0; basictime=10;
	FLASH_ReadByte(WRITE_START_ADDR,temp);
	oswst1=temp[0];
	oswst2=temp[1];
	ADC_Cmd(ADC1, DISABLE);
	TIM_Cmd(TIM3, DISABLE);
	heat_time = 0x20;
	add_flag = 0x05;
	rbyte4 = 0x05;
	adc_counter = 0x00;
	ADC_summer_0_14 = 0;
	ADC_summer_15_29 = 0;
	adc_num = 0;
	adc_5value_sum = 0;
	adc_5value_aver = 0;
}
	
int main(void)
{
	init();
	
	while(1)
	{
		ADC_Cmd(ADC1, ENABLE);
		delay_ms(2000);
		if (adc_counter == 30)
		{
			ADC_summer_0_14 /= 15;
			ADC_summer_15_29 /= 15;
			ADC_summer_0_14 = (ADC_summer_0_14+ADC_summer_15_29) / 2;
			adc_5value_aver = ADC_summer_0_14;
			
			sbyte1=ADC_summer_0_14>>8;
			sbyte2=ADC_summer_0_14;
			Usart_SendByte(DEBUG_USARTx,sbyte1);
			Usart_SendByte(DEBUG_USARTx,sbyte2);
			//compute the ambient temperature

			if (ADC_summer_0_14 > 2080) //-40
			{heat_time_max = 0x98;heat_time_min = 0x7A;} //max:120us min:90us
			else if (ADC_summer_0_14 > 2070) //-35
			{heat_time_max = 0x8E;heat_time_min = 0x70;} //max:110us min:80us
			else if (ADC_summer_0_14 > 2040) //-30
			{heat_time_max = 0x8E;heat_time_min = 0x70;} //max:110us min:80us
			else if (ADC_summer_0_14 > 2010) //-25
			{heat_time_max = 0x84;heat_time_min = 0x66;} //max:100us min:70us
			else if (ADC_summer_0_14 > 1990) //-20
			{heat_time_max = 0x84;heat_time_min = 0x66;} //max:100us min:70us
			else if (ADC_summer_0_14 > 1950) //-15
			{heat_time_max = 0x7A;heat_time_min = 0x5C;} //max:90us min:60us
			else if (ADC_summer_0_14 > 1920) //-10
			{heat_time_max = 0x70;heat_time_min = 0x5C;} //max:80us min:60us
			else if (ADC_summer_0_14 > 1900) //-5
			{heat_time_max = 0x66;heat_time_min = 0x52;} //max:70us min:50us
			else if (ADC_summer_0_14 > 1880) //0
			{heat_time_max = 0x5C;heat_time_min = 0x48;} //max:60us min:40us
			else if (ADC_summer_0_14 > 1820) //15
			{heat_time_max = 0x52;heat_time_min = 0x48;} //max:50us min:40us
			else if (ADC_summer_0_14 > 1760) //25
			{heat_time_max = 0x48;heat_time_min = 0x3E;} //max:40us min:30us
			else
			{heat_time_max = 0x20;heat_time_min = 0x20;}

			heat_time_start = heat_time_max;
			heat_time = heat_time_max;
			TIM4_redo(add_flag,heat_time);
			adc_counter += 1;
			TIM_Cmd(TIM3,ENABLE);
		}
		else if (adc_counter > 30)
		{
			//according the current temperature adjust the heat time
			if (adc_5value_aver < 1760)
			{heat_time_new = 0x20;}
			else if (adc_5value_aver < 1820)
			{heat_time_new = 0x3E;}
			else if (adc_5value_aver < 1850)
			{heat_time_new = 0x48;}
			else if (adc_5value_aver < 1870)
			{heat_time_new = heat_time_min;}
			else if (adc_5value_aver >= 1870)
			{heat_time_new = heat_time_max;}
			
			if ((add_flag != rbyte4)|(heat_time != heat_time_new))
			{add_flag = rbyte4;heat_time = heat_time_new;
			TIM4_redo(add_flag,heat_time);}
		}
		

	}
	return 0;
}
