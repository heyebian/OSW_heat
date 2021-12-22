#include "stm32f10x.h"
#include "my_own.h"

//������STM32F103CBU6
//�޸���2021/11/20���������¶ȴ��������ƹ��ܣ�
//�޸���2021/11/16��������heat���ܣ�δ�����ԣ�
//�޸���2021/11/1�����ڲ�8M��Ƶ��64M��
//�����ڶ��ͨ�Ű汾
//��������final��/������
//д�����ʼ��ַ�ͼ�����ַ
//����ʱ���͵���0
//ƽ��ʱ���͵���1
#define WRITE_START_ADDR ((uint32_t)0x08009000)
#define WRITE_END_ADDR ((uint32_t)0x0800C000)

uchar oswctlcomd,readstcomd,readbitcomd,readoswcmd,switchover,readtem;//����

//�жϺ�Ŀ�����
uchar oswctr1,oswctr2,oswctr3,oswctr4,oswctr5,oswctr6,oswctr7,oswctr8,oswctr9,oswctr10,oswctr11,oswctr12;
uchar rcount;//���������ж�
uchar rbyte1,rbyte2,rbyte3,rbyte4;//1:��� 2:12-5 3:4-1 4:CtlCmd
uchar sbyte1,sbyte2,sbyte3;//same
uchar oswst1,oswst2;
uchar adcount;
uchar swtime;//�л�״̬����
uchar basictime;//���峤��
uchar comdok;
uchar temp[3]={0};//flash�ݴ�
uchar panel_number;//���ӱ��
uchar xcom_flag;//�����ʽ���ƣ������xcomΪ1����λ����Ϊ0
uchar heat_time;
uint16_t ADC_ConvertedValue;
uchar add_flag;
uchar heat_time_new;
uchar adc_counter;

//����Ƶ��Ϊ64M
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

//û����Ӿ������Բ��õ����ڲ�����8M������Ƶ�õ���64M
void BASIC_TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//�л�����20*10 = 200us
	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//����ʱ��
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Period=20;//20us
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64M/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM2, DISABLE);
	
	//��������2000ms
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//����ʱ��
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseStructure.TIM_Period=2000;//2ms
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM3, DISABLE);
		
	//��������20us(ʵ�ʳ��Ȳ�һ��)
	TIM_DeInit(TIM4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//����ʱ��
	TIM_InternalClockConfig(TIM4);
	TIM_TimeBaseStructure.TIM_Period=53;//20us
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM4, DISABLE);
	
}

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

void FLASH_ReadByte(uint32_t addr , uchar *p )
{
	uint8_t n;
	
	n=2;
	while(n--)
	{*(p++)=*((uchar*)addr++);}
}

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//Ƕ���жϿ�������ѡ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//����USARTΪ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
	//�������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);

	//���ö�ʱ��2Ϊ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQ ;
	//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	//���ö�ʱ��3Ϊ�ж�Դ
	//����Heat
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn ;
	//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	//���ö�ʱ��4Ϊ�ж�Դ
	//����Heat
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;
	//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	//����ADCΪ�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	//��ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	//�����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	/*
	//�����ж�Դ
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	//��ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//ʹ���ж�
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStructure);
	*/
}


//�ⲿ�ж�����
/*
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	//ʱ������
	RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO),ENABLE);
	//NVIC����
	NVIC_Configuration();
	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//�ж�����
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,	GPIO_PinSource13);
	EXTI_InitStructure.EXTI_Line = EXTI_Line13;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
*/

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	//�򿪴���GPIO��ʱ��
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

	//�򿪴��������ʱ��
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	//��USART Tx����Ϊ���츴��
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	//��USART Rx������������
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	//���ô��ڵĹ�������
	//���ò�����
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	//���������ֳ�
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//����ֹͣλ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//����У��λ
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	//����Ӳ��������
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	//���ù���ģʽ���շ�һ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//USART_InitStructure.USART_Mode = USART_Mode_Rx ;
	//��ɴ��ڳ�ʼ��
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	//�������ȼ�����
	NVIC_Configuration();
	//ʹ�ܴ��ڽ����ж�
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	//ʹ�ܴ���
	USART_Cmd(DEBUG_USARTx, ENABLE);
}

void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	//��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	//ֻʹ��һ��ADC,����ģʽ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	//����ɨ��ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ;
	//����ת��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//�����ⲿ����
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//�ұ߶���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//����ת��ͨ������
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	//��ʼ��ADC
	ADC_Init(ADC1, &ADC_InitStructure);
	//����8�ַ�Ƶ����64/8=8Mhz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	//���ò�������239.5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);
	//�����ж�
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	
	ADC_TempSensorVrefintCmd(ENABLE);
	//����ADC
	ADC_Cmd(ADC1, ENABLE);
	//У׼
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	//��������
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


//���͵��ֽ�
void Usart_SendByte( USART_TypeDef * pUSARTx, uchar ch)
{
	//����һ���ֽ�����
	USART_SendData(pUSARTx,ch);
	//�ȴ����ͼĴ���Ϊ��
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

//�����ַ���
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
	
	do 
	{
	Usart_SendByte( pUSARTx, *(str + k) );
	k++;
	} while (*(str + k)!='\0');

	//�ȴ��������
	while (USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET) 
	{}
}

void GPIO_CONFIG(void)
{
	//��ʼ������ʱ��
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
	
	//ʹ���źŶ˿�
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//ͬ���źŶ˿�
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/* TX����
	//synp
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
*/

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

void oswststore(void)
{
	temp[0]=oswst1;
	temp[1]=oswst2;
	FLASH_WriteByte(WRITE_START_ADDR,temp);
}

/* ���ж�����ִ��
void command_process()
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

	//GPIO_SetBits(GPIOA, GPIO_Pin_11);
	TIM_Cmd(TIM2, ENABLE);
	
	switchover=0;
  }
	else;
	oswctlcomd=0;
 }
 */

//�ı��������
void TIM4_redo(uchar add_f, uchar heat_t)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIM4);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//����ʱ��
		TIM_InternalClockConfig(TIM4);
		TIM_TimeBaseStructure.TIM_Period=(add_f-0x05)*10+heat_t;//20us
		TIM_TimeBaseStructure.TIM_Prescaler= 63;//64/64
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
		TIM_ARRPreloadConfig(TIM4, DISABLE);
}
 
void init(void)
{
	panel_number = 0x00;//���ӱ��
	RCC_Configuration();
	GPIO_CONFIG();
	BASIC_TIM_Config();
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
			heat_time = 0x20;
			TIM4_redo(add_flag,heat_time);
			adc_counter += 1;
			TIM_Cmd(TIM3,ENABLE);
		}
		else if (adc_counter > 30)
		{
			if (add_flag != rbyte4)
			{add_flag = rbyte4;TIM4_redo(add_flag,heat_time);}
		}
		/*
		if (ADC_ConvertedValue > 2070)
		{
			heat_time_new = 0x9F;
		}
		else if (ADC_ConvertedValue > 2000)
		{
			heat_time_new = 0x7F;
		}
		else if (ADC_ConvertedValue > 1950)
		{
			heat_time_new = 0x5F;
		}
		else if (ADC_ConvertedValue > 1900)
		{
			heat_time_new = 0x4A;
		}
		else if (ADC_ConvertedValue > 1870)
		{
			heat_time_new = 0x34;
		}
		else
		{
			heat_time_new = 0x20;
		}
		*/

	}
	return 0;
}

