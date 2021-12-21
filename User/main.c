#include "stm32f10x.h"
#include "my_own.h"

//适用于STM32F103CBU6
//修改于2021/11/20（增加了温度传感器控制功能）
//修改于2021/11/16（增加了heat功能，未经测试）
//修改于2021/11/1（将内部8M分频成64M）
//适用于多机通信版本
//基本就是final了/并不是
//写入的起始地址和技术地址
//交叉时发送的是0
//平行时发送的是1
#define WRITE_START_ADDR ((uint32_t)0x08009000)
#define WRITE_END_ADDR ((uint32_t)0x0800C000)

uchar oswctlcomd,readstcomd,readbitcomd,readoswcmd,switchover,readtem;//命令

//判断后的控制字
uchar oswctr1,oswctr2,oswctr3,oswctr4,oswctr5,oswctr6,oswctr7,oswctr8,oswctr9,oswctr10,oswctr11,oswctr12;
uchar rcount;//接受字数判断
uchar rbyte1,rbyte2,rbyte3,rbyte4;//1:板号 2:12-5 3:4-1 4:CtlCmd
uchar sbyte1,sbyte2,sbyte3;//same
uchar oswst1,oswst2;
uchar adcount;
uchar swtime;//切换状态次数
uchar basictime;//脉冲长度
uchar comdok;
uchar temp[3]={0};//flash暂存
uchar panel_number;//板子编号
uchar xcom_flag;//命令格式控制，如果是xcom为1，上位机则为0
uchar heat_time;
uint16_t ADC_ConvertedValue;
uchar add_flag;
uchar heat_time_new;
uchar adc_counter;

//设置频率为64M
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

//没有外接晶振所以采用的是内部晶振8M，经分频得到的64M
void BASIC_TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//切换脉冲20*10 = 200us
	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//开启时钟
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Period=20;//20us
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64M/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM2, DISABLE);
	
	//加热周期2000ms
	TIM_DeInit(TIM3);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//开启时钟
	TIM_InternalClockConfig(TIM3);
	TIM_TimeBaseStructure.TIM_Period=2000;//2ms
	TIM_TimeBaseStructure.TIM_Prescaler= 63;//64/64
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM3, DISABLE);
		
	//加热脉冲20us(实际长度不一致)
	TIM_DeInit(TIM4);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//开启时钟
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

	//嵌套中断控制器组选择
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//配置USART为中断源
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQ;
	//抢断优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);

	//配置定时器2为中断源
	NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQ ;
	//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	//配置定时器3为中断源
	//负责Heat
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn ;
	//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	//配置定时器4为中断源
	//负责Heat
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn ;
	//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	//子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	//配置ADC为中断源
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	//子优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);
	
	/*
	//配置中断源
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);
	*/
}


//外部中断配置
/*
void EXTI_Key_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	//时钟配置
	RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO),ENABLE);
	//NVIC配置
	NVIC_Configuration();
	//串口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//中断配置
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

	//打开串口GPIO的时钟
	DEBUG_USART_GPIO_APBxClkCmd(DEBUG_USART_GPIO_CLK, ENABLE);

	//打开串口外设的时钟
	DEBUG_USART_APBxClkCmd(DEBUG_USART_CLK, ENABLE);

	//将USART Tx配置为推挽复用
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	//将USART Rx配置上拉输入
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	//配置串口的工作参数
	//配置波特率
	USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
	//配置数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	//配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	//配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//USART_InitStructure.USART_Mode = USART_Mode_Rx ;
	//完成串口初始化
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	//串口优先级配置
	NVIC_Configuration();
	//使能串口接受中断
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	//使能串口
	USART_Cmd(DEBUG_USARTx, ENABLE);
}

void ADC_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	//打开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	//只使用一个ADC,独立模式
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	//禁用扫描模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE ;
	//连续转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//禁用外部触发
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//右边对齐
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//设置转换通道数量
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	//初始化ADC
	ADC_Init(ADC1, &ADC_InitStructure);
	//设置8分分频，即64/8=8Mhz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	//设置采样周期239.5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);
	//设置中断
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	
	ADC_TempSensorVrefintCmd(ENABLE);
	//开启ADC
	ADC_Cmd(ADC1, ENABLE);
	//校准
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1));
	//软件触发
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


//发送单字节
void Usart_SendByte( USART_TypeDef * pUSARTx, uchar ch)
{
	//发送一个字节数据
	USART_SendData(pUSARTx,ch);
	//等待发送寄存器为空
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

//发送字符串
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
	
	do 
	{
	Usart_SendByte( pUSARTx, *(str + k) );
	k++;
	} while (*(str + k)!='\0');

	//等待发送完成
	while (USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET) 
	{}
}

void GPIO_CONFIG(void)
{
	//初始化并打开时钟
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
	
	//使能信号端口
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//同步信号端口
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/* TX复用
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

/* 在中断里面执行
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

//改变加热脉宽
void TIM4_redo(uchar add_f, uchar heat_t)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIM4);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//开启时钟
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
	panel_number = 0x00;//板子编号
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

