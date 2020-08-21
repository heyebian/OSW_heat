#include "stm32f10x.h"
#include "my_own.h"

//适用于STM32F103C8T6
//暂时最终版本
//写入的起始地址和技术地址
#define WRITE_START_ADDR ((uint32_t)0x08008000)
#define WRITE_END_ADDR ((uint32_t)0x0800C000)

uchar eepromupdate,oswctlcomd,readstcomd,readeepromcomd,switchover;//命令

//暂存状态
uchar temp1n,temp2n,temp3n,temp4n,temp5n,temp6n,temp7n,temp8n,temp9n,temp10n,temp11n,temp12n;
uchar temp1p,temp2p,temp3p,temp4p,temp5p,temp6p,temp7p,temp8p,temp9p,temp10p,temp11p,temp12p;
//判断后的控制字
uchar oswctr1,oswctr2,oswctr3,oswctr4,oswctr5,oswctr6,oswctr7,oswctr8,oswctr9,oswctr10,oswctr11,oswctr12;
uchar rcount;//接受字数判断
uchar rbyte1,rbyte2,rbyte3;//1:板号 2:12-5 3:4-1
uchar sbyte1,sbyte2,sbyte3;//same
uchar oswst1,oswst2;
uchar adcount;
uchar swtime;//切换状态次数
uchar basictime;//脉冲长度
uchar comdok;
uchar temp[3]={0};//flash暂存
uchar exitst,checktmp;
uchar flag;//tim_test

//没有外接晶振所以采用的是内部晶振8M
void BASIC_TIM_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	TIM_DeInit(TIM2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//开启时钟
	TIM_InternalClockConfig(TIM2);
	TIM_TimeBaseStructure.TIM_Period=40;//35us
	TIM_TimeBaseStructure.TIM_Prescaler= 7;//8M/8
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	TIM_ARRPreloadConfig(TIM2, DISABLE);
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
	//抢断优先级为0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);

	//配置定时器为中断源
	NVIC_InitStructure.NVIC_IRQChannel = BASIC_TIM_IRQ ;
	//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//子优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);

	//配置中断源
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	//使能中断
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStructure);
}

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

	//将USART Rx配置为浮空输入
	GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
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
	//完成串口初始化
	USART_Init(DEBUG_USARTx, &USART_InitStructure);
	//串口优先级配置
	NVIC_Configuration();
	//使能串口接受中断
	USART_ITConfig(DEBUG_USARTx, USART_IT_RXNE, ENABLE);

	//使能串口
	USART_Cmd(DEBUG_USARTx, ENABLE);
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
	
	PWR_BackupAccessCmd(ENABLE);
	RCC_LSEConfig(RCC_LSE_OFF);
	BKP_TamperPinCmd(DISABLE);
	BKP_ITConfig(DISABLE); 
	PWR_BackupAccessCmd(DISABLE);
	
	//使能信号端口
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//synp
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

//延时函数
void Delay(__IO uint32_t nCount)
{
	for (; nCount != 0; nCount--);
}

void delay_ms(u16 time)
{    
	u16 i=0;  
	while(time--)
	{
		i=12000;  
		while(i--) ;    
	}
}

void command_process()
{
	if(switchover==1)
	{if((rbyte2 & 0x80)==0)	 //12
        oswctr12=0;
	else oswctr12=1;
	if((rbyte2 & 0x40)==0)//11
        oswctr11=0;
	else oswctr11=1;
	if((rbyte2 & 0x20)==0)//10
        oswctr10=0;
	else oswctr10=1;
	if((rbyte2 & 0x10)==0)//9
        oswctr9=0;
	else oswctr9=1;
	if((rbyte2 & 0x08)==0)//8
        oswctr8=0;
	else oswctr8=1;
	if((rbyte2 & 0x04)==0)//7
        oswctr7=0;
	else oswctr7=1;
	if((rbyte2 & 0x02)==0)//6
        oswctr6=0;
	else oswctr6=1;
	if((rbyte2 & 0x01)==0)//5
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
	{GPIO_SetBits(IN12P);GPIO_ResetBits(IN12N);temp12p=1;temp12n=0;}
	else  //no.12
	{GPIO_ResetBits(IN12P);GPIO_SetBits(IN12N);temp12p=0;temp12n=1;}
	if(oswctr11)  //no.11
	{GPIO_SetBits(IN11P);GPIO_ResetBits(IN11N);temp11p=1;temp11n=0;}
	else  //no.11
	{GPIO_ResetBits(IN11P);GPIO_SetBits(IN11N);temp11p=0;temp11n=1;}
	if(oswctr10)  //no.10
	{GPIO_SetBits(IN10P);GPIO_ResetBits(IN10N);temp10p=1;temp10n=0;}
	else  //no.10
	{GPIO_ResetBits(IN10P);GPIO_SetBits(IN10N);temp10p=0;temp10n=1;}
	if(oswctr9)  //no.9
	{GPIO_SetBits(IN9P);GPIO_ResetBits(IN9N);temp9p=1;temp9n=0;}
	else  //no.9
	{GPIO_ResetBits(IN9P);GPIO_SetBits(IN9N);temp9p=0;temp9n=1;}
	if(oswctr8)  //no.8
	{GPIO_SetBits(IN8P);GPIO_ResetBits(IN8N);temp8p=1;temp8n=0;}
	else  //no.8
	{GPIO_ResetBits(IN8P);GPIO_SetBits(IN8N);temp8p=0;temp8n=1;}
	if(oswctr7)  //no.7
	{GPIO_SetBits(IN7P);GPIO_ResetBits(IN7N);temp7p=1;temp7n=0;}
	else  //no.7
	{GPIO_ResetBits(IN7P);GPIO_SetBits(IN7N);temp7p=0;temp7n=1;}
	if(oswctr6)  //no.6
	{GPIO_SetBits(IN6P);GPIO_ResetBits(IN6N);temp6p=1;temp6n=0;}
	else  //no.6
	{GPIO_ResetBits(IN6P);GPIO_SetBits(IN6N);temp6p=0;temp6n=1;}
	if(oswctr5)  //no.5
	{GPIO_SetBits(IN5P);GPIO_ResetBits(IN5N);temp5p=1;temp5n=0;}
	else  //no.5
	{GPIO_ResetBits(IN5P);GPIO_SetBits(IN5N);temp5p=0;temp5n=1;}
	if(oswctr4)  //no.4
	{GPIO_SetBits(IN4P);GPIO_ResetBits(IN4N);temp4p=1;temp4n=0;}
	else  //no.4
	{GPIO_ResetBits(IN4P);GPIO_SetBits(IN4N);temp4p=0;temp4n=1;}
	if(oswctr3)  //no.3
	{GPIO_SetBits(IN3P);GPIO_ResetBits(IN3N);temp3p=1;temp3n=0;}
	else  //no.3
	{GPIO_ResetBits(IN3P);GPIO_SetBits(IN3N);temp3p=0;temp3n=1;}
	if(oswctr2)  //no.2
	{GPIO_SetBits(IN2P);GPIO_ResetBits(IN2N);temp2p=1;temp2n=0;}
	else  //no.2
	{GPIO_ResetBits(IN2P);GPIO_SetBits(IN2N);temp2p=0;temp2n=1;}
	if(oswctr1)  //no.1
	{GPIO_SetBits(IN1P);GPIO_ResetBits(IN1N);temp1p=1;temp1n=0;}
	else  //no.1
	{GPIO_ResetBits(IN1P);GPIO_SetBits(IN1N);temp1p=0;temp1n=1;}

	GPIO_SetBits(GPIOA, GPIO_Pin_11);
	TIM_Cmd(TIM2, ENABLE);
	switchover=0;
	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
  }
	else;
	oswctlcomd=0;
 }

void oswststore(void)
{
	temp[0]=rbyte2;
	temp[1]=rbyte3;
	FLASH_WriteByte(WRITE_START_ADDR,temp);
	eepromupdate=0;
}

 
void init(void)
{
	GPIO_CONFIG();
	BASIC_TIM_Config();
	USART_Config();
	EXTI_Key_Config();
	GPIO_ResetBits(GPIOA, GPIO_Pin_11);
	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	oswctlcomd=0;eepromupdate=0;readstcomd=0;readeepromcomd=0;
	switchover=1;swtime=0;
	rcount=0;comdok=0;    
	adcount=0; basictime=5;
	FLASH_ReadByte(WRITE_START_ADDR,temp);
	rbyte2=temp[0];
	rbyte3=temp[1];
	command_process();
	oswst1=rbyte2;oswst2=rbyte3;
	exitst=0;
}

void open_n(void)
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
}

void open_p(void)
{
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

void close_n(void)
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
}
void close_p(void)
{		 
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

	
int main(void)
{
	init();
	while(1)
	{
	if(oswctlcomd==1)
	{command_process();}
	else if(eepromupdate==1)
	{oswststore();}
	else if(readstcomd==1)
	{
		sbyte1=0x00;sbyte2=oswst1;sbyte3=oswst2;
		//发送00 oswst1 oswst2 swtime
		Usart_SendByte(DEBUG_USARTx,sbyte1);
		Usart_SendByte(DEBUG_USARTx,sbyte2);
		Usart_SendByte(DEBUG_USARTx,sbyte3);
		Usart_SendByte(DEBUG_USARTx,swtime);
		readstcomd=0;
	} 
	else if(readeepromcomd==1)
	{
		sbyte1=0x00;	
		FLASH_ReadByte(WRITE_START_ADDR,temp);
		sbyte2=temp[0];
		sbyte3=temp[1];
		//发送00 sbyte2 sbyte3 swtime
		Usart_SendByte(DEBUG_USARTx,sbyte1);
		Usart_SendByte(DEBUG_USARTx,sbyte2);
		Usart_SendByte(DEBUG_USARTx,sbyte3);
		Usart_SendByte(DEBUG_USARTx,swtime);
		readeepromcomd=0;
	}
	}
	return 0;
}

