#define uchar unsigned char

//IO
#define IN1N GPIOA, GPIO_Pin_0
#define IN1P GPIOA, GPIO_Pin_1
#define IN2N GPIOA, GPIO_Pin_2
#define IN2P GPIOA, GPIO_Pin_3
#define IN3N GPIOA, GPIO_Pin_4
#define IN3P GPIOA, GPIO_Pin_5
#define IN4N GPIOA, GPIO_Pin_6
#define IN4P GPIOA, GPIO_Pin_7
#define IN5N GPIOB, GPIO_Pin_0
#define IN5P GPIOB, GPIO_Pin_1
#define IN6N GPIOB, GPIO_Pin_10
#define IN6P GPIOB, GPIO_Pin_11
#define IN7N GPIOB, GPIO_Pin_5
#define IN7P GPIOB, GPIO_Pin_6
#define IN8N GPIOB, GPIO_Pin_7
#define IN8P GPIOB, GPIO_Pin_8
#define IN9N GPIOB, GPIO_Pin_12
#define IN9P GPIOB, GPIO_Pin_13
#define IN10N GPIOB, GPIO_Pin_14
#define IN10P GPIOB, GPIO_Pin_15
#define IN11N GPIOB, GPIO_Pin_9
#define IN11P GPIOC, GPIO_Pin_13
#define IN12N GPIOC, GPIO_Pin_14
#define IN12P GPIOC, GPIO_Pin_15
#define ENA GPIOA, GPIO_Pin_11

//串口1-USART1
#define DEBUG_USARTx USART1
#define DEBUG_USART_CLK RCC_APB2Periph_USART1
#define DEBUG_USART_APBxClkCmd RCC_APB2PeriphClockCmd
#define DEBUG_USART_BAUDRATE 115200
#define DEBUG_USART_GPIO_CLK (RCC_APB2Periph_GPIOA)
#define DEBUG_USART_GPIO_APBxClkCmd RCC_APB2PeriphClockCmd

//USART GPIO引脚宏定义
#define DEBUG_USART_TX_GPIO_PORT GPIOA
#define DEBUG_USART_TX_GPIO_PIN GPIO_Pin_9
#define DEBUG_USART_RX_GPIO_PORT GPIOA
#define DEBUG_USART_RX_GPIO_PIN GPIO_Pin_10

#define DEBUG_USART_IRQ USART1_IRQn
#define DEBUG_USART_IRQHandler USART1_IRQHandler

//使用定时器2
#define BASIC_TIM TIM2
#define BASIC_TIM_APBxClock_FUN RCC_APB1PeriphClockCmd
#define BASIC_TIM_CLK RCC_APB1Periph_TIM2
#define BASIC_TIM_IRQ TIM2_IRQn
#define BASIC_TIM_IRQHandler TIM2_IRQHandler

