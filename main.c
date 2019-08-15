
#include "stm32l1xx.h"
#include <stdio.h>
#include <string.h>
#define APBCLK   16000000UL
#define BAUDRATE 9600UL

#define TS_cal_30 ((uint16_t*) 0x1FF800FA)
#define TS_cal_110 ((uint16_t*) 0x1FF800FE)
#define Vref_int_cal ((uint16_t*) 0x1FF800F8)

#define Enter 0x0D //ascii-код Enter

#define led_green_on  'q'
#define led_green_off 'w'
#define led_blue_on   'e'
#define led_blue_off  'r'


void SendUSART (uint8_t *text);
uint8_t TakeUSART  (void); 
uint8_t TakeUSART_wait  (void);

void LED_GREEN_ON  (void);
void LED_GREEN_OFF (void);
void LED_BLUE_ON   (void);
void LED_BLUE_OFF  (void);

void EXTI0_IRQHandler (void);
void SysTick_Handler (void);
void TIM3_IRQHandler (void);

uint8_t hour=0, minute=0, sec=0, flag_sec=0, count_sec=0;
uint8_t led_mode=0;

int main()
{
	uint8_t i=0; //counter of the received bytes
	uint8_t input_ok=0; //sign of successful input
	uint32_t tim_count;
	
  char buf[10];
	char text_buf[200];
	
	RCC->CR|=RCC_CR_HSION; //HSI on
	RCC->CFGR|=RCC_CFGR_MCO_DIV2|RCC_CFGR_MCO_SYSCLK|RCC_CFGR_SW_0|RCC_CFGR_PPRE1_DIV1; //MCO is divided by 2; SYSCLK clock selected; PCLK1 not divided; HSI used as system clock

	//PORTB clocks
	RCC->AHBENR|=RCC_AHBENR_GPIOBEN; //LED blue and green
	
	//PORTA clocks
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN; //ADC, MCO, USART 
		
	//USART2 clocks
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	

	
	//SYSCFG clocks
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	//TIM3 clocks
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; 
	
	
	

	//GPIO LED init
	/******************************************************************************/
	GPIOB->MODER|=GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0; // PB7=01, PB6=01 output
  GPIOB->OTYPER|=0; //push-pull 
	GPIOB->OSPEEDR|=0; // Low Speed 
	GPIOB->PUPDR|=GPIO_PUPDR_PUPDR6_1|GPIO_PUPDR_PUPDR7_1;// pull-down
	/******************************************************************************/
	
		//GPIO blue button init
	/******************************************************************************/
	GPIOA->MODER &=~ GPIO_MODER_MODER0; // PA0=00 input
	GPIOA->OTYPER|=0; //push-pull 
	GPIOA->OSPEEDR|=0; // Low Speed 
	GPIOA->PUPDR|=GPIO_PUPDR_PUPDR6_1;// pull-down
	/******************************************************************************/
	
	
	//GPIO MCO init
	/******************************************************************************/
	GPIOA->MODER|=GPIO_MODER_MODER8_1; //PA8=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR &=~ GPIO_PUPDR_PUPDR8;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR8; // High Speed 
	GPIOA->AFR[1]|=(GPIO_AFRH_AFRH8>>4);
	/******************************************************************************/
	
	
	//GPIO USART init
	/******************************************************************************/
	GPIOA->MODER|=GPIO_MODER_MODER2_1|GPIO_MODER_MODER3_1; //PA2=10, PA3=10 AF
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR2 & ~GPIO_PUPDR_PUPDR3;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR2|GPIO_OSPEEDER_OSPEEDR3; // High Speed 
	GPIOA->AFR[0]|=((GPIO_AFRL_AFRL2 & (0x00000007<<8))|(GPIO_AFRL_AFRL3 & (0x00000007<<12))); //PA2, PA3 AF7
	/******************************************************************************/
	
		
	//GPIO ADC init
	/******************************************************************************/
	GPIOA->MODER |=GPIO_MODER_MODER1; // analog function ch1, PA1
	/******************************************************************************/
	
		
	//GPIO DAC init
	/******************************************************************************/
	GPIOA->MODER|=GPIO_MODER_MODER4|GPIO_MODER_MODER5; //PA4, PA5 Analog Function
  GPIOA->OTYPER|=0; //push-pull 
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4 & ~GPIO_PUPDR_PUPDR5;// no pull
	GPIOA->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR4|GPIO_OSPEEDER_OSPEEDR5; // High Speed 
	/******************************************************************************/
		
	//USART config
	/******************************************************************************/
	USART2->CR1 |= USART_CR1_UE; //USART on
  USART2->CR1 |= USART_CR1_M;  // 8 bit
  USART2->CR2 &=~ USART_CR2_STOP; //1 stop bit
  //USART2->BRR =0x8b ; //APBCLK/BAUDRATE // 115200 baud
	USART2->BRR =APBCLK/BAUDRATE; //APBCLK/BAUDRATE // 115200 baud
	USART2->CR1 &=~ USART_CR1_PCE; // parity bit disabled
  USART2->CR1 |= USART_CR1_TE; // USART transmitter  on 
  USART2->CR1 |= USART_CR1_RE; //USART receiver on
	/******************************************************************************/
	
	
	//DAC config
	/******************************************************************************/
  DAC->CR|=DAC_CR_EN1;
	//DAC->CR|=DAC_CR_TSEL1; // 111-software trigger
	/******************************************************************************/
		
	
	//ADC config, injected channel ch1
	/******************************************************************************/
	ADC1->CR1 &= ~ADC_CR1_RES; //res 12 bit
	ADC1->CR1 = ADC_CR1_SCAN; //scan mode enabled
	ADC1->CR2 &= ~ADC_CR2_CONT; //single conv mode
	ADC1->CR2 &= ~ADC_CR2_ALIGN; //right alignment
	ADC1->CR2 &= ~ADC_CR2_CFG; //bank A 
		
	ADC1->JSQR |= ADC_JSQR_JL_1; //3 conversions
	ADC1->JSQR |= ADC_JSQR_JSQ2_0|ADC_JSQR_JSQ2_4; //1st conversion in injected channel PA17(Vrefint)
	ADC1->JSQR |= ADC_JSQR_JSQ3_4; //2nd conversion in injected channel PA16(Tsensor) 
	ADC1->JSQR |= ADC_JSQR_JSQ4_0; //3rd conversion in injected channel PA1
	
  ADC1->SMPR3 |= ADC_SMPR3_SMP1; //sample time 384 cycles ch1=PA1, write when ADON=0 	
	ADC1->SMPR2 |= ADC_SMPR2_SMP17 ; //sample time 384 cycles, ch17 (write when ADON=0)
	ADC1->SMPR2 |= ADC_SMPR2_SMP16 ; //sample time 384 cycles, ch16 (write when ADON=0)
	
	
	ADC->CCR |=ADC_CCR_TSVREFE; //Temperature Sensor and VREFINT Enable 
	ADC1->CR2 |= ADC_CR2_ADON; //ADC on
	/******************************************************************************/
	
	
	//TIM3 config
	/******************************************************************************/
	TIM3->PSC |= (SystemCoreClock/1000)-1; //f=1000 Hz, 1 ms
	TIM3->CR1 |= TIM_CR1_ARPE; //Auto-reload preload enable
	NVIC_EnableIRQ(TIM3_IRQn);
	
	/******************************************************************************/
	SystemCoreClockUpdate(); //update SystemCoreClock
	
	//SysTick config. f=10Hz T=100 ms
	/******************************************************************************/
		SysTick_Config(SystemCoreClock/10);	
	/******************************************************************************/
		
	//NVIC config. 
	/******************************************************************************/
	__enable_irq();	
	//__disable_irq();	
	NVIC_EnableIRQ(EXTI0_IRQn);		
	/******************************************************************************/
	
	//EXTI0 config. 
	/******************************************************************************/
	SYSCFG->EXTICR[0]|=SYSCFG_EXTICR1_EXTI0_PA;//EXTI PA0 configuration
  
	EXTI->RTSR|=EXTI_RTSR_TR0; //rising edge trigger enabled for input line 0
	/******************************************************************************/
	
	
		
  	// \n - переместить курсор на строку вниз 
		// \r - переместить курсор в крайнее левое положение
		
		SendUSART((uint8_t *)" _____________________________  \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|   Developed by Ialaletdinov | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|STM32l152RCT6 ready for work | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
		SendUSART((uint8_t *)"<Каллибровочные константы> нажмите z \n\r<Показания с каналов АЦП> нажмите x\n\r");
		SendUSART((uint8_t *)"<Помигать светодиодами> понажимайте q,w,e,r\n\r");
		sprintf (text_buf, "\n\rSystemCoreClock=%d",SystemCoreClock);
    SendUSART ((uint8_t*) text_buf);	

	i=0;
	while (!input_ok)
	{
		if (i==0) SendUSART ((uint8_t *) "\n\rВведите значение задержки мигания в миллисекундах от 0 до 65535 "); 	
		buf[i] = TakeUSART_wait(); 
		if (buf[i]==Enter)
		{
			buf[i]=0; //значение кода конца строки(нуль-терминатора /0)	равно 0 
			sscanf(buf, "%d", &tim_count);
			sprintf (text_buf, "\n\rВведено время задержки %d миллисекунд, пишем его в регистр TIM3_ARR\n\r", tim_count);
			SendUSART ((uint8_t *) text_buf);
			TIM3->ARR = tim_count; //maximum value
			TIM3->CNT = 0;
			TIM3->DIER |= TIM_DIER_UIE; //Update interrupt enable TIM3, при переполнении
			TIM3->CR1 |= TIM_CR1_CEN; //start TIM3
			EXTI->IMR|=EXTI_IMR_MR0; //interrupt request from Line 0 is not masked, разрешить прерывания EXTI
			
			input_ok=1; //разрешить выход из цикла ввода
		}
		else //не Enter
		{
     if (i>4 || buf[i]<'0' || buf[i]>'9') 
		 { 
		  SendUSART ((uint8_t*) "\n\rНедопустимый ввод");
		  i=0;
      buf[0]=0;
		 }
		 else 
     {
			if (USART2->SR & USART_SR_TC)
			{	
      USART2->DR = buf[i]; //вывод эхо=принтого байта
      i++;
			}				
		 } 			 
		}			
	}
			
while(1)
{ 
	if (flag_sec==1)
	{
	flag_sec=0;
		sprintf (text_buf, "\n\rSTM32l152RC работает %d часов %d минут %d секунд",hour, minute, sec);	
		SendUSART ((uint8_t*) text_buf);
	}	

		
			
	
}//end while(1)

} //end main

void TIM3_IRQHandler (void)
{	
 TIM3->SR &=~ TIM_SR_UIF;
 if (led_mode)
 {
  LED_GREEN_ON();
	LED_BLUE_ON();
 }
 else
 {
  if (GPIOB->ODR & GPIO_ODR_ODR_6) LED_BLUE_OFF(); 
	else LED_BLUE_ON();
	
	if (GPIOB->ODR & GPIO_ODR_ODR_7) LED_GREEN_OFF(); 
	else LED_GREEN_ON(); 
 }	 
}
void EXTI0_IRQHandler (void)
	{
	led_mode=~led_mode;
	LED_BLUE_OFF();	
	LED_GREEN_OFF();	
	EXTI->PR|=EXTI_PR_PR0;	
	}	
	
void SysTick_Handler (void)
{
 count_sec++;
 if (count_sec==10)
 {
 count_sec=0;	 
 flag_sec=1;
 sec++;	
 if (sec==60)
  {
   minute++;
	 sec=0;	
   if (minute==60)
   {
   hour++;
	 minute=0;
	 } 		 
	}
 }	
}


void SendUSART (uint8_t *text)
{
	while(*text)
	{
		//uint32_t i;
		while(!(USART2->SR & USART_SR_TC)); //Transmission is complete
		//for (i=0;i<3000;++i);
		USART2->DR = *text;	//write data
		text++;		
	}
}

uint8_t TakeUSART (void)
{
	uint8_t data;
	
	if (USART2->SR & USART_SR_RXNE) //Received data is ready to be read
		{
			data = USART2->DR; //read data
		}
		
		return data;
}

uint8_t TakeUSART_wait (void)
{
	while (!(USART2->SR & USART_SR_RXNE)) {}; //Received data is ready to be read
	return (USART2->DR); //read data
}

void LED_GREEN_ON (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_7;
}

void LED_GREEN_OFF (void)
{
	GPIOB->BSRRH|=GPIO_BSRR_BS_7; //reset PB7 to low level
}

void LED_BLUE_ON (void)
{
	GPIOB->ODR|=GPIO_ODR_ODR_6;
}

void LED_BLUE_OFF (void)
{
	GPIOB->BSRRH|=GPIO_BSRR_BS_6; ////reset PB6 to low level
}


