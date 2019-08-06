
#include "stm32l1xx.h"
#include <stdio.h>
#include <string.h>
uint32_t i, temp, voltage; 
#define APBCLK   16000000UL
#define BAUDRATE 115200UL

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
void LED_GREEN_ON  (void);
void LED_GREEN_OFF (void);
void LED_BLUE_ON   (void);
void LED_BLUE_OFF  (void);
void EXTI0_IRQHandler (void);

int main()
{
	uint32_t i, temp, voltage;
	uint8_t command,  b=0;
	
	
	uint16_t  ADC_result, TS_result, Vdda, a, DAC_result;
	uint16_t ADC_data, TS_data, Vrefint_data;
  
	char txt_buf[200];
	char DAC_buf[10];
	
	RCC->CR|=RCC_CR_HSION; //HSI on
	RCC->CFGR|=RCC_CFGR_MCO_DIV2|RCC_CFGR_MCO_SYSCLK|RCC_CFGR_SW_0|RCC_CFGR_PPRE1_DIV1; //MCO is divided by 2; SYSCLK clock selected; PCLK1 not divided; HSI used as system clock

	//PORTB clocks
	RCC->AHBENR|=RCC_AHBENR_GPIOBEN; //LED blue and green
	
	//PORTA clocks
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN; //ADC, MCO, USART 
		
	//USART2 clocks
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;	
	
	//ADC clocks
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  //DAC clocks
  RCC->APB1ENR |= RCC_APB1ENR_DACEN; 
	
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
  USART2->BRR =0x8b ; //APBCLK/BAUDRATE // 115200 baud
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
	TIM3->PSC |= SystemCoreClock/1000-1; //f=1000 Hz
	TIM3->CR1 |= TIM_CR1_ARPE; //Auto-reload preload enable
	NVIC_EnableIRQ(TIM3_IRQn);
	
	/******************************************************************************/
	
	
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
  EXTI->IMR|=EXTI_IMR_MR0; //interrupt request from Line 0 is not masked
	EXTI->RTSR|=EXTI_RTSR_TR0; //rising edge trigger enabled for input line 0
	/******************************************************************************/
	
	SystemCoreClockUpdate(); //update SystemCoreClock
		
  	// \n - переместить курсор на строку вниз 
		// \r - переместить курсор в крайнее левое положение
		
		SendUSART((uint8_t *)" _____________________________  \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|   Developed by Ialaletdinov | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
		SendUSART((uint8_t *)"|                             | \n\r");
    SendUSART((uint8_t *)"|STM32l152RCT6 ready for work | \n\r");
		SendUSART((uint8_t *)"|_____________________________| \n\r");
		//for (i=0;i<200000;++i) {};	
    SendUSART((uint8_t *)"<Каллибровочные константы> нажмите z \n\r<Показания с каналов АЦП> нажмите x\n\r");
		SendUSART((uint8_t *)"<Помигать светодиодами> понажимайте q,w,e,r\n\r");
		//for (i=0;i<200000;++i) {};		
		SendUSART ((uint8_t*) "\n\rВведите желаемое напряжение от 0 до 3000 мВ и нажмите Enter ");	
		sprintf (txt_buf, "\n\rSystemCoreClock=%d",SystemCoreClock);
    SendUSART ((uint8_t*) txt_buf);	

	
			
while(1)
{ 
	/*
	if (GPIOA->IDR & GPIO_IDR_IDR_0)
	{
   LED_GREEN_ON ();
	}		
	*/
	
    /* 	
	  ADC1->CR2 |= ADC_CR2_JSWSTART; //start ADC, inj ch
	     
	  if  (ADC1->SR & ADC_SR_JEOC) //wait of JEOC
		{			
	  Vrefint_data = ADC1->JDR1;
	  Vdda = ((*Vref_int_cal) * 3000)/Vrefint_data;
	  }	
		
	  if  (ADC1->SR & ADC_SR_JEOC) //wait of JEOC
		{			
	  TS_data = ADC1->JDR2;
		TS_result = 80*(TS_data-(*TS_cal_30))/((*TS_cal_110)-(*TS_cal_30))+30; 
	  }				
			
	  if  (ADC1->SR & ADC_SR_JEOC) //wait of JEOC
		{			
	  ADC_data = ADC1->JDR3;
	  ADC_result = (Vdda*ADC_data)/4095;
	  }		
	 
		
	
	command = TakeUSART();
			switch(command)
			{
				case led_green_on:
				  LED_GREEN_ON();
				  SendUSART((uint8_t *)"\n\rLed green ON");
				break;
				case led_green_off:
					LED_GREEN_OFF();
				  SendUSART((uint8_t *)"\n\rLed green OFF");
				break;
				case led_blue_on:
					LED_BLUE_ON();
				  SendUSART((uint8_t *)"\n\rLed blue ON");
				break;
				case led_blue_off:
					LED_BLUE_OFF();
				  SendUSART((uint8_t *)"\n\rLed blue OFF");	
				break;
				case 'z':
				SendUSART((uint8_t *)"\n\r___Каллибровочные константы___");	
				sprintf (txt_buf, "\n\rTS_cal_30=%d \n\rTS_cal_110=%d \n\rVref_int_cal=%d",*TS_cal_30, *TS_cal_110, *Vref_int_cal);	
				SendUSART((uint8_t *)txt_buf);	
				SendUSART ((uint8_t*) "\n\rВведите желаемое напряжение от 0 до 3000 мВ и нажмите Enter ");	
				break;
				case 'x':
				SendUSART((uint8_t *)"\n\r___Коды каналов АЦП___");		
				sprintf (txt_buf, "\n\rADC_data=%d",ADC_data);
				SendUSART ((uint8_t*) txt_buf);	
				sprintf (txt_buf, "\n\rTS_data=%d",TS_data);
				SendUSART ((uint8_t*) txt_buf);	
				sprintf (txt_buf, "\n\rVrefint_data=%d",Vrefint_data);
				SendUSART ((uint8_t*) txt_buf); 
        SendUSART((uint8_t *)"\n\r___Показания с каналов АЦП___");					
        sprintf (txt_buf, "\n\rТекущее аналоговое напряжение АЦП Vdda=%d мВ",Vdda);
				SendUSART ((uint8_t*) txt_buf); 
				sprintf (txt_buf, "\n\rТемпература на кристалле TS=%d град.",TS_result);
				SendUSART ((uint8_t*) txt_buf); 
				sprintf (txt_buf, "\n\rНапряжение на АЦП U=%d мВ",ADC_result);
				SendUSART ((uint8_t*) txt_buf); 
				SendUSART ((uint8_t*) "\n\rВведите желаемое напряжение от 0 до 3000 мВ и нажмите Enter ");	
				break;
			}
  
	if (command>='0' && command<='9'|| command==Enter)
	  {				
		 DAC_buf[b]=command; //запись в массив цифры
		 sprintf (txt_buf, "%c",command); //вывод введеной цифры
		 SendUSART ((uint8_t*) txt_buf);	//вывод введеной цифры 
     ++b;
				
		
		 if (b>4 && command!=Enter) //проверка инкремента массива
		 {
			 SendUSART((uint8_t *)"\n\rнедопустимый ввод, введите заново\n\r ");
       b=0;
		 }		

		 
     if (command == Enter)	
     {
			//DAC-ADC conversion 
     sscanf (DAC_buf,"%d",&voltage); 
		 DAC->DHR12R1=voltage*4095/Vdda;
			 
     sprintf (txt_buf, "\n\rКод ЦАП %d",DAC->DOR1);
		 SendUSART ((uint8_t*) txt_buf);  
			 
		 SendUSART ((uint8_t*) "\n\rВведите желаемое напряжение   и нажмите Enter ");	 
		 b=0;      		 
     } 			 
			
	}
		*/	
	  
		
			
	
}//end while(1)

} //end main


void EXTI0_IRQHandler (void)
	{
	uint32_t i;
	LED_GREEN_ON ();	
	for (i=0;i<300000;++i);
	LED_GREEN_OFF ();	
	EXTI->PR|=EXTI_PR_PR0;	
	}	

void SendUSART (uint8_t *text)
{
	while(*text)
	{
		//uint32_t i;
		while(!(USART2->SR & USART_SR_TC)); //Transmission is complete
		for (i=0;i<3000;++i);
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


