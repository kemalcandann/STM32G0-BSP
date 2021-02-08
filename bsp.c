#include "bsp.h"

volatile uint32_t tick = 0;

void SysTick_Handler(void)
{
	if (tick > 0)
		--tick;
}

void delay(volatile unsigned int s)
{
	for(; s > 0; --s);
}

void delay_ms(volatile uint32_t s)
{
	for (uint32_t i = 0; i < s; ++i);
}

void BSP_button_init()
{
	  RCC->IOPENR |= (1U << 1);				// GPIOB port enable

	  GPIOB->MODER &= ~(3U << 2*2);			// B2 button input mode enable
}

int BSP_button_read()
{
		 int button = (GPIOB->IDR >> 2) & 1;// read B2 button (pressed or not pressed)

		if(button)
			return 1;
		else
	    	return 0;
}

void BSP_led_init(void)
{
	 RCC->IOPENR |= (1U << 2);				// GPIOC port enable

	 GPIOC->MODER &= ~(3U << 2*6);			// setup PC6 as output mode
	 GPIOC->MODER |= (1U << 2*6);

	 GPIOC->BRR |= (1U << 6);
}

void BSP_led_set(void)
{
    GPIOC->ODR |= (1U << 6);				// PC6 led set
}

void BSP_led_clear(void)
{
 	 GPIOC->BRR |= (1U << 6);				// PC6 led clear
}

void BSP_led_toggle(void)
{
    GPIOC->ODR ^= (1U << 6);				// PC6 led toggle
}

void init_timer1(void)						// feed to PWM
{
	// init TIM1
	RCC->APBENR2 |= (1U << 11);				// TIM1 enable on APB bus
	TIM1->CR1 = 0;
	TIM1->CR1 |= (1<<7); 					// ARPE bit set
	TIM1->CNT = 0;
	TIM1->PSC = 99;
	TIM1->ARR = 16000; 						// every 100 ms jump to the TIM1 handler
	TIM1->DIER |= (1<<0); 					// enable interrupts

	TIM1->CR1 |= (1<<0);

	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
	TIM1->SR &= ~(1U << 0); 				// update SR registers when jump the code TIM1 handler
}

void init_timer2(void)						//for ADC
{
	RCC->APBENR1 |= (1U << 0);
	TIM2->CR1 = 0;
	TIM2->CNT = 0;
	TIM2->PSC = 9;  						// arrange PSC and ARR registers, according to your porpuse
	TIM2->ARR = (16000);					// every 1 second jump to the TIM2 handler
	TIM2->CR2 |= (1 << 5);  				//choose update mode,
	TIM2->CR1 |= (1 << 0);
}

void TIM2_IRQHandler()
{
	TIM2->SR &=  ~(1U<<0); 					// update SR registers
}

void init_timer3(void)						// PWM use TIM3
 {
	RCC->APBENR1 |= (1U << 1);				//TIM3 enable on APB bus

	TIM3->CR1 = 0;							// All bits are zero but reset value is 0 (not necessary)
	TIM3->CR1 |= (1U << 7);					// TIMx_ARR register is buffered, Auto-reload enable
	TIM3->CNT = 0;							// zero out counter
	TIM3->PSC = 9;
	TIM3->ARR = 16000;						// constant on global namespace

	TIM3->DIER |= (1U << 0);				// update interrupt enable
	// capture compare mode 1 (channel 1)
	TIM3->CCMR1 |= (1U << 3);				// output compare 1 mode preload enable
	TIM3->CCMR1 |= (1U << 5);				// PWM Mode output compare mode enable 110 -> OC1M [0110]
	TIM3->CCMR1 |= (1U << 6);
	TIM3->CCMR1 &= ~(1U << 16);				// PWM1 mode enable OC1M enable
	TIM3->CCMR1 &= ~(1U << 4);
	TIM3->CCER |= (1U << 0);				// CCER register capture compare 1 output enable
	TIM3->CCR1 = 0;			     			// duty cycle

	// channel 2
	TIM3->CCMR1 &= ~(0x7U << 12); 			// clear OCM2
	TIM3->CCMR1 &= ~(1U << 24); 			// clear
	TIM3->CCMR1 |= (1U << 11);
	TIM3->CCMR1	 |= (0x6U << 12);			// PWM Mode output compare mode enable 110 -> OC2M [0110]
	TIM3->CCER |= (1U << 4);
	TIM3->CCR2 = 500;	// capture compare mode 2 for channel 2

	TIM3->CR1 |= (1U << 0);					// TIM3 enable

	NVIC_SetPriority(TIM3_IRQn, 1);			// TIM3 NVIC enable
	NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
	TIM3->SR &= ~(1U << 0);					// status register flag
}

void EXTI0_1_IRQHandler()
{
	//...
	//...
}

void BSP_uart_init(uint32_t baud)
{
											// PA2 PA3 are connected to the on port using uart2
	RCC->IOPENR |= (1U << 0);
	RCC->APBENR1 |= (1U << 17);

	// setup PA3 AF mode
	GPIOA->MODER &= ~(3U << 2*2);
	GPIOA->MODER |= (2 << 2*2);				// 10 alternate function mode so 2(10) shift left 4

											// choese AF2 from low register mux
	GPIOA->AFR[0] &= ~(0xFU << 4*2);
	GPIOA->AFR[0] |= (1 << 4*2);

	GPIOA->MODER &= ~(3U << 2*3);			// setup PA3 AF mode
	GPIOA->MODER |= (2 << 2*3);

	GPIOA->AFR[0] &= ~(0xFU << 4*3);		// chooese AF3 from low register mux
	GPIOA->AFR[0] |= (1 << 4*3);

	// setup uart2
	USART2->CR1 = 0;						// clear all bits on CR1 register
											// transmit receiver bits enable
	USART2->CR1 |= (1U << 3);
	USART2->CR1 |= (1U << 2);

	USART2->CR1 |= (1U << 5);				// RXNIE register enable
	// BRR register writes 16 bit, other bits reserved
	USART2->BRR = (uint16_t)(SystemCoreClock / baud);


	USART2->CR1 |= (1U << 0);				// uart2 enable

	NVIC_SetPriority(USART2_IRQn , 1);		// USART2 NVIC enable
	NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQhandler(void)
{
	uint8_t data = (uint8_t)USART2->RDR;	// ISR flag set by hardware
	print_char(data);						// RXNE is automatically cleared when read
}

void uart_tx(uint8_t c)
{
	USART2->TDR = c;
	while(!(USART2->ISR && (1 << 6) ));
}

uint8_t uart_rx(void)
{
	char *s = "kemal candan\n\r";
	int len = 0;
	int i = 0;
	while(s[len++] != '\0');
	if(i == len)
		i = 0;
	return s[i++];
}

void print_char(uint8_t c)
{
	USART2->TDR = c;
	while(!(USART2->ISR & (1U << 6)));		// if is 1 get out here or if is 0 wait until complete transmission
}

int _print(int fd, char *buf, int len)
{
	(void)fd;
	for(int i = 0; i < len ; ++i)
		print_char(buf[i]);
	return len;
}
void print(char *buf)
{
	int len = 0;
	while(buf[len++] != '\0');
	_print(0, buf, len);
}

void BSP_iwdg_init(void)
{
	 RCC->CSR |= (1U << 0);					//RCC->CSR register first bit active for LSION, independent watchdog works LSI(Low speed clock)
	 while((RCC->CSR & (1U << 1)) == 0);

    IWDG->KR  = 0x5555; 					// enable write to PR, RLR ister
    IWDG->PR  = IWDG_PR;					// Init prescaler
    IWDG->RLR = IWDG_RLR;					//Init RLR
    IWDG->KR  = 0xAAAA;						// periodically feed the dog(reload independent watchdog)
    IWDG->KR  = 0xCCCC;  					// start and enable independent watchdog

    if(RCC->CSR & (1<< 29))					// IWDG Reset Flag set
    	RCC->CSR |= (1<<23);  				// IWDG clear reset Flag set
	else
	   	BSP_led_clear();
}

void BSP_init_PWM(void)
{
	RCC->APBENR2 |= (1U << 11);				// TIM1 enable
	RCC->IOPENR |= (1U << 0);				// GPIOA port enable

	TIM1->CR1 = 0;							// All bits are zero but reset value is 0
	TIM1->CR1 |= (1U << 7);					// TIMx_ARR register is buffered, Auto-reload enable
	TIM1->CNT = 0;

	TIM1->DIER |= (1U << 0);

	GPIOA->MODER &= ~(3U << 2*8);			// PA8 Alternate function mode enable
	GPIOA->MODER |= (2U << 2*8);

	GPIOA->AFR[1] &= ~(0xFU << 4*0); 		// choose AF0 from high register mux
	GPIOA->AFR[1] |= (2U << 4*0);

	GPIOA->MODER &= ~(3U << 2*9);			// PA9 Alternate function mode enable
	GPIOA->MODER |= (2U << 2*9);

	GPIOA->AFR[1] &= ~(0xFU << 4*1); 		// choose AF0 from high register mux
	GPIOA->AFR[1] |= (2U << 4*1);

	TIM1->CCMR1 |= (1U << 3);
	TIM1->CCMR1 &= ~(1U << 16);				// PWM1 mode enable  BAK
	TIM1->CCMR1 |= (1U << 5);
	TIM1->CCMR1 |= (1U << 6);
	TIM1->CCMR1 &= ~(1U << 4);


	TIM1->CCER |= (1U << 0);				// CCER register capture compare 1 output enable

	TIM1->ARR = 0;							// clear ARR register
	TIM1->ARR = 16000;						// ARR register period 1000
	TIM1->PSC = 10;							// 10 ms de interrupt
	TIM1->CCR1 = 2000;						// duty cycle

	TIM1->CR1 |= (1U << 0);					// TIM1 enable

	NVIC_SetPriority(TIM1_CC_IRQn, 0);		// TIM1 NVIC enable
	NVIC_EnableIRQ(TIM1_CC_IRQn);

}

void BSP_init_ADC(void)
{
	RCC->IOPENR |=(1U << 0);				// A port enable
	RCC->APBENR2 |= (1U << 20);				// ADC port enable

	GPIOA->MODER &= ~(3U << 2 * 7);			// PA7 analog mode
	GPIOA->MODER |= (3U << 2 * 7);

	ADC1->CR = 0;
	ADC1->CR |= (1U << 28);					// ADC voltage regulater register enable
	delay_ms(100);							//wait until voltage regulater become stable

	ADC1->CR |= (1U << 31);					// enable calibraiton register
	ADC1->CFGR1 = (1U << 4);				// 8 bits sample (10)
	while (ADC1->CR & (1U << 31));			// wait until calibraiton is completed
	ADC1->SMPR |= (1 << 1);					// 7.5 clock cycle enable (010)
	ADC1->SMPR |= (1 << 0);

	ADC1->CFGR1 &= ~(1U << 8);
	ADC1->CFGR1 |= (1 << 7);				// TRGO register enable bits
	ADC1->CFGR1 &= ~(1U << 6);

	ADC1->CFGR1 |= (1 << 11);				//EXTEN enable
	ADC1->CFGR1 |= (1 << 10);

	ADC1->CHSELR |= (1U << 7); 				// ADC channel seleciton register

	ADC1->IER |= (1U << 3); 				//enable sequence interrpt seleciton register

	ADC1->ISR &= ~(1U << 0);
	ADC1->CR |= (1U << 0);					// enable ADC

	while (!(ADC1->ISR & (1 << 0)));		// wait until ADC ready flag is 1
	ADC1->CR |= (1U << 2);					// ADC start conversion command enable

	NVIC_SetPriority(ADC1_IRQn,2);			// ADC NVIC enable
	NVIC_EnableIRQ(ADC1_IRQn);
}


void ADC_COMP_IRQHandler ()
{
	//...
	//...
	ADC1->ISR |= (1U << 3); 				// clear EOS flag
}

uint16_t read_ADC(void)
{
	ADC1->CR |= (1U << 2);					// ADC start conversion command register enable
	while(!(ADC1->ISR & (1U << 2)));		// End of conversion flag is set? wait until be done
	return  (uint16_t)ADC1->DR;				// return data from pin (Data register has 16 bits)
}


void I2C_init(void)							// PA9 SCL, PA10 SDA
{
	RCC->IOPENR |= (1U << 0);				// A port enable
	RCC->APBENR1 |= (1U << 21);				// I2C1 enable

	GPIOA->MODER &= ~(3U << 2 * 9);
	GPIOA->MODER |= (2U << 2 * 9);			// PA9 Alternate function mode
	GPIOA->OTYPER |= (1U << 9);				// Output open-drain for PA9 SCL
	GPIOA->AFR[1] &= ~(0xFU << 4 * 1);
	GPIOA->AFR[1] |= (6U << 4 * 1);			// PA9 AFRH register AF6


	GPIOA->MODER &= ~(3U << 2 * 10);
	GPIOA->MODER |= (2U << 2 * 10);			// PA10 Alternate function mode
	GPIOA->OTYPER |= (1U << 10);			// Output open-drain for PA10  SDA
	GPIOA->AFR[1] &= ~(0xFU << 4 * 2);
	GPIOA->AFR[1] |= (6U << 4 * 2);			// PA10 AFRH register AF6


	I2C1->CR1 = 0;
	I2C1->CR1 |= (1U << 7);					// Error interrupts register enable if error occurs jump to ISR

	I2C1->TIMINGR |= (3U << 28);			// PSC register
	I2C1->TIMINGR |= (0x13U << 0);			// SCL low period (master mode) register
	I2C1->TIMINGR |= (0xFU << 8);			// Data hold time register
	I2C1->TIMINGR |= (2U << 16);			// Data hold time register
	I2C1->TIMINGR |= (4U << 20);			// Data setup time register

	I2C1->CR1 |= (1U << 0);					// Peripheral enable


	NVIC_SetPriority(I2C1_IRQn,2);
	NVIC_EnableIRQ(I2C1_IRQn);
}

void write_eeprom(uint8_t devAddr, uint16_t memAddr,uint16_t* data, int size)
{
	//Send address and register to read
	I2C1->CR2 = 0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (uint32_t)((size + 2)<< 16);
	I2C1->CR2 |= (1U << 25);				//Autoend
	I2C1->CR2 |= (1U << 13);				//Generate start

	while(!(I2C1->ISR & (1 << 1)));			//high address
	I2C1->TXDR = (uint32_t)(memAddr >> 8);

	while(!(I2C1->ISR & (1 << 1)));			//low address
	I2C1->TXDR = (uint32_t)(memAddr & 0xFF);

	while(size){
		while(!(I2C1->ISR & (1 << 1)));		//is flag busy
		I2C1->TXDR = (*data++);				//send data
		size--;
	}
}

void read_eeprom(uint8_t devAddr,uint16_t memAddr, uint16_t* data, int size){
	//Send address and register to read
	I2C1->CR2 = 0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (2U << 16);				//Number of bytes
	I2C1->CR2 |= (1U << 13);				//Generate Start

	while(!(I2C1->ISR & (1 << 1)));
	I2C1->TXDR = (uint32_t)(memAddr >> 8);

	while(!(I2C1->ISR & (1 << 1)));
	I2C1->TXDR = (uint32_t)(memAddr & 0xFF);

	while(!(I2C1->ISR & (1 << 6)));			//is transmission complete

	//read data
	I2C1->CR2 = 0;
	I2C1->CR2 |= (uint32_t)(devAddr << 1);
	I2C1->CR2 |= (1U << 10);				//Read mode
	I2C1->CR2 |= (uint32_t)(size << 16);	//Number of bytes
	I2C1->CR2 |= (1U << 25);				//AUTOEND
	I2C1->CR2 |= (1U << 13);				//Generate start

	while(size)
	{
		while(!(I2C1->ISR & (1 << 2)));
		(*data++) = (uint8_t)I2C1->RXDR;
		size--;
	}
}
