#ifndef BSP_H
#define BSP_H
#include "stm32g0xx.h"

#define LED_DELAY	1600000U
#define IWDG_PR		    4
#define IWDG_PERIOD		2000000
#define IWDG_CLOCK		(32000UL / 32)
#define IWDG_RLR (IWDG_PERIOD* IWDG_CLOCK / 1000000UL - 1)

void delay(volatile unsigned int);
void delay_ms(volatile uint32_t );
void SysTick_Handler(void);

void BSP_led_init(void);
void BSP_led_set(void);
void BSP_led_clear(void);
void BSP_led_toggle(void);

void BSP_button_init(void);
int BSP_button_read(void);

void init_timer1(void);
void init_timer2(void);
void init_timer3(void);

void BSP_iwdg_init(void);

void BSP_uart_init(uint32_t);
uint8_t uart_rx(void);
void uart_tx(uint8_t);
void print_char(uint8_t);
int _print(int, char*, int);
void print(char*);

void BSP_init_PWM(void);

void BSP_init_ADC(void);

void I2C_init(void);
void write_eeprom(uint8_t, uint16_t ,uint16_t* , int );
void read_eeprom(uint8_t ,uint16_t ,uint16_t* , int );

#endif /* BSP_H_ */
