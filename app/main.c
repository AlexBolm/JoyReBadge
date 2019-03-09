//*****************************************************************************
//
//! \file main.c
//! \brief main application
//! \version 1.0.0.0
//! \date $Creat_time$
//! \author $Creat_author$
//! \copy
//!
//! Copyright (c) 2014 CooCox.  All rights reserved.
//
//! \addtogroup project
//! @{
//! \addtogroup main
//! @{
//*****************************************************************************
//							Project info
//*****************************************************************************
/*1)7x16 LEDs (matrix)
 *2)8 RGB LEDs
 *3)2 capacitive touch buttons
 *
 */
//*****************************************************************************
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
/*
 * ########  PIO  ###########
 * PA0 - lipo voltage ADC
 * PA1-PA7 - catodes of LED MATRIX
 * PA8 - RGB [R] catode
 * PA9 - RGB [G] catode
 * PA10 - RGB [B] catode
 * PA11 - capative touch button left
 * PA12 - capative touch button right
 * PA13 - SWDIO
 * PA14 - SWCLK
 *
 * PB0 - test pin
 * PB1 - test pin
 * PB2 - BOOT1
 * PB3 - test pin
 * PB4 - RCK (74HC595)
 * PB5 - SER (74HC595)
 * PB6 - SCK (74HC595)
 * PB7 - SCL (74HC595)
 * PB8-PB15 - RGB anodes
 *
 * PC13 - test
 * PC14 - test
 * PC15 - test
 */

#define DEBUG_ON GPIO_SetBits(GPIOB,GPIO_Pin_0)
#define DEBUG_OFF GPIO_ResetBits(GPIOB,GPIO_Pin_0)


#define LEDS 112
#define FRONT_LEDS 108
#define LED_START (LEDS>>1)
#define RBG_LEDS 8
#define LEDS_A 16
#define LEDS_C 7

#define RED 0
#define GREEN 1
#define BLUE 2

#define RGB_LEVELS 10

#define BUTT_MIN_PRESS 5
#define BUTT_LONG_PRESS 80

#define MAIN_CONTROL_TIMER 100//in Hz

//EVENT
#define EVENT_LED_REFRES		0
#define EVENT_RGB_LED_REFRESH	1

#define EVENT_L_BUT_TAP 		2
#define EVENT_L_BUT_PRESS 		3
#define EVENT_L_BUT_LONG_PRESS 	4

#define EVENT_R_BUT_TAP 		5
#define EVENT_R_BUT_PRESS 		6
#define EVENT_R_BUT_LONG_PRESS 	7

#define EVENT_BATT_CHECK 		8
#define EVENT_BATT_LOW			9
#define EVENT_SHOW_VOLTAGE		10

#define ADC_CNT 8
#define ADC_MIN_VAL 2170
#define ADC_MAX_VAL 2605
#define SHOW_VOLT_TIME 8 //sec


//######### MODES ##############
#define MAX_MODES 10
#define INTRO			0
#define SIMPL_BLINK_ALL 1
#define INTERLEAVED 	2//blink 0-1-0-1-0-1-0-1...
#define BIG_INTERLEAVED	3//blink 16-20-16-20-16-20...
#define SIMPLE_UP 		4
#define CORNER_TO_CENTR	5
#define BRSNCH_TOBRSNCH 6//20 then 16 then 20 then 16
#define BRSNCH_HLFBRSNC 7//10 then 8 then 10 then 8
#define BRANCH_SWITCH	8//switch between 3 branchs
#define HOLE1_MOVING	9//1 dead led run over all leds
#define MORZE			10//something in circle


//########## VARIABLES AND STRUCT ###########

struct St_System {
	uint32_t event_reg;
	uint8_t mode;//
	uint8_t show_reserve;//flag for active massive
	uint8_t array_ready;//set to 1 when new array is ready
	uint32_t speed;//speed of changing leds (depend on Timer2)
	uint32_t time;
	uint32_t led_timer;

};
volatile struct St_System Sys;

uint8_t leds[LEDS]={0};
uint8_t leds_reserv[LEDS]={0};
const uint8_t morze_code[LEDS]={
1,0,1,0,1,
0,0,0,
1,
0,0,0,
1,1,1,0,1,
0,0,0,
1,1,1,0,1,0,1,
0,0,0,0,0,0,0,
1,1,1,0,1,
0,0,0,
1,0,1,0,1,1,1,
0,0,0,
1,1,1,0,1,0,1,
0,0,0,
1,
0,0,0,
1,0,1,0,1,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0,
0,0,0,0,0,0,0};

uint8_t RGB_leds[RBG_LEDS][3]={0};
uint8_t RGB_leds_reserv[RBG_LEDS][3]={0};

uint16_t adc_data[ADC_CNT]={0};

uint8_t buttons=0;





void init_clocks()
{
	/*
	 * clock settings
	 * HSE = quarz = 8MHz
	 * PLL mult x4 =>  SYSCLK = 32MHz
	 * AHB=1 =>        HCLK = 32MHz
	 * APB1=4 =>       TIM2-4 = 16MHz(8*2)
	 * APB2=1 =>       TIM1,GPIO = 32MHz
	 * ADCPRESC=8 =>   ADCCLK = 4MHz
	 */
	  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

		  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
		  /* Enable HSE */

		  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
		  RCC->CR &= ~((uint32_t)RCC_CR_HSION);

		  /* Wait till HSE is ready and if Time out is reached exit */
		  do
		  {
		    HSEStatus = RCC->CR & RCC_CR_HSERDY;
		    StartUpCounter++;
		  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

		  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
		  {
		    HSEStatus = (uint32_t)0x01;
		  }else{
		    HSEStatus = (uint32_t)0x00;
		  }

		  RCC->CR |= ((uint32_t)RCC_CR_CSSON);// protector - with auto swithcing on HSI if HSE is bad


		  if (HSEStatus == (uint32_t)0x01)
		  {
			RCC->CFGR &= (uint32_t)~(RCC_CFGR_SW_1);
			RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSE;//HSE select as SYSCLK
		    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_HSE)
		    {
		    }

		    /* Enable Prefetch Buffer */
		//    FLASH->ACR |= FLASH_ACR_PRFTBE;

		    /* Flash 0 wait state */
		//    FLASH->ACR &= ~(FLASH_ACR_LATENCY);
			FLASH->ACR |= FLASH_ACR_PRFTBE;
		    FLASH->ACR &= ~FLASH_ACR_HLFCYA;
			FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_2;

		    /* PLL OFF */
		    RCC->CR &= ~RCC_CR_PLLON;

		    /* PLL mult */
		    //PLL*4
		    RCC->CFGR &=~(RCC_CFGR_PLLMULL);//18..21=0000
		    RCC->CFGR |= RCC_CFGR_PLLMULL4;//19=1
		    //HSE as PLL input
		    RCC->CFGR |= RCC_CFGR_PLLSRC;//16=1

			/* AHB prescaler */
		    RCC->CFGR &= (uint32_t)~RCC_CFGR_HPRE;//7..4=0000 # AHB prescaler = SYSCLK not divided = 32MHz

		    /* APB 1 prescaler */
		    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PPRE1); //10..8=000
		    RCC->CFGR |=(uint32_t)(RCC_CFGR_PPRE1_DIV4);//8,10=1 # HCLK/4 = 8MHz

		    /* APB 2 prescaler */
		    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PPRE2);//11..13=000 # HCLK not divided = 32MHz

		    /* ADC prescaler */
		    RCC->CFGR |= (uint32_t)RCC_CFGR_ADCPRE; //15,14=11 ADC prescaler = 8 = 4MHz

		    /* Select PLL as system clock source */
		    RCC->CFGR &= (uint32_t)~(RCC_CFGR_SW_0);
		    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;//PLL select as SYSCLK

		    /* PLL ON */
		    RCC->CR |= RCC_CR_PLLON;
		    while (!(RCC->CR & RCC_CR_PLLRDY)){

		    }
		    /* Wait till PLL is used as system clock source */
		    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
		    {
		    }

		  }else{ /* If HSE fails to start-up, the application will have wrong clock
		         configuration. User can add here some code to deal with this error */
		  }


}

void init_gpio_a()
{

	GPIO_InitTypeDef porta;

	RCC_APB2PeriphClockCmd	(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_0;
	porta.GPIO_Mode = GPIO_Mode_AIN;
	porta.GPIO_Speed = GPIO_Speed_2MHz;


	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_1;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_2;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_3;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_4;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_5;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_6;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_7;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_8;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_9;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_10;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_11;
	porta.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_12;
	porta.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_13;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_14;
	porta.GPIO_Mode = GPIO_Mode_Out_PP;
	porta.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&porta);

	GPIO_StructInit(&porta);
	porta.GPIO_Pin = GPIO_Pin_15;
	porta.GPIO_Mode = GPIO_Mode_IPD;
	porta.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&porta);


}

void init_gpio_b()
{

	GPIO_InitTypeDef portb;

	RCC_APB2PeriphClockCmd	(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_0;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_1;
	portb.GPIO_Mode = GPIO_Mode_IPD;
	portb.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_2;
	portb.GPIO_Mode = GPIO_Mode_IPD;
	portb.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_3;
	portb.GPIO_Mode = GPIO_Mode_IPD;
	portb.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_4;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_5;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_6;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_7;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_8;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_9;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_10;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_11;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_12;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_13;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_14;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);

	GPIO_StructInit(&portb);
	portb.GPIO_Pin = GPIO_Pin_15;
	portb.GPIO_Mode = GPIO_Mode_Out_PP;
	portb.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&portb);


}

void init_gpio_startup(){
	GPIO_ResetBits(GPIOA,GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);

	GPIO_ResetBits(GPIOB,GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);
	GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
}


void init_adc(){

	//Init clock level
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; //ADC1 clock enable

	ADC1->CR1 |= (ADC_CR1_DISCEN | ADC_CR1_EOCIE); // discont. 1channels mode and interrupt
	ADC1->SQR3 = 0x00;//pin #0
	ADC1->CR2 |= ADC_CR2_ADON;
	//1 sample ~ 20cycles = 10us an 2MHz
	ADC1->SMPR2 |= ADC_SMPR2_SMP0_0;

	ADC1->CR2 |= ADC_CR2_CAL;
	while(ADC1->CR2 & ADC_CR2_CAL){}

}

//button control
void init_timer2(){//100Hz

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->ARR = 20;//0.0005 * 20 = 0.01s // 100
	TIM2->PSC = 8000;//16 000 000 / 8 000 = 2000 => 1/2000 = 0.0005
	TIM2->CR1 = TIM_CR1_URS | TIM_CR1_CEN;//go
}

//led speed
void init_timer3(){//400Hz

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->ARR = 25;//0.0001 * 25 = 0.0025s // 400
	TIM3->PSC = 1600;//16 000 000 / 1600 = 10 000 => 1/10 000 = 0.0001
	TIM3->CR1 = TIM_CR1_URS | TIM_CR1_CEN;//go
}

//rgb led refresh
void init_timer4(){//2KHz

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	TIM4->DIER |= TIM_DIER_UIE;
	TIM4->ARR = 50;//0.00001 * 50 = 0.00050s // ~2k=100Hz x 10level or brightness
	TIM4->PSC = 160;//16 000 000 / 160 = 100 000 => 1/100 000 = 0.00001
	TIM4->CR1 = TIM_CR1_URS | TIM_CR1_CEN;//go
}



//########## FUNCTIONS #############

void send_one(){
	//send 1
	GPIO_SetBits(GPIOB,GPIO_Pin_5);//data up
	GPIO_SetBits(GPIOB,GPIO_Pin_5);//data up

	GPIO_SetBits(GPIOB,GPIO_Pin_6);//clk up
	GPIO_SetBits(GPIOB,GPIO_Pin_6);//clk up

	GPIO_ResetBits(GPIOB,GPIO_Pin_6);//clk down
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);//clk down

	GPIO_ResetBits(GPIOB,GPIO_Pin_5);//data down
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);//data down


}
void send_zero(){
	//send 0
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);//data down
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);//data down


	GPIO_SetBits(GPIOB,GPIO_Pin_6);//clk up
	GPIO_SetBits(GPIOB,GPIO_Pin_6);//clk up


	GPIO_ResetBits(GPIOB,GPIO_Pin_6);//clk down
	GPIO_ResetBits(GPIOB,GPIO_Pin_6);//clk down


	GPIO_ResetBits(GPIOB,GPIO_Pin_5);//data down
	GPIO_ResetBits(GPIOB,GPIO_Pin_5);//data down

}
void send_latch(){
	GPIO_SetBits(GPIOB,GPIO_Pin_4);//latch up
	GPIO_SetBits(GPIOB,GPIO_Pin_4);//latch up


	GPIO_ResetBits(GPIOB,GPIO_Pin_4);//latch down
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);//latch down

}

//~100Hz for all leds
void light_leds(uint8_t led_array[]){
	uint8_t anod_data=0;
	uint8_t catod_data=0;
	static uint8_t leds_row=15;
	static uint8_t led_add=0;
	static uint8_t led_addr_start=LED_START;
//turn off all catodes
	GPIO_ResetBits(GPIOA,(GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7));

//send 74hc595 =anodes
	for (anod_data=0;anod_data<LEDS_A;anod_data++) {//16
		if (anod_data==leds_row) {
			send_one();
		} else {
			send_zero();
		}
	}
	send_latch();

	if (leds_row==0) {
		leds_row=15;
	} else {
		leds_row--;
	}
//send uln2003 = catodes
	for (catod_data=1;catod_data<(LEDS_C+1);catod_data++) {//7
		if (led_array[led_addr_start+led_add]==1) {
			GPIO_SetBits(GPIOA,(1<<catod_data));
		} else {
			GPIO_ResetBits(GPIOA,(1<<catod_data));
		}
		led_add+=8;//7times
	}
	led_add=0;

	if (led_addr_start!=63 && led_addr_start!=7) {
		led_addr_start++;
	} else {
		if (led_addr_start==63) {
			led_addr_start=0;
		} else {
			led_addr_start=LED_START;
			//end of all leds refresh
			if (Sys.array_ready==1) {
				Sys.array_ready=0;
				Sys.show_reserve ^= 0x01;//change array
			}
		}

	}

}

void light_rgb_leds(uint8_t led_array[][3]){
	uint8_t leds=0;
	static uint8_t rgb=0;
	static uint8_t lightlevel=1;

	switch (rgb) {
		case 0:
			for (leds=0;leds<8;leds++) {
				GPIO_SetBits(GPIOB,(0x100<<leds));//set = off
			}
			GPIO_SetBits(GPIOA,GPIO_Pin_8);//r+
			GPIO_ResetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10);//g-b-
				for (leds=0;leds<8;leds++) {
					if (led_array[leds][0]>lightlevel){
						GPIO_ResetBits(GPIOB,(0x100<<leds));
					} else {
						GPIO_SetBits(GPIOB,(0x100<<leds));
					}
				}
				rgb++;
			break;
		case 1:
			for (leds=0;leds<8;leds++) {
				GPIO_SetBits(GPIOB,(0x100<<leds));//set = off
			}
			GPIO_SetBits(GPIOA,GPIO_Pin_9);//g+
			GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_10);//r-b-
				for (leds=0;leds<8;leds++) {
					if (led_array[leds][1]>lightlevel){
						GPIO_ResetBits(GPIOB,(0x100<<leds));
					} else {
						GPIO_SetBits(GPIOB,(0x100<<leds));
					}
				}
				rgb++;
			break;
		case 2:
			for (leds=0;leds<8;leds++) {
				GPIO_SetBits(GPIOB,(0x100<<leds));//set = off
			}
			GPIO_SetBits(GPIOA,GPIO_Pin_10);//b+
			GPIO_ResetBits(GPIOA,GPIO_Pin_8|GPIO_Pin_9);//r-g-
				for (leds=0;leds<8;leds++) {
					if (led_array[leds][2]>lightlevel){
						GPIO_ResetBits(GPIOB,(0x100<<leds));
					} else {
						GPIO_SetBits(GPIOB,(0x100<<leds));
					}
				}
				rgb=0;
				if (lightlevel<RGB_LEVELS) {//scans all brightnesses 1 to RGB_LEVELS
					lightlevel++;
				} else {
					lightlevel=1;
				}
			break;

	}
}


void clear_led_arrs(){
	uint8_t i=0;

	for (i=0;i<FRONT_LEDS;i++) {
		leds[i]=0;
		leds_reserv[i]=0;
	}
	for (i=0;i<RBG_LEDS;i++) {
		RGB_leds[i][RED]=0;
		RGB_leds[i][GREEN]=0;
		RGB_leds[i][BLUE]=0;
	}
}
//########## INTERRUPTS ###########

void ADC1_2_IRQHandler(void){
	static uint8_t cnt=0;
	static startup_flag=1;

	if (ADC1->SR & ADC_SR_EOC) {
		adc_data[cnt]=ADC1->DR;//read and clear EOC
		if (cnt<(ADC_CNT-1)) {
			cnt++;
		} else {
			cnt=0;
			startup_flag=0;
			Sys.event_reg |= (1<<EVENT_BATT_CHECK);
		}

		if (startup_flag==1) {
			ADC1->CR2 |= ADC_CR2_ADON;
		}
	}
}

//~2kHz
void TIM4_IRQHandler() {
	if (TIM4->SR & TIM_SR_UIF) {
			DEBUG_ON;

			light_rgb_leds(RGB_leds);


			if (Sys.show_reserve==1) {
				light_leds(leds_reserv);
			} else {
				light_leds(leds);
			}
			DEBUG_OFF;
			TIM4->SR &= ~TIM_SR_UIF;
	}
}


//100Hz
void TIM2_IRQHandler() {
	static uint8_t timer=0;
	static uint8_t button_left_cnt=0;
	static uint8_t button_right_cnt=0;

	if (TIM2->SR & TIM_SR_UIF) {
		if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_11)) {
			if (button_left_cnt<255) {
				button_left_cnt++;
			} else {
				Sys.event_reg |= (1<<EVENT_L_BUT_LONG_PRESS);
			}
		} else {
			if (button_left_cnt>1) {
				if (Sys.event_reg & (1<<EVENT_L_BUT_LONG_PRESS)) {// long press event
					Sys.event_reg &= ~(1<<EVENT_L_BUT_LONG_PRESS);
				} else { //TAP OR PRESS
					if (button_left_cnt>BUTT_MIN_PRESS && button_left_cnt<BUTT_LONG_PRESS) {
						Sys.event_reg |= (1<<EVENT_L_BUT_TAP);
					} else {
						if (button_left_cnt>BUTT_LONG_PRESS) {
							Sys.event_reg |= (1<<EVENT_L_BUT_PRESS);
						}
					}
				}
			}
			button_left_cnt=0;
		}

		if (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)) {
			if (button_right_cnt<255) {
				button_right_cnt++;
			} else {
				Sys.event_reg |= (1<<EVENT_R_BUT_LONG_PRESS);
			}
		} else {
			if (button_right_cnt>1) {
				if (Sys.event_reg & (1<<EVENT_R_BUT_LONG_PRESS)) {// long press event
					Sys.event_reg &= ~(1<<EVENT_R_BUT_LONG_PRESS);
				} else { //TAP OR PRESS
					if (button_right_cnt>BUTT_MIN_PRESS && button_right_cnt<BUTT_LONG_PRESS) {
						Sys.event_reg |= (1<<EVENT_R_BUT_TAP);
					} else {
						if (button_right_cnt>BUTT_LONG_PRESS) {
							Sys.event_reg |= (1<<EVENT_R_BUT_PRESS);
						}
					}
				}
			}
			button_right_cnt=0;
		}

	//check voltage
		if (timer<100) {//~1 sec
			timer++;
		} else {
			ADC1->CR2 |= ADC_CR2_ADON;
			timer=0;
			Sys.time++;
		}
		TIM2->SR &= ~TIM_SR_UIF;
	}

}

//400hz speed
void TIM3_IRQHandler() {
	if (TIM3->SR & TIM_SR_UIF) {
		//time for speed of changing leds
		if (Sys.led_timer > Sys.speed ) {
			Sys.event_reg |= (1<<EVENT_LED_REFRES);
			Sys.led_timer=0;
		} else {
			Sys.led_timer++;
		}

		TIM3->SR &= ~TIM_SR_UIF;
	}

}

/*
 * Programm light up leds in certain mode, witch is chosen by pressing a button
 * all leds route as matrix, and refreshes as matrix in timers interrupts
 *
 * Main while loop changes leds arrays while timers interrupts turn actual GPIO
 * changing leds made with reserve array, coz changing array should not affect active array
 * as a result new leds light up only when all new array is ready
 */

int main(void)
{

	static uint8_t led_tmp=0;
	static uint8_t led_tmp1=0;
	static uint8_t led_tmp2=0;
	static uint8_t led_tmp3=0;

	static uint8_t led_tmp_rgb1=0;
	static uint8_t led_tmp_rgb2=0;
	static uint8_t led_tmp_rgb3=0;

	static uint16_t batt_level=0;
	static uint16_t batt_tmp=0;

	static uint16_t speed_tmp=0;

	uint32_t temp=0;
	uint8_t i,j,k,u=0;
	uint32_t delay_cnt=0;

	Sys.event_reg=0;
	Sys.mode=0;
	Sys.show_reserve=0;
	Sys.array_ready=0;
	Sys.speed=31;
	Sys.led_timer=0;

	//make debug SWD
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

	init_clocks();
	init_gpio_a();
	init_gpio_b();

	init_gpio_startup();

	init_timer4();
	init_timer2();
	init_timer3();

	init_adc();

	NVIC_EnableIRQ (TIM4_IRQn);
	NVIC_EnableIRQ (TIM2_IRQn);
	NVIC_EnableIRQ (TIM3_IRQn);


	NVIC_EnableIRQ (ADC1_2_IRQn);
	__enable_irq();
	ADC1->CR2 |= ADC_CR2_ADON;

	GPIO_SetBits(GPIOB,GPIO_Pin_7);//74hc595 ready

	//debug
//	GPIO_ResetBits(GPIOB,GPIO_Pin_8);
//	GPIO_SetBits(GPIOA,GPIO_Pin_10);
    while(1)
    {
    	//we want to see voltage

    	if ((Sys.event_reg & (1<<EVENT_R_BUT_LONG_PRESS)) && (!(Sys.event_reg&(1<<EVENT_SHOW_VOLTAGE)))) {
    		Sys.event_reg|=(1<<EVENT_SHOW_VOLTAGE);

    		//max 4.2v=2605 ADC_MAX_VAL
    		//min 3.5v=2171 ADC_MIN_VAL
			if (batt_level>(ADC_MIN_VAL+4)) {
				batt_tmp=(batt_level-ADC_MIN_VAL)>>2;
			} else {
				batt_tmp=1;
			}
			temp=Sys.time;
    		speed_tmp=Sys.speed;
    		Sys.speed=21;
    		led_tmp=0;
    		clear_led_arrs();
    	}


    	if ((Sys.event_reg & (1<<EVENT_BATT_LOW)) || (Sys.event_reg & (1<<EVENT_SHOW_VOLTAGE))) {
    		if (Sys.event_reg & (1<<EVENT_BATT_LOW)) {
    			RGB_leds[0][RED]=7;
    			RGB_leds[1][RED]=7;
    		}

    		if ((Sys.event_reg & (1<<EVENT_SHOW_VOLTAGE)) && (Sys.time>temp)) {//1sec delay for refreshing leds
        		//battary showing
        		if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
					if (Sys.show_reserve==1){
						for (i=0;i<FRONT_LEDS;i++) {
							if (led_tmp>=i) {
								leds[i]=1;
							} else {
								leds[i]=0;
							}
						}
					} else {
						for (i=0;i<FRONT_LEDS;i++) {
							if (led_tmp>=i) {
								leds_reserv[i]=1;
							} else {
								leds_reserv[i]=0;
							}
						}
					}

					if (led_tmp<batt_tmp) {
						led_tmp++;
					}

					//new array ready
					Sys.array_ready=1;
					Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
				}

        		//end showing voltage
        		if (Sys.time>(temp+SHOW_VOLT_TIME)) {
        			Sys.event_reg &= ~(1<<EVENT_SHOW_VOLTAGE);
        			clear_led_arrs();
					led_tmp=0;
					Sys.speed=speed_tmp;
        		}
			}

    	} else {//MAIN PROGRAMM
		//fast click == speed change
				if (((Sys.event_reg & (1<<EVENT_R_BUT_TAP)) || (Sys.event_reg & (1<<EVENT_L_BUT_TAP))) && (Sys.event_reg & (1<<EVENT_LED_REFRES))){
					if ((Sys.event_reg & (1<<EVENT_R_BUT_TAP))) {
						if (Sys.speed>6) {
							Sys.speed-=5;
							Sys.led_timer=0;
						}
						Sys.event_reg &= ~(1<<EVENT_R_BUT_TAP);
					} else {
						if (Sys.speed<250) {
							Sys.speed+=5;
							Sys.led_timer=0;
						}
						Sys.event_reg &= ~(1<<EVENT_L_BUT_TAP);
					}
				}
		//long click == mode switch
				if (((Sys.event_reg & (1<<EVENT_R_BUT_PRESS)) || (Sys.event_reg & (1<<EVENT_L_BUT_PRESS))) && (Sys.event_reg & (1<<EVENT_LED_REFRES))){
					if ((Sys.event_reg & (1<<EVENT_R_BUT_PRESS))) {
						led_tmp=0;
						led_tmp1=0;
						led_tmp2=0;
						led_tmp3=0;

						if (Sys.mode<MAX_MODES) {
							Sys.mode++;
						} else {
							Sys.mode=0;
						}

						clear_led_arrs();
						Sys.event_reg &= ~(1<<EVENT_R_BUT_PRESS);
					} else { //LEDFT BUTTON PRESSED
						led_tmp=0;
						led_tmp1=0;
						led_tmp2=0;
						led_tmp3=0;
						if (Sys.mode>0) {
							Sys.mode--;
						} else {
							Sys.mode=MAX_MODES;
						}

						clear_led_arrs();
						Sys.event_reg &= ~(1<<EVENT_L_BUT_PRESS);
					}
				}

				//RGB
				if ((Sys.event_reg & (1<<EVENT_LED_REFRES)) && Sys.mode==MORZE && !(Sys.event_reg & (1<<EVENT_BATT_LOW))) {//set depending on speed
					RGB_leds[led_tmp_rgb1][led_tmp_rgb3]=led_tmp_rgb2;

					if (led_tmp_rgb1<(RBG_LEDS-1)) {
						led_tmp_rgb1++;
					} else {
						led_tmp_rgb1=0;
					}
					if (led_tmp_rgb2<(RGB_LEVELS-5)) {
						led_tmp_rgb2++;
					} else {
						led_tmp_rgb2=0;
							if (led_tmp_rgb3<2) {
								led_tmp_rgb3++;
							} else {
								led_tmp_rgb3=0;
							}
					}
				}


				//when change mode always change led_tmp
				switch (Sys.mode) {

				case INTRO:
					if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
		//					if (Sys.show_reserve==1){
							switch(led_tmp){

							case 0://smile
								RGB_leds[4][BLUE]=led_tmp1;
								RGB_leds[5][BLUE]=led_tmp1;
								if (led_tmp1<5) {
									led_tmp1++;
								} else {
									led_tmp1=0;
									led_tmp++;
								}
								break;
							case 1:
								RGB_leds[3][BLUE]=led_tmp1;
								RGB_leds[6][BLUE]=led_tmp1;
								if (led_tmp1<5) {
									led_tmp1++;
								} else {
									led_tmp1=0;
									led_tmp++;
								}
								break;
							case 2:
								RGB_leds[2][BLUE]=led_tmp1;
								RGB_leds[7][BLUE]=led_tmp1;
								if (led_tmp1<5) {
									led_tmp1++;
								} else {
									led_tmp1=0;
									led_tmp++;
									led_tmp2=0;
								}
								break;
							case 3:
								if (led_tmp2==0) {
									RGB_leds[0][RED]=led_tmp1;
									RGB_leds[1][GREEN]=led_tmp1;
									if (led_tmp1<7) {
										led_tmp1++;
									} else {
										//led_tmp1=0;
										led_tmp++;
									}
								} else {
									RGB_leds[0][GREEN]=led_tmp1;
									RGB_leds[1][RED]=led_tmp1;
									if (led_tmp1<7) {
										led_tmp1++;
									} else {
										//led_tmp1=0;
										led_tmp++;
									}
								}
								break;
							case 4:
								if (led_tmp2==0) {
									RGB_leds[0][RED]=led_tmp1;
									RGB_leds[1][GREEN]=led_tmp1;
									if (led_tmp1>1) {
										led_tmp1--;
									} else {
										led_tmp1=0;
										led_tmp=3;
										led_tmp2^=0x01;
									}
								} else {
									RGB_leds[0][GREEN]=led_tmp1;
									RGB_leds[1][RED]=led_tmp1;
									if (led_tmp1>1) {
										led_tmp1--;
									} else {
										led_tmp1=0;
										led_tmp=3;
										led_tmp2^=0x01;
									}
								}
								break;
							}
		//					}

						if (led_tmp>2){
							if (Sys.show_reserve==1){
								for (i = 0; i < 18; i++) {
									if (i<=led_tmp3) {
										leds[10+i]=1;
										leds[46+i]=1;
										leds[82+i]=1;

										leds[45-i]=1;
										leds[81-i]=1;
										if (i<10){
											leds[9-i]=1;
										} else {
											leds[107-(i-10)]=1;
										}
									} else {
										leds[10+i]=0;
										leds[46+i]=0;
										leds[82+i]=0;

										leds[45-i]=0;
										leds[81-i]=0;
										if (i<10){
											leds[9-i]=0;
										} else {
											leds[107-(i-10)]=0;
										}
									}
								}
							} else {
								for (i = 0; i < 18; i++) {
									if (i<=led_tmp3) {
										leds_reserv[10+i]=1;
										leds_reserv[46+i]=1;
										leds_reserv[82+i]=1;

										leds_reserv[45-i]=1;
										leds_reserv[81-i]=1;
										if (i<10){
											leds_reserv[9-i]=1;
										} else {
											leds_reserv[107-(i-10)]=1;
										}
									} else {
										leds_reserv[10+i]=0;
										leds_reserv[46+i]=0;
										leds_reserv[82+i]=0;

										leds_reserv[45-i]=0;
										leds_reserv[81-i]=0;
										if (i<10){
											leds_reserv[9-i]=0;
										} else {
											leds_reserv[107-(i-10)]=0;
										}
									}
								}
							}

							if (led_tmp3<18) {
								led_tmp3++;
							} else {
								led_tmp3=0;
							}
							//new array ready
							Sys.array_ready=1;
						}


						Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
					}
					break;

					case SIMPLE_UP:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								for (i=0;i<FRONT_LEDS;i++) {
									if (led_tmp>=i) {
										leds[i]=1;
									} else {
										leds[i]=0;
									}
								}
							} else {
								for (i=0;i<FRONT_LEDS;i++) {
									if (led_tmp>=i) {
										leds_reserv[i]=1;
									} else {
										leds_reserv[i]=0;
									}
								}
							}
							if (led_tmp<FRONT_LEDS) {
								led_tmp++;
							} else {
								led_tmp=0;
							}

							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

					case SIMPL_BLINK_ALL:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								for (i=0;i<FRONT_LEDS;i++) {
									if (led_tmp==0) {
										leds[i]=0;
									} else {
										leds[i]=1;
									}
								}
							} else {
								for (i=0;i<FRONT_LEDS;i++) {
									if (led_tmp==0) {
										leds_reserv[i]=0;
									} else {
										leds_reserv[i]=1;
									}
								}
							}
							led_tmp ^= 0x01;
							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

					case INTERLEAVED:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								for (i=0;i<(FRONT_LEDS);i+=2) {
									if (led_tmp==0) {
										leds[i]=0;
										leds[i+1]=1;
									} else {
										leds[i]=1;
										leds[i+1]=0;
									}
								}
							} else {
								for (i=0;i<(FRONT_LEDS);i+=2) {
									if (led_tmp==0) {
										leds_reserv[i]=0;
										leds_reserv[i+1]=1;
									} else {
										leds_reserv[i]=1;
										leds_reserv[i+1]=0;
									}
								}
							}
							led_tmp ^= 0x01;
							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

					case BIG_INTERLEAVED:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								for (k=0;k<3;k++) {
									for (i=led_tmp1;i<(led_tmp1+20);i++) {
										leds[i]=1;
									}
									led_tmp1+=20;
									for (j=led_tmp1;j<(led_tmp1+16);j++) {
										leds[j]=0;
									}
									led_tmp1+=16;
								}
								led_tmp1=0;
							} else {
								for (k=0;k<3;k++) {
									for (i=led_tmp1;i<(led_tmp1+20);i++) {
										leds_reserv[i]=0;
									}
									led_tmp1+=20;
									for (j=led_tmp1;j<(led_tmp1+16);j++) {
										leds_reserv[j]=1;
									}
									led_tmp1+=16;
								}
								led_tmp1=0;
							}
							led_tmp ^= 0x01;
							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

		//			case CENTR_TO_CORNER:
		//				if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
		//					if (Sys.show_reserve==1){
		//						for (i = 0; i < 18; i++) {
		//							if (i<=led_tmp) {
		//								leds[10+i]=1;
		//								leds[46+i]=1;
		//								leds[82+i]=1;
		//
		//								leds[45-i]=1;
		//								leds[81-i]=1;
		//								if (i<10){
		//									leds[9-i]=1;
		//								} else {
		//									leds[107-(i-10)]=1;
		//								}
		//							} else {
		//								leds[10+i]=0;
		//								leds[46+i]=0;
		//								leds[82+i]=0;
		//
		//								leds[45-i]=0;
		//								leds[81-i]=0;
		//								if (i<10){
		//									leds[9-i]=0;
		//								} else {
		//									leds[107-(i-10)]=0;
		//								}
		//							}
		//						}
		//					} else {
		//						for (i = 0; i < 18; i++) {
		//							if (i<=led_tmp) {
		//								leds_reserv[10+i]=1;
		//								leds_reserv[46+i]=1;
		//								leds_reserv[82+i]=1;
		//
		//								leds_reserv[45-i]=1;
		//								leds_reserv[81-i]=1;
		//								if (i<10){
		//									leds_reserv[9-i]=1;
		//								} else {
		//									leds_reserv[107-(i-10)]=1;
		//								}
		//							} else {
		//								leds_reserv[10+i]=0;
		//								leds_reserv[46+i]=0;
		//								leds_reserv[82+i]=0;
		//
		//								leds_reserv[45-i]=0;
		//								leds_reserv[81-i]=0;
		//								if (i<10){
		//									leds_reserv[9-i]=0;
		//								} else {
		//									leds_reserv[107-(i-10)]=0;
		//								}
		//							}
		//						}
		//					}
		//
		//					if (led_tmp<18) {
		//						led_tmp++;
		//					} else {
		//						led_tmp=0;
		//					}
		//					//new array ready
		//					Sys.array_ready=1;
		//					Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
		//				}
		//				break;

					case CORNER_TO_CENTR:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								for (i = 0; i < 18; i++) {
									if (i<=led_tmp) {
										leds[28+i]=1;
										leds[64+i]=1;

										leds[27-i]=1;
										leds[63-i]=1;
										leds[99-i]=1;
										if (i<8){
											leds[100+i]=1;
										} else {
											leds[0+(i-8)]=1;
										}
									} else {
										leds[28+i]=0;
										leds[64+i]=0;

										leds[27-i]=0;
										leds[63-i]=0;
										leds[99-i]=0;
										if (i<8){
											leds[100+i]=0;
										} else {
											leds[0+(i-8)]=0;
										}
									}
								}
							} else {
								for (i = 0; i < 18; i++) {
									if (i<=led_tmp) {
										leds_reserv[28+i]=1;
										leds_reserv[64+i]=1;

										leds_reserv[27-i]=1;
										leds_reserv[63-i]=1;
										leds_reserv[99-i]=1;
										if (i<8){
											leds_reserv[100+i]=1;
										} else {
											leds_reserv[0+(i-8)]=1;
										}
									} else {
										leds_reserv[28+i]=0;
										leds_reserv[64+i]=0;

										leds_reserv[27-i]=0;
										leds_reserv[63-i]=0;
										leds_reserv[99-i]=0;
										if (i<8){
											leds_reserv[100+i]=0;
										} else {
											leds_reserv[(i-8)]=0;
										}
									}
								}
							}

							if (led_tmp<18) {
								led_tmp++;
							} else {
								led_tmp=0;
							}
							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

					case BRSNCH_TOBRSNCH:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								if (led_tmp1==0 && led_tmp2==0){
									led_tmp2=20;
								} else {
									led_tmp1=led_tmp2;
									if (led_tmp==1){
										led_tmp2=led_tmp1+16;
									} else {
										led_tmp2=led_tmp1+20;
									}
								}

								for (i = 0; i < FRONT_LEDS; i++) {
									if (i>=led_tmp1 && i<led_tmp2){
										leds[i]=1;
									} else {
										leds[i]=0;
									}
								}
								if(led_tmp2>100){
									led_tmp2=0;
									led_tmp1=0;
								}
							} else {
								if (led_tmp1==0 && led_tmp2==0){
									led_tmp2=20;
								} else {
									led_tmp1=led_tmp2;
									if (led_tmp==1){
										led_tmp2=led_tmp1+16;
									} else {
										led_tmp2=led_tmp1+20;
									}
								}

								for (i = 0; i < FRONT_LEDS; i++) {
									if (i>=led_tmp1 && i<led_tmp2){
										leds_reserv[i]=1;
									} else {
										leds_reserv[i]=0;
									}
								}
								if(led_tmp2>100){
									led_tmp2=0;
									led_tmp1=0;
								}
							}
							led_tmp^=0x01;

							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

					case BRSNCH_HLFBRSNC:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								if (led_tmp<2) {
									led_tmp=10;
									led_tmp1=led_tmp2;
									led_tmp2+=led_tmp;//0-10,10-20
									led_tmp++;
								} else {
									led_tmp=8;
									led_tmp1=led_tmp2;
									led_tmp2+=led_tmp;
									if (led_tmp<3) {
										led_tmp++;
									} else {
										led_tmp=0;
									}
								}
								for (i = 0; i < FRONT_LEDS; i++) {
									if (i>=led_tmp1 && i<led_tmp2){
										leds[i]=1;
									} else {
										leds[i]=0;
									}
								}
								if (led_tmp2>105){
									led_tmp1=0;
									led_tmp2=0;
								}
							} else {
								if (led_tmp<2) {
									led_tmp=10;
									led_tmp1=led_tmp2;
									led_tmp2+=led_tmp;//0-10,10-20
									led_tmp++;
								} else {
									led_tmp=8;
									led_tmp1=led_tmp2;
									led_tmp2+=led_tmp;
									if (led_tmp<3) {
										led_tmp++;
									} else {
										led_tmp=0;
									}
								}
								for (i = 0; i < FRONT_LEDS; i++) {
									if (i>=led_tmp1 && i<led_tmp2){
										leds_reserv[i]=1;
									} else {
										leds_reserv[i]=0;
									}
								}
								if (led_tmp2>105){
									led_tmp1=0;
									led_tmp2=0;
								}
							}

							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

					case BRANCH_SWITCH:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								switch (led_tmp) {
									case 0:
										led_tmp1=10;
										led_tmp2=45;
										led_tmp=1;
										break;
									case 1:
										led_tmp1=46;
										led_tmp2=81;
										led_tmp=2;
										break;
									case 2:
										led_tmp1=82;
										led_tmp2=108;
										led_tmp=0;
										break;
									default:
										break;
								}

								for (i = 0; i < FRONT_LEDS; i++) {
									if (i>=led_tmp1 && i<led_tmp2){
										leds[i]=1;
									} else {
										leds[i]=0;
									}
									if (led_tmp==0){
										if (i<10){
											leds[i]=1;
										}
									}
								}
							} else {
								switch (led_tmp) {
									case 0:
										led_tmp1=10;
										led_tmp2=45;
										led_tmp=1;
										break;
									case 1:
										led_tmp1=46;
										led_tmp2=81;
										led_tmp=2;
										break;
									case 2:
										led_tmp1=82;
										led_tmp2=108;
										led_tmp=0;
										break;
									default:
										break;
								}

								for (i = 0; i < FRONT_LEDS; i++) {
									if (i>=led_tmp1 && i<led_tmp2){
										leds_reserv[i]=1;
									} else {
										leds_reserv[i]=0;
									}
									if (led_tmp==0){
										if (i<10){
											leds_reserv[i]=1;
										}
									}
								}

							}

							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
						break;

					case HOLE1_MOVING:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								for (i = 0; i < FRONT_LEDS; i++) {
									if (i==led_tmp){
										leds[i]=0;
									} else {
										leds[i]=1;
									}
								}
							} else {
								for (i = 0; i < FRONT_LEDS; i++) {
									if (i==led_tmp){
										leds_reserv[i]=0;
									} else {
										leds_reserv[i]=1;
									}
								}
							}

							if (led_tmp<(FRONT_LEDS-1)) {
								led_tmp++;
							} else {
								led_tmp=0;
							}
							//new array ready
							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
					break;

					case MORZE:
						if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
							if (Sys.show_reserve==1){
								for (i = 0; i < FRONT_LEDS; i++) {
										if (led_tmp>=i) {
											leds[i]=morze_code[led_tmp-i];
											leds_reserv[i]=morze_code[led_tmp-i];
										} else {
											leds[i]=0;
										}
								}
							} else {
								for (i = 0; i < FRONT_LEDS; i++) {
									if (led_tmp>=i) {
										leds[i]=morze_code[led_tmp-i];
										leds_reserv[i]=morze_code[led_tmp-i];
									} else {
										leds[i]=0;
									}
								}
							}
							if (led_tmp<(FRONT_LEDS-1)) {
								led_tmp++;
							} else {
								if (led_tmp1>100) {//delay
									led_tmp1=0;
									led_tmp=0;
									for (j=0;j< FRONT_LEDS; j++) {
										leds[j]=0;
										leds_reserv[j]=0;
									}
								} else {
									led_tmp1++;
								}
							}


							Sys.array_ready=1;
							Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
						}
					break;

					default:
						break;
				}
    	}


    	if (Sys.event_reg & (1<<EVENT_BATT_CHECK)){
    		for (i=1;i<ADC_CNT;i++) {
    			adc_data[0]+=adc_data[i];
    		}
    		adc_data[0]=adc_data[0]>>3;

    		batt_level=adc_data[0];

    		if (adc_data[0]<(ADC_MIN_VAL)) {
    			Sys.event_reg |= (1<<EVENT_BATT_LOW);
    			clear_led_arrs();
    		} else {
    			Sys.event_reg &= ~(1<<EVENT_BATT_LOW);
    		}
    		Sys.event_reg &= ~(1<<EVENT_BATT_CHECK);
    	}

    }
}


//template
//			case INCR_MOVE_CIRCL:
//				if (Sys.event_reg & (1<<EVENT_LED_REFRES)) {//set depending on speed
//					if (Sys.show_reserve==1){
//
//					} else {
//
//					}
//
//					Sys.array_ready=1;
//					Sys.event_reg &= ~(1<<EVENT_LED_REFRES);
//				}
//				break;
