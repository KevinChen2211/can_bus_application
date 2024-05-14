#include <stm32f10x.h>

#define LED_NUM      			8                   /* Number of user LEDs                */


#define WAKEUP_BUTTON			1  									//PA0;
#define WAKEUP_LED        1<<15
#define USER_BUTTON				1<<7 								//PB7;
#define USER_LED        	1<<14
#define LED9						 	1<<9
#define LED8 							1<<8


const long led_mask[] = { 1<<15, 1<<14, 1<<13, 1<<12, 1<<11, 1<<10, 1<<9, 1<<8 };


int main (void){
	// ----------------------------------------------------------------------------------------
	
	
	// set up clock for ports
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // port a wakeup button
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // port b user button
	RCC->APB2ENR |= RCC_APB2ENR_IOPDEN; // port d CAN1
	RCC->APB2ENR |= RCC_APB2ENR_IOPEEN; // port e LEDs
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // enable ADC peripheral
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Alternate function I/O clock
	
	// LED SETUP
	GPIOE->CRH = 0x33333333; // enable output all LED
	
	// WAKEUP button setup 
	GPIOA->CRL &= ~(0x0F<<0); // reset Configuration and Mode
	GPIOA->CRL |= GPIO_CRL_CNF0_1; // set to input with pull up/pull down
	
	// USER button setup
	GPIOB->CRL &= ~(0x0FU<<28); // reset Configuration and Mode
	GPIOB->CRL |= GPIO_CRL_CNF7_1; // set to input with pull up/pull down


// ---------------------------------------------------------------
	// enable config for can
	// configure GPIO Pins
	
	// CAN receive data output
	GPIOD->CRL &= ~(0x0F<<0);
	GPIOD->CRL |= GPIO_CRL_CNF0_1; // CAN_RX: Input, Pull-up/Pull-down D0

  // CAN transmit data input
  GPIOD->CRL &= ~(0x0F<<4);
	GPIOD->CRL |= 0x03; // CAN_TX: Output 50 Mhz
	GPIOD->CRL |= GPIO_CRL_CNF1_1; // CAN_TX: Push-pul

	/**
	– CAN_RX: Input, Pull-up/Pull-down D0
	– CAN_TX: Output, Push-pul D1
	**/
	
	// set AFIO remap
	AFIO->MAPR   &= 0xFFFF9FFF; // reset CAN remap
	AFIO->MAPR |= AFIO_MAPR_CAN_REMAP; //   set CAN remap, use PD0, PD1
	
	// enable clock for can
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN; 
	// reset can1
	RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
	
	// reset CAN
	CAN1->MCR |= CAN_MCR_RESET;
	// wait for can to be reset
	while (CAN1->MCR & CAN_MCR_RESET);
	// initilization mode CAN
	CAN1->MCR |= CAN_MCR_INRQ;
	
	// set timing on CAN
	
	CAN1->BTR |=  (((0x02) << 24) | ((0x02) << 20) | ((0xB) << 16) | (0x8));
	
	// start CAN
	
	CAN1->MCR &= ~CAN_MCR_INRQ;                      // normal operating mode, reset INRQ
  while (CAN1->MSR & CAN_MCR_INRQ);
	
	// setup Rx
	CAN1->FMR  |=  CAN_FMR_FINIT;
	//deactivate filter
	CAN1->FA1R &=  ~CAN_FA1R_FACT0; // deactivate filter
	// set filter mode
	CAN1->FM1R |= CAN_FM1R_FBM0;
	// set filter scale
	CAN1->FS1R |= CAN_FS1R_FSC0;
	// filter regester 
	CAN1->sFilterRegister[0].FR1 = 0x01A4F2B;
	CAN1->sFilterRegister[0].FR2 = 0x0024FCE;
	
	// activate filter
	CAN1->FA1R |= CAN_FA1R_FACT0;
	
	// reset Initialisation mode for filter banks
	CAN1->FMR &= ~CAN_FMR_FINIT;    

// TODO: set up receiving and sending can messages, Set up ADC??? should be configured correctly and questions about timing



	// ---------------------------------------------------------------
	// code Block
	int wakeupLoop = 1;
	while (wakeupLoop){
		if (GPIOA->IDR & WAKEUP_BUTTON){
			GPIOE->BSRR = WAKEUP_LED;
			wakeupLoop = 0;
			// send WAKEUP LED status (on) VIA CAN
		}else{
			GPIOE->BRR = WAKEUP_LED;
		}
		// send WAKEUP LED status (off) VIA CAN
	}
	
	int userLoop = 1;
	while (userLoop){
		if (GPIOB->IDR & USER_BUTTON){
			GPIOE->BRR = USER_LED;
			// send USER LED status (on) VIA CAN
		}else{
			GPIOE->BSRR = USER_LED;
			userLoop = 0;
		}
		// send USER LED status (off) VIA CAN
	}
	

	
	while (1){
		// send ADC values VIA CAN
		GPIOE->BSRR = 1<<12;
		// receive CAN values 
		// set LED9 based on CAN values
		// set LED8 based on CAN values
	}                 
}

void timer(void){
	RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
  
  GPIOE->CRH |= GPIO_CRH_MODE15; /* 0b11, output, 50 MHz */
  GPIOE->CRH &= ~GPIO_CRH_CNF15; /* 0b00, push-pull */
  	
	// 5Hz timer config.	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable clock
	
	TIM2->CR1 &= ~TIM_CR1_CEN; // disable counter
	TIM2->CR1 |= TIM_CR1_DIR; // make downcounter
	
	TIM2->ARR = 0xFFFF; // 2^16 - 16-bit counter reset value (count DOWN from here)
	
	// set prescaler (assuming 2x 36MHz timer clock)
	// 5 = (2*36e6/(PSC+1))/2^16
	// PSC = (2*36e6/5)/2^16 - 1 = 218;
	TIM2->PSC = 218;
	
	TIM2->CR1 |= TIM_CR1_CEN; // enable counter
}

int main1 (void) {
  int AD_val, i;
  int num = -1; 
  int dir =  1;

  SystemInit();

  /* Setup GPIO for LEDs                                                      */
  RCC->APB2ENR |=  1 <<  6;             /* Enable GPIOE clock                 */
  GPIOE->CRH    = 0x33333333;           /* Configure the GPIO for LEDs        */

  /* Setup and initialize ADC converter                                       */
  RCC->APB2ENR |=  1 <<  9;             /* Enable ADC1 clock                  */
  GPIOC->CRL   &= 0xFFF0FFFF;           /* Configure PC4 as ADC.14 input      */
  ADC1->SQR1    = 0x00000000;           /* Regular channel 1 conversion       */
  ADC1->SQR2    = 0x00000000;           /* Clear register                     */
  ADC1->SQR3    = 14 <<  0;             /* SQ1 = channel 14                   */
  ADC1->SMPR1   =  5 << 12;             /* Channel 14 sample time is 55.5 cyc */
  ADC1->SMPR2   = 0x00000000;           /* Clear register                     */
  ADC1->CR1     =  1 <<  8;             /* Scan mode on                       */
  ADC1->CR2     = (1 << 20) |           /* Enable external trigger            */
                  (7 << 17) |           /* EXTSEL = SWSTART                   */
                  (1 <<  1) |           /* Continuous conversion              */
                  (1 <<  0) ;           /* ADC enable                         */
  ADC1->CR2    |=  1 <<  3;             /* Initialize calibration registers   */
  while (ADC1->CR2 & (1 << 3));         /* Wait for initialization to finish  */
  ADC1->CR2    |=  1 <<  2;             /* Start calibration                  */
  while (ADC1->CR2 & (1 << 2));         /* Wait for calibration to finish     */
  ADC1->CR2    |=  1 << 22;             /* Start first conversion             */ 

  for (;;) {                            /* Loop forever                       */
    if (ADC1->SR & (1 << 1)) {          /* If conversion has finished         */
      AD_val = ADC1->DR & 0x0FFF;       /* Read AD converted value            */
      ADC1->CR2 |= 1 << 22;             /* Start new conversion               */ 
    }

    /* Calculate 'num': 0, 1, ... , LED_NUM-1, LED_NUM-1, ... , 1, 0, 0, ...  */
    num += dir;
    if (num >= LED_NUM) { dir = -1; num = LED_NUM-1; } 
    else if   (num < 0) { dir =  1; num = 0;         }

    GPIOE->BSRR = led_mask[num];        /* Turn LED on                        */
    for (i = 0; i < ((AD_val << 8) + 100000); i++);
    GPIOE->BSRR = led_mask[num] << 16;  /* Turn LED off                       */
  }
}
