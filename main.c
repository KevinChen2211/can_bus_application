#include <stm32f10x.h>

#define LED_NUM 8 /* Number of user LEDs                */

#define WAKEUP_BUTTON 		1 // PA0;
#define WAKEUP_LED 				1 << 15
#define USER_BUTTON 			1 << 7 // PB7;
#define USER_LED 					1 << 14
#define LED9 							1 << 9
#define LED8 							1 << 8
#define LED9CANID 				0x0024FCE
#define LED8CANID         0x01A4F2B

const long led_mask[] = {1 << 15, 1 << 14, 1 << 13, 1 << 12, 1 << 11, 1 << 10, 1 << 9, 1 << 8};
unsigned int  CAN_TxRdy = 0;
unsigned int  CAN_RxRdy = 0;

typedef struct  {
  unsigned int   id;                 // 29 bit identifier
  unsigned char  data[8];            // Data field
  unsigned char  len;                // Length of data field in bytes
} CAN_msg;

CAN_msg CAN_RxMsg;
CAN_msg CAN_RxMsg2;



void check_message (void);
void CAN_rdMsg (CAN_msg *msg, CAN_msg *msg2);
void CAN_waitReady (void);
void turn_On_LED(CAN_msg *msg);
void timerSetup(void);
void check_Send_Message(void);
void send_Can_Message(CAN_msg *msg);
void createCanMessage(int id, char data[8], char len);
	

int main(void)
{
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
	GPIOA->CRL &= ~(0x0F << 0);	   // reset Configuration and Mode
	GPIOA->CRL |= GPIO_CRL_CNF0_1; // set to input with pull up/pull down

	// USER button setup
	GPIOB->CRL &= ~(0x0FU << 28);  // reset Configuration and Mode
	GPIOB->CRL |= GPIO_CRL_CNF7_1; // set to input with pull up/pull down
	// ---------------------------------------------------------------
	// ADC
	RCC->CFGR	&= ~RCC_CFGR_ADCPRE;
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // divide clock down to 12MHz (72/6)
	// setup for continuous conversion
	ADC1->CR2 |= ADC_CR2_CONT;
	ADC1->SQR1 &= ~ADC_SQR1_L; // clear to 0x0 = single channel conversion
	ADC1->SQR3 = 0x0000000E; // E = Channel 14 (hex) - PC.4
	
	ADC1->CR2 |= ADC_CR2_ADON; // turn on conversion
	//while (ADC1->CR2 & ADC_CR2_ADON);
	ADC1->CR2 |= ADC_CR2_ADON; // start conversion
	
	
	// ---------------------------------------------------------------
	// enable config for can
	// configure GPIO Pins

	// CAN receive data output
	GPIOD->CRL &= ~(0x0F << 0);
	GPIOD->CRL |= GPIO_CRL_CNF0_1; // CAN_RX: Input, Pull-up/Pull-down D0

	// CAN transmit data input
	GPIOD->CRL &= ~(0x0F << 4);
	GPIOD->CRL |= 0x03 << 4;			   // CAN_TX: Output 50 Mhz
	GPIOD->CRL |= GPIO_CRL_CNF1_1; // CAN_TX: Push-pul

	// set AFIO remap
	AFIO->MAPR &= 0xFFFF9FFF;		   // reset CAN remap
	AFIO->MAPR |= AFIO_MAPR_CAN_REMAP; //   set CAN remap, use PD0, PD1

	// enable clock for can
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	// reset can1
	__ASM("NOP"); __ASM("NOP"); 
	RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
	__ASM("NOP"); __ASM("NOP"); 
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_CAN1RST);
	__ASM("NOP"); __ASM("NOP"); 
	// reset CAN
	CAN1->MCR |= CAN_MCR_RESET;
	// wait for can to be reset
	//while (CAN1->MCR & CAN_MCR_RESET);
	CAN1->MCR &= ~CAN_MCR_RESET;
	CAN1->MCR &= ~CAN_MCR_SLEEP;
	// initilization mode CAN
	CAN1->MCR |= CAN_MCR_INRQ;

	// set timing on CAN

	CAN1->BTR |= (((0x02) << 24) | ((0x02) << 20) | ((0xB) << 16) | (0x8));

	// start CAN

	CAN1->MCR &= ~CAN_MCR_INRQ; // normal operating mode, reset INRQ
	//while (CAN1->MSR & CAN_MCR_INRQ);


	// setup Rx
	CAN1->FMR |= CAN_FMR_FINIT;
	// deactivate filter
	CAN1->FA1R &= ~CAN_FA1R_FACT0; // deactivate filter
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


	// ---------------------------------------------------------------
	// code Block
	int wakeupLoop = 1;
	while (wakeupLoop)
	{
		char  data[8] = {0};
		if (GPIOA->IDR & WAKEUP_BUTTON)
		{
			GPIOE->BSRR = WAKEUP_LED;
			wakeupLoop = 0;
			// send WAKEUP LED status (on) VIA CAN
			data[0] = 0x01;
			createCanMessage(0x0002BEF, data, 0x01);
		}
		else
		{
			GPIOE->BRR = WAKEUP_LED;
			// send WAKEUP LED status (off) VIA CAN
			data[0] = 0x00;
			createCanMessage(0x0002BEF, data, 0x01);
		}
	}

	int userLoop = 1;
	while (userLoop)
	{
		char  data[8] = {0};
		if (GPIOB->IDR & USER_BUTTON)
		{
			GPIOE->BRR = USER_LED;
			// send USER LED status (on) VIA CAN
			data[0] = 0x00;
			createCanMessage(0x01AEFCA, data, 0x01);
		}
		else
		{
			GPIOE->BSRR = USER_LED;
			userLoop = 0;
			// send USER LED status (off) VIA CAN
			data[0] = 0x01;
			createCanMessage(0x01AEFCA, data, 0x01);
		}
	}
	CAN_waitReady();
	timerSetup();
	while (1)
	{
		if (TIM2->SR & TIM_SR_UIF) {
			// send ADC values VIA CAN
			unsigned int adc = ADC1->DR;
			char  data[8] = {0};
			data[0] = (char)(adc & 0xFF); // Lower byte
			data[1] = (char)((adc >> 8) & 0xFF);
			data[2] = (char)((adc >> 16) & 0xFF);
			data[3] = (char)((adc >> 24) & 0xFF); // Upper byte
			createCanMessage(0xBADCAFE, data, 0x04);

			// receive CAN values
			check_message();
			if(CAN_RxRdy){
				// set LED9 based on CAN values
				// set LED8 based on CAN values
				turn_On_LED(&CAN_RxMsg);
				turn_On_LED(&CAN_RxMsg2);
				
			}
			// clear flag
			TIM2->SR &= ~TIM_SR_UIF;
		}
	}
}



void createCanMessage(int id, char data[8], char len){
	CAN_msg CAN_TxMsg;
	CAN_TxMsg.id = id;
	CAN_TxMsg.data[0] = data[0];
	CAN_TxMsg.data[1] = data[1];
	CAN_TxMsg.data[2] = data[2];
	CAN_TxMsg.data[3] = data[3];
	CAN_TxMsg.len = len;
	send_Can_Message(&CAN_TxMsg);
	
}
void turn_On_LED(CAN_msg *msg){
	switch(msg->id){
		case LED8CANID:
			if(msg->data[0]){
				GPIOE->BSRR = LED8;
			}else{
				GPIOE->BRR = LED8;
			}
			break;
		case LED9CANID:
			if(msg->data[0]){
				GPIOE->BSRR = LED9;
			}else{
				GPIOE->BRR = LED9;
			}
			break;
	}
}

void check_message (void) {

  if (CAN1->RF0R & CAN_RF0R_FMP0) {			      // message pending ?
		CAN_rdMsg (&CAN_RxMsg, &CAN_RxMsg2);      // read the message

    CAN_RxRdy = 1;                            // set receive flag
  }
}

void check_Send_Message(void) {
	if (CAN1->TSR & CAN_TSR_RQCP0) {                 // request completed mbx 0
    CAN1->TSR |= CAN_TSR_RQCP0;                    // reset request complete mbx 0
	
	CAN_TxRdy = 1; 
  }
}

void send_Can_Message(CAN_msg *msg) {
	check_Send_Message();
	CAN1->sTxMailBox[0].TIR &= ~(0xFFFFFFF << 3);
	CAN1->sTxMailBox[0].TIR |= (unsigned int)(msg->id <<  3);
	CAN1->sTxMailBox[0].TIR |= (unsigned int)(1 <<  2);
	CAN1->sTxMailBox[0].TIR &= ~(0x1 << 1);
	CAN1->sTxMailBox[0].TDLR = (((unsigned int)msg->data[3] << 24) | 
                             ((unsigned int)msg->data[2] << 16) |
                             ((unsigned int)msg->data[1] <<  8) | 
                             ((unsigned int)msg->data[0])        );
	CAN1->sTxMailBox[0].TDTR &= ~CAN_TDT0R_DLC;
  CAN1->sTxMailBox[0].TDTR |=  (msg->len & CAN_TDT0R_DLC);
	CAN1->sTxMailBox[0].TIR |=  CAN_TI0R_TXRQ;
  while(!(CAN1->TSR & CAN_TSR_TXOK0)){}
}

void CAN_rdMsg (CAN_msg *msg, CAN_msg *msg2)  {
	//msg->id     = (u32)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
	msg->id     = (u32)0x0003FFFF & (CAN1->sFIFOMailBox[0].RIR >> 3); // extended
	msg->len = (unsigned char)0x0000000F & CAN1->sFIFOMailBox[0].RDTR;
	msg->data[0] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR);
  msg->data[1] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 8);
  msg->data[2] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 16);
  msg->data[3] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDLR >> 24);

  msg->data[4] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR);
  msg->data[5] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 8);
  msg->data[6] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 16);
  msg->data[7] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[0].RDHR >> 24);
	
	//msg2->id     = (u32)0x000007FF & (CAN1->sFIFOMailBox[0].RIR >> 21);
	msg2->id     = (u32)0x0003FFFF & (CAN1->sFIFOMailBox[0].RIR >> 3); // extended
	msg2->len = (unsigned char)0x0000000F & CAN1->sFIFOMailBox[1].RDTR;
	msg2->data[0] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDLR);
  msg2->data[1] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDLR >> 8);
  msg2->data[2] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDLR >> 16);
  msg2->data[3] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDLR >> 24);

  msg2->data[4] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDHR);
  msg2->data[5] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDHR >> 8);
  msg2->data[6] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDHR >> 16);
  msg2->data[7] = (unsigned int)0x000000FF & (CAN1->sFIFOMailBox[1].RDHR >> 24);
	
	CAN1->RF0R |= CAN_RF0R_RFOM0;
}

void CAN_waitReady (void)  {
  while ((CAN1->TSR & CAN_TSR_TME0) == 0);         // Transmit mailbox 0 is empty
  CAN_TxRdy = 0;
}

void timerSetup(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;

	GPIOE->CRH |= GPIO_CRH_MODE15; /* 0b11, output, 50 MHz */
	GPIOE->CRH &= ~GPIO_CRH_CNF15; /* 0b00, push-pull */

	// 5Hz timer config.
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable clock

	TIM2->CR1 &= ~TIM_CR1_CEN; // disable counter
	TIM2->CR1 |= TIM_CR1_DIR;  // make downcounter

	TIM2->ARR = 0xFFFF; // 2^16 - 16-bit counter reset value (count DOWN from here)

	// set prescaler (assuming 2x 36MHz timer clock)
	// 5 = (2*36e6/(PSC+1))/2^16
	// PSC = (2*36e6/5)/2^16 - 1 = 218;
	TIM2->PSC = 218;

	TIM2->CR1 |= TIM_CR1_CEN; // enable counter
}
