#include <avr/io.h>	/* Device specific declarations */
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "USART.h"
#include "ADC.h"
#include "motorctrl.h"
#include "serialparser.h"

FILE port;

//Debug led control for seeing that main loop is running
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PINB
#define LED_PIN_NUM 5
#define DISABLE_LED LED_DDR &= ~(1<<LED_PIN_NUM); LED_PORT &= ~(1<<LED_PIN_NUM);
#define ENABLE_LED LED_DDR |= (1<<LED_PIN_NUM); LED_PORT |= (1<<LED_PIN_NUM);


//16bit timer interrupt
ISR(TIMER1_COMPA_vect){
   //motorController();
}

void initSystemTimer(void){
    
    TCCR1A = 0x00; // WGM11 = 0, WGM10 = 0   => CTC mode
    // clock frequency = 16MHz / 1024 = 15625Hz
    TCCR1B = 0x08 + 0x05; //WGM12 = 1, WGM 13 = 0        => CTC mode, Clock divider 1024 = 0x05
    OCR1AH = 0x00;
    OCR1AL = 0x64;  // interrupt 100 times per second 15625/156~about 100 
    
    //Set interrupt to TIMER1 COMPA
    TIMSK1 = 0x02; //OCIEA enabled
    
    GTCCR = 0x00;    
}
int main (int argc, char *argv[])
{
	USART_Init(&port, 115200);
	USART0_Flush();
    initADC();
    initMotor();
	initSerialParser(&port);
    
    initSystemTimer(); //Starts all timers which are used => GTCCR = 0x00;
    sei();

	while(1){
		DISABLE_LED

        motorController();
        _delay_ms(1);

		ENABLE_LED
		//_delay_ms(100);
	}
	
}
