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
#include "buttons.h"

FILE port;
//motor motors[NUM_OF_MOTORS];

//Debug led control for seeing that main loop is running
#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PINB
#define LED_PIN_NUM 5
#define DISABLE_LED LED_PORT &= ~(1<<LED_PIN_NUM);
#define ENABLE_LED LED_PORT |= (1<<LED_PIN_NUM);
#define TOGGLE_LED LED_PORT ^= (1<<LED_PIN_NUM);

//16bit timer interrupt
ISR(TIMER1_COMPA_vect){
   // TOGGLE_LED
}

void initSystemTimer(void){
    TCCR1A = 0x00; // WGM11 = 0, WGM10 = 0   => CTC mode
    // clock frequency = 16MHz / 1024 = 15625Hz
    TCCR1B = 0x08 + 0x05; //WGM12 = 1, WGM 13 = 0        => CTC mode, Clock divider 1024 = 0x05
    OCR1AH = 0x05; //Zero this when tests are done
    OCR1AL = 0x64;  // interrupt 100 times per second 15625/156~about 100 
    //Set interrupt to TIMER1 COMPA
    TIMSK1 = 0x02; //OCIEA enabled
    GTCCR = 0x00;    
}
void initLED(void){
    LED_DDR |= (1<<LED_PIN_NUM); 
}
int main (int argc, char *argv[])
{
	USART_Init(&port, 115200);
	USART0_Flush();
    initLED();
    initADC();
    initMotor();
	initSerialParser(&port);
    initButtons();    

    initSystemTimer(); //Starts all timers which are used => GTCCR = 0x00;
    sei();
// 
//     float (*acor)(double angle);
//     float (*tcor)(double angle);
//     acor = &angleConversion;
//     tcor = &tiltConversion;
//     for(uint16_t i=400;i<600;i++){
//         double point = i;
//         point /= 10;
//         fprintf(&port, "angle d:%f a:%f\n",point,tcor(point));
//         //_delay_ms(100);
//     }
//     while(1){}
// 
//     motorController(); //Update positions
//     
//     fprintf(&port, "\n\n\n");
//     uint16_t tiltlen = getTiltActuatorCurrentLength();
//     fprintf(&port, "tilt_len:%d\n",tiltlen);
//     uint16_t anglelen = getAngleActuatorCurrentLength();
//     fprintf(&port, "angle_len:%d\n",anglelen);
// 
//     fprintf(&port, "tilt_angle:%5.2f\n",getTilt());
//     fprintf(&port, "angle_angle:%5.2f\n",getAngle());
//     
//     fprintf(&port, "tilt_angle:%5.2f\n",tiltConversion(tiltlen));
//     fprintf(&port, "angle_angle:%5.2f\n",angleConversion(anglelen));
//  

    uint8_t angle = 0;
    uint8_t tilt = 125;
    //motors[0].set_position = 51;
    //motors[1].set_position = 244;
 
    while(1){
	//	DISABLE_LED
        fprintf(&port,"auto:%d, tilt:%d, turn:%d\n",readAutoManualState(),readTiltButtonState(),readTurnButtonState());
     //   uint8_t status = motorController();
     //   if (status > STATUS_OK){
            //sendError();
     //   }
        setAngle(angle++);
        setTilt(tilt++);
        _delay_ms(100);
        TOGGLE_LED
    //    fprintf(&port,"angle:%d tilt:%d\n",getSetAngle(), getSetTilt());
      //  fprintf(&port,"d_angle:%d d_tilt:%d\n",motors[0].set_position, motors[1].set_position);

	//	ENABLE_LED
		//_delay_ms(100);
	}
	
}
