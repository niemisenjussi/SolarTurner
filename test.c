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
//    TOGGLE_LED
}

void initSystemTimer(void){
    //TCCR1A = 0x00; // WGM11 = 0, WGM10 = 0   => CTC mode
    // clock frequency = 16MHz / 1024 = 15625Hz
    //TCCR1B = 0x08 + 0x05; //WGM12 = 1, WGM 13 = 0        => CTC mode, Clock divider 1024 = 0x05
    //OCR1AH = 0x05; //Zero this when tests are done
    //OCR1AL = 0x64;  // interrupt 100 times per second 15625/156~about 100 
    //Set interrupt to TIMER1 COMPA
    //TIMSK1 = 0x02; //OCIEA enabled
    GTCCR = 0x00;    
}
void initLED(void){
    LED_DDR |= (1<<LED_PIN_NUM); 
}
int main (int argc, char *argv[])
{
    _delay_ms(50);
	USART_Init(&port, 115200);
	USART0_Flush();
    initLED();
    initADC();
    initMotor(&port);
	initSerialParser(&port);
    initButtons();    

    initSystemTimer(); //Starts all timers which are used => GTCCR = 0x00;
    
//     for (uint16_t i = 340; i < 540; i+=10){
//         float angle = tiltConversion(i);
//         uint16_t l = tiltDegToLength(angle);
//         fprintf(&port, "tilt:%5.2f f:%d len:%d\n", angle, i, l);
//     }
// 
//     for (uint16_t i = 515; i < 890; i+=10){
//         float angle = angleConversion(i);
//         uint16_t l = angleDegToLength(angle);
//         fprintf(&port, "angle:%5.2f f:%d len:%d\n",angle, i, l);
//     }
//     while(1){}
// 
    GTCCR = 0x00;
    sei();

    uint8_t current_mode = readAutoManualState();
    while(1){
       // uint8_t status = motorController();
        setLengthLoop();

//        if (status > STATUS_OK){
                //sendError();
//        }

        if (readAutoManualState() == MANUAL){ 
            switch(readTiltButtonState()){
                case 1:{
                    setAngle(getSetAngle()+2);
                    fprintf(&port,"Turning 2 degrees left\n");
                    while(readTiltButtonState() != 0){motorController();} //Wait until button is released
                    break;
                }
                case 2:{
                    setAngle(getSetAngle()-2);
                    fprintf(&port,"Turning 2 degrees right\n");
                    while(readTiltButtonState() != 0){motorController();} //Wait until button is released
                    break;
                }
            }
            switch(readTurnButtonState()){
                case 1:{
                    setTilt(getSetTilt()+2);
                    fprintf(&port,"Turning 2 degree UP\n");
                    while(readTurnButtonState() != 0){motorController();} //Wait until button is released
                    break;
                }
                case 2:{
                    setTilt(getSetTilt()-2);
                    fprintf(&port,"Turning 2 degrees DOWN\n");
                    while(readTurnButtonState() != 0){motorController();} //Wait until button is released
                    break;
                }
            }
        }
        else{
             TOGGLE_LED
        }

        if (current_mode != readAutoManualState()){
            if (readAutoManualState() == 1){
                fprintf(&port, "Automatic mode turned ON\n");
                current_mode = 1;
            }
            else{
                fprintf(&port, "Manual mode Activated\n");
                current_mode = 0;
                shutdownMotors();
            }
        }
	}
}
