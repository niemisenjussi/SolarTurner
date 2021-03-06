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

void initLED(void){
    LED_DDR |= (1<<LED_PIN_NUM); 
}
int main (int argc, char *argv[])
{
    _delay_ms(500);
	USART_Init(&port, 115200);
	USART0_Flush();
    initLED();
    initADC();
    initMotor(&port);
	initSerialParser(&port);
    initButtons();    

    //for (float i=0; i<90; i+=1){
    //    uint16_t len = 
    //    fprintf(&port, "Len:%d, angle:%f\n",tiltDegToLength(i),i);
    //}
    //while(1){}
    
    GTCCR = 0x00;
    sei();

    uint8_t current_mode = readAutoManualState();
    fprintf(&port, "G6:%d:%d:%d\n",readAutoManualState(), readTiltButtonState(), readTurnButtonState());
    while(1){
        setLengthLoop();
        
        if (readAutoManualState() == MANUAL){ 
            switch(readTiltButtonState()){
                case 1:{
                    fprintf(&port, "G6:%d:%d:%d\n",readAutoManualState(), readTiltButtonState(), readTurnButtonState());
                    setAngleMotorLength(getAngleActuatorSetLength()+5);
                    while(readTiltButtonState() != 0){setLengthLoop();} //Wait until button is released
                    break;
                }
                case 2:{
                    fprintf(&port, "G6:%d:%d:%d\n",readAutoManualState(), readTiltButtonState(), readTurnButtonState());
                    setAngleMotorLength(getAngleActuatorSetLength()-5);
                    while(readTiltButtonState() != 0){setLengthLoop();} //Wait until button is released
                    break;
                }
            }
            switch(readTurnButtonState()){
                case 1:{
                    fprintf(&port, "G6:%d:%d:%d\n",readAutoManualState(), readTiltButtonState(), readTurnButtonState());
                    setTiltMotorLength(getTiltActuatorSetLength()+5);
                    while(readTurnButtonState() != 0){setLengthLoop();} //Wait until button is released
                    break;
                }
                case 2:{
                    fprintf(&port, "G6:%d:%d:%d\n",readAutoManualState(), readTiltButtonState(), readTurnButtonState());
                    setTiltMotorLength(getTiltActuatorSetLength()-5);
                    while(readTurnButtonState() != 0){setLengthLoop();} //Wait until button is released
                    break;
                }
            }
        }
        else{
             TOGGLE_LED
        }

        if (current_mode != readAutoManualState()){
            fprintf(&port, "G6:%d:%d:%d\n",readAutoManualState(), readTiltButtonState(), readTurnButtonState());
            if (readAutoManualState() == 1){
               // fprintf(&port, "Automatic mode turned ON\n");
                current_mode = 1;
            }
            else{
               // fprintf(&port, "Manual mode Activated\n");
                current_mode = 0;
                shutdownMotors();
            }
        }
	}
}
