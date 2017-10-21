#include <avr/io.h>	/* Device specific declarations */
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "USART.h"
#include "ADC.h"
#include "motorctrl.h"

FILE port;
uint16_t set_angle = 0;
uint16_t set_tilt = 0;
uint8_t rxcount = 0;

#define SW_VERSION 1
#define BUFSIZE 256

#define FIND_SUCCESS 1
#define ERROR_CANNOT_FIND 2
#define ERROR_MAX_LENGTH 3


volatile uint8_t ring_write = 0;
volatile uint8_t ring_read = 0;
volatile char buffer[BUFSIZE+1];

void parseCommands(FILE *targetport);
uint16_t readInt16(uint8_t start, uint8_t stop);
uint8_t findParameter(char startchar, char stopchar, char secondstopchar, uint8_t maxlength, uint8_t *start, uint8_t *stop);
void parseCommands(FILE *targetport);

__attribute__((always_inline)) inline static uint8_t wlanbufferstate(uint8_t wr, uint8_t ww){
	if (wr == ww){
		return 0;
	}
	if (wr < ww){
		return ww-wr;
	}
	else{
		return ((BUFSIZE)-wr)+ww;
	}
}

__attribute__((always_inline)) inline static void clearBuffer( char *buffer,uint8_t len){
	for(uint8_t i=0;i<len;i++){	*buffer++ = '\0';}
}



ISR(USART_RX_vect) //Debug port
{
	volatile char temp = USART_Receive0();
	buffer[ring_write++] = temp;
	if (temp == '\n' || temp == '\r'){
        parseCommands(&port);
	}
}

uint16_t readInt16(uint8_t start, uint8_t stop){
    char buf[5];
    clearBuffer(buf,5);
    for (uint8_t i=0; i<5; i++){
        buf[i] = buffer[start++];
        if (start == stop){
           i = 5;
        }
    }
    return atoi(buf);
}

//Start and Stop values are pointing to Global ring buffer
uint8_t findParameter(char startchar, char stopchar, char secondstopchar, uint8_t maxlength, uint8_t *start, uint8_t *stop){
    //Find correct start character
    while(buffer[ring_read] != startchar && ring_read != ring_write){ 
        //fprintf(&port,"b:%c\n",buffer[ring_read]);
        ring_read++;
        maxlength --;
        if (maxlength == 0){
            return ERROR_MAX_LENGTH;
        }
    }
    if (ring_read == ring_write){
        return ERROR_CANNOT_FIND;
    }

    ring_read++; ///Next character is what we are looking for

    //Find stop character
    *start = ring_read;
    while(buffer[ring_read] != stopchar && buffer[ring_read] != secondstopchar){ //Find next delimiter or end of line
        ring_read++; //Step forward
    }
    *stop = ring_read;
    
    return FIND_SUCCESS;
}

void parseCommands(FILE *targetport){
    while (ring_read != ring_write){
        uint8_t command = buffer[ring_read++];
        if (command == 'P'){ //PUT new values command: P1:20:40    last  angle:tilt
            uint8_t start = 0;
            uint8_t stop = 0;
                
            if (buffer[ring_read] == '1'){    
                uint8_t succ = findParameter(':', ':', '\n', 20, &start, &stop);
                if (succ == FIND_SUCCESS){
                    set_angle = readInt16(start, stop);
                    succ = findParameter(':', ':', '\n', 20, &start, &stop);

                    set_tilt = readInt16(start, stop);
                    fprintf(targetport,"OK\n");
                }
                else{
                    fprintf(targetport,"ERR\n");
                }
            }
            else{
                fprintf(targetport,"ERR\n");
            }
        }
        else if (command == 'G'){ //Get values
            uint8_t value = buffer[ring_read++]; 
            if (value == '1'){ //Rread generic info
                fprintf(targetport,"G1:%d\n", SW_VERSION);
            }
            else if (value == '2'){ //Read current angle and tilt
                fprintf(targetport,"G2:%d:%d\n", set_angle, set_tilt);
            }
            else{
                fprintf(targetport,"ERR\n");
            }
            while(buffer[ring_read] != '\n' &&  buffer[ring_read] != '\r'  && ring_read != ring_write){
                ring_read++;
            }
        }
        else if (command == 'A'){ //ADC read voltage, commands A0\n  A1\n ,A2\n ,A3\n ,A4\n and so on are possible
            //ring_read++;
            uint8_t channel = 0;
            char buff[2];
            clearBuffer(buff, 2);
            buff[0] = buffer[ring_read];
            channel = atoi(buff);
            if (channel < 8){
                fprintf(targetport, "A%d:%d\n", channel, GetVoltage(channel, 0x40));
            }
            else if (channel == 8){
                fprintf(targetport,"A");
                for (uint8_t i=0; i<8; i++){
                    fprintf(targetport, "%d:%d:", i, GetVoltage(i, 0x40));
                }
                fprintf(targetport,"\n");
            }
            else{
                fprintf(targetport, "ERR\n");
            }
        }
        else if (command == 'R'){ //RAW ADC values

        }
    }
}

//16bit timer interrupt
ISR(TIMER1_COMPA_vect){
   motorController();
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
	sei();

	while(1){
		DDRB = 0xFF;
		PORTB = 0x00;
		_delay_ms(100);

		DDRB = 0xFF;
		PORTB = 0xFF;
		_delay_ms(100);
	}
	
}
