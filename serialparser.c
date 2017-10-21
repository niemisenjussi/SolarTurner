#include "serialparser.h"
#include "ADC.h"
#include "USART.h"
#include "motorctrl.h"
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>

#define SW_VERSION 1
#define BUFSIZE 0xFF

#define FIND_SUCCESS 1
#define ERROR_CANNOT_FIND 2
#define ERROR_MAX_LENGTH 3

volatile uint8_t ring_write = 0;
volatile uint8_t ring_read = 0;
volatile char buffer[BUFSIZE+1];

FILE *port; //Serialport to host machine

__attribute__((always_inline)) inline static void clearBuffer(volatile char *buffer,uint8_t len){
	for(uint8_t i=0;i<len;i++){	*buffer++ = '\0';}
}

ISR(USART_RX_vect) //Serial port to Host machine
{
	volatile char temp = USART_Receive0();
	buffer[ring_write++] = temp;
	if (temp == '\n' || temp == '\r'){
        parseCommands();
	}
}

/*
    Reads ring buffer until it finds linefeed, carrier return or ring buffer is empty.
*/
void read_until_line_end(void){
    while(buffer[ring_read] != '\n' &&  buffer[ring_read] != '\r'  && ring_read != ring_write){
        ring_read++;
    }
}

/*
    Initializes serialport and clears ring buffer
    This function also takes angle and tilt global variable pointer where set angle values are stored 
*/
void initSerialParser(FILE *serialport){
    port = serialport;
    clearBuffer(buffer, BUFSIZE);
}

/*
    Reads 16 bit int from ringbuffer. 
    Function must be used because serial data is stored in to ring buffer
*/
uint16_t readInt16(uint8_t start, uint8_t stop){
    char buf[5] = {'\0','\0','\0','\0','\0'};
    
    //clearBuffer(buf,5);
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

void parseCommands(void){
    while (ring_read != ring_write){
        uint8_t command = buffer[ring_read++];
        if (command == 'P'){ //PUT new values command: P1:20:40    last  angle:tilt
            uint8_t start = 0;
            uint8_t stop = 0;
                
            if (buffer[ring_read] == '1'){    
                uint8_t succ = findParameter(':', ':', '\n', 20, &start, &stop);
                if (succ == FIND_SUCCESS){
                    setAngle(readInt16(start, stop));
                    succ = findParameter(':', ':', '\n', 20, &start, &stop);
                    if (succ == FIND_SUCCESS){
                        setTilt(readInt16(start, stop));
                        fprintf(port,"OK\n");
                    }
                    else{
                        fprintf(port, "ERR\n");
                    }
                }
                else{
                    fprintf(port,"ERR\n");
                }
            }
            else{
                fprintf(port,"ERR\n");
            }
            read_until_line_end();
        }
        else if (command == 'G'){ //Get values
            uint8_t value = buffer[ring_read++]; 
            if (value == '1'){ //Rread generic info
                fprintf(port,"G1:%d\n", SW_VERSION);
            }
            else if (value == '2'){ //Read current angle and tilt set_values
                fprintf(port,"G2:%d:%d\n", getSetAngle(), getSetTilt());
            }
            else if (value == '3'){ //Reads current values from motorctrl -module, actual values
                fprintf(port,"G3:%d:%d\n", getAngle(), getTilt());
            }
            else{
                fprintf(port,"ERR\n");
            }
            read_until_line_end();
        }
        else if (command == 'A'){ //ADC read voltage, commands A0\n  A1\n ,A2\n ,A3\n ,A4\n and so on are possible
            //ring_read++;
            uint8_t channel = 0;
            char buff[2];
            clearBuffer(buff, 2);
            buff[0] = buffer[ring_read];
            channel = atoi(buff);
            if (channel < 8){
                fprintf(port, "A%d:%d\n", channel, GetVoltage(channel, 0x40));
            }
            else if (channel == 8){
                fprintf(port,"A");
                for (uint8_t i=0; i<8; i++){
                    fprintf(port, "%d:%d:", i, GetVoltage(i, 0x40));
                }
                fprintf(port,"\n");
            }
            else{
                fprintf(port, "ERR\n");
            }
            read_until_line_end();
        }
        else if (command == 'R'){ //RAW ADC values
             
            
            
             read_until_line_end(); 
        }
    }
}
