#include <avr/io.h>
#include <util/delay.h>
#include "ADC.h"

#define ADC_PORT PORTC
#define ADC_DIR  DDRC

uint16_t GetVoltage(uint8_t Sensor, uint8_t ADCRange)
{
    int Voltage = 0;
    ADMUX = Sensor+ADCRange;
    ADCSRA = 0b11000111; //tämä toimi AC kanssa 128 jakaja
    
    do {} while (bit_is_set(ADCSRA,6));
    
    Voltage = ADCW;     
    
    if (Voltage < 0) {Voltage = 0;}
    
    return Voltage;
}

void initADC(void){
    ADC_DIR = 0x00;  //All input
    ADC_PORT = 0x00; //Pulldown
}


uint16_t AVGVoltage(uint8_t Sensor, uint8_t ADCRange, uint8_t num_of_samples){
    
    uint32_t res = 0;
    for (uint8_t i=0; i<num_of_samples; i++){
        res += GetVoltage(Sensor, ADCRange);
        _delay_us(10);
    }
    uint16_t final = res/num_of_samples;
    return final;
}


