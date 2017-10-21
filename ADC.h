//#include <avr/io.h> 

uint16_t AVGVoltage(uint8_t Sensor, uint8_t ADCRange, uint8_t num_of_samples);
uint16_t GetVoltage(uint8_t Sensor, uint8_t ADCRange);
void initADC(void);
