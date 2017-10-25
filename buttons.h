#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


void initButtons(void);
uint8_t readAutoManualState(void);
uint8_t readTiltButtonState(void);
uint8_t readTurnButtonState(void);

