#include <avr/io.h>	/* Device specific declarations */
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


void initMotor(void);
uint16_t getAngle(void);
uint16_t getTilt(void);
void setAngle(uint16_t angle);
void setTilt(uint16_t tilt);
uint16_t getSetAngle(void);
uint16_t getSetTilt(void);
void motorController(void);
void motorControlLoop(uint8_t motor);
void motorControl(uint8_t motor, uint8_t dir, uint8_t pwm);
void disableMotorPWM(uint8_t motor);
void setMotor(uint8_t motor, uint8_t dir, uint8_t pwm);
void delayLoop_us(uint16_t delay); 

 



