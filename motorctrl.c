#include "motorctrl.h"
#include "ADC.h"



#define FORWARD 1
#define BACKWARD 0

#define MOTOR_A 0
#define MOTOR_A_FWD_PORT PORTD //OC0A PD6
#define MOTOR_A_FWD_DIR  DDRD 
#define MOTOR_A_FWD_PIN  6
#define MOTOR_A_FWD_COUNTER 0x80 + 0x03 

#define MOTOR_A_RWD_PORT PORTD // 0C0B PD5
#define MOTOR_A_RWD_DIR  DDRD 
#define MOTOR_A_RWD_PIN  5
#define MOTOR_A_RWD_COUNTER 0x20 + 0x03// 

#define MOTOR_A_ENABLE_PORT PORTD //PD7 PCINT23/AIN1
#define MOTOR_A_ENABLE_DIR  DDRD
#define MOTOR_A_ENABLE_PIN  7


#define MOTOR_B 1
#define MOTOR_B_FWD_PORT PORTB //OC2A PB3
#define MOTOR_B_FWD_DIR  DDRB 
#define MOTOR_B_FWD_PIN  3
#define MOTOR_B_FWD_COUNTER 0x80 + 0x03// OC4B

#define MOTOR_B_RWD_PORT PORTD //OC2B PD3
#define MOTOR_B_RWD_DIR  DDRD
#define MOTOR_B_RWD_PIN  3
#define MOTOR_B_RWD_COUNTER 0x20 + 0x03// OC2B

#define MOTOR_B_ENABLE_PORT PORTB //PB4 MISO PIN
#define MOTOR_B_ENABLE_DIR  DDRB
#define MOTOR_B_ENABLE_PIN  4

#define ACTUATOR_ADC_A 2
#define ACTUATOR_ADC_B 3

#define MOTOR_MAX_PWM_A 200 //Angle motor
#define MOTOR_MAX_PWM_B 200 //Tilt MOTOR

#define PRESCALER 0x04 //PWM frequency divider
#define NUMOFSAMPLES 8 //ADC averaging sample count

#define MOTOR_HYSTERESIS 2 //IN Degrees

uint8_t motor_acd_mapping[2] = {ACTUATOR_ADC_A, ACTUATOR_ADC_B}; //Map motor to correct ADC channel
volatile uint8_t motor_current_pwm[2] =   {0, 0};
volatile uint8_t motor_current_dir[2] =   {0, 0};
volatile uint16_t motor_current_position[2] = {0, 0};
uint16_t motor_acceleration_step[2] =      {1, 1};
uint16_t motor_deacceleration_step[2] =    {1, 1};
uint16_t motor_acceleration_time[2] =      {5000, 5000};
uint16_t motor_deacceleration_time[2] =    {5000, 5000};
uint8_t motor_angle_hysteresis[2] =        {2, 2}; //motor hysteresis in degrees
uint8_t motor_max_pwm[2] = {MOTOR_MAX_PWM_A, MOTOR_MAX_PWM_B};

volatile uint16_t ANGLE_MIN = 90;    //minimum possible angle, in degrees
volatile uint16_t ANGLE_RANGE = 160; //possible angles, in degrees

    
/*  
    Converts ADC voltage to actuator angle
*/
uint16_t getAngle(uint8_t actuator){
    uint16_t voltage = AVGVoltage(actuator, 0x40, NUMOFSAMPLES);
                    // 25 + (180 - 25) = (155 / 1024) => 0.15136 * voltage = >
    uint16_t angle = ANGLE_MIN + ((ANGLE_RANGE / 1024) * voltage); //volts per degree
    //motor_current_position[actuator] = angle; //actuator is ADC position => 2 or 3 so array position is also 2 or 3   
    return angle;
}

void motorController(void){
    motorControlLoop(MOTOR_A);
    motorControlLoop(MOTOR_B);
}

//This is motorcontrol loop which is called n. times per second.
void motorControlLoop(uint8_t motor){    
    //Update ACTUATOR position
    motor_current_position[motor] = getAngle(motor_adc_mapping[motor]);
    
    if (motor_set_position[motor] > motor_current_position[motor] + motor_angle_hysteresis[motor]){
        motorControl(motor, FORWARD, motor_max_pwm[motor]);
    }
    else if (motor_a_set_position < motor_a_current_position - motor_angle_hysteresis[motor]){
        motorControl(motor, BACKWARD, motor_max_pwm[motor]);
    }
    else{
        motorControl(motor, BACKWARD, 0);
    }
}

void delayLoop_us(uint16_t delay){
    for (uint16_t i = 0; i < delay/50; i++){
        _delay_us(47);
    }
}

void motorControl(uint8_t motor, uint8_t dir, uint8_t pwm){
    // We can directly control motor if direction is correct or motor is stopped
    if (motor_current_dir[motor] == dir || motor_current_pwm[motor] == 0){
        if (motor_current_pwm[motor] < pwm){ //Need to accelerate
            while(motor_current_pwm[motor] != pwm){ //Loop until set is equal
                if ((motor_current_pwm[motor] + motor_acceleration_step[motor]) > 255){
                    motor_current_pwm[motor] = 255;
                }
                else{
                    motor_current_pwm[motor] += motor_acceleration_step[motor];
                }
                setMotor(motor, dir, pwm);
                delayLoop_us(motor_acceleration_time[motor]);
            }
        }
        else{
            while(motor_current_pwm[motor] != pwm){ //Loop until set is equal
                if (motor_current_pwm[motor] - motor_deacceleration_step[motor] < 0){
                    motor_current_pwm[motor] = 0;
                }
                else{
                    motor_current_pwm[motor] += motor_deacceleration_step[motor];
                }
                setMotor(motor, dir, pwm);
                delayLoop_us(motor_deacceleration_time[motor]);
            }
        }
    }
    else{ //Motor is going so it need to stop before direction change
        setMotor(motor, motor_current_dir[motor], 0);
        motorControl(motor, dir, pwm); // Call recursively this function again when motor is stopped.
    }
}


void initMotor(void){
    //Set direction and enable to output
    MOTOR_A_FWD_DIR |= 1<<MOTOR_A_FWD_PIN; 
    MOTOR_A_RWD_DIR |= 1<<MOTOR_A_RWD_PIN; 
    MOTOR_A_ENABLE_DIR |= 1<<MOTOR_A_ENABLE_PIN;
    
    MOTOR_B_FWD_DIR |= 1<<MOTOR_B_FWD_PIN; 
    MOTOR_B_RWD_DIR |= 1<<MOTOR_B_RWD_PIN;
    MOTOR_B_ENABLE_DIR |= 1<<MOTOR_B_ENABLE_PIN;
    
    //Init to low so nothing moves
    MOTOR_A_FWD_PORT &= ~(1<<MOTOR_A_FWD_PIN);
    MOTOR_A_RWD_PORT &= ~(1<<MOTOR_A_RWD_PIN);
    MOTOR_A_ENABLE_PORT &= ~(1<<MOTOR_A_ENABLE_PIN);
    
    MOTOR_B_FWD_PORT &= ~(1<<MOTOR_B_FWD_PIN);
    MOTOR_B_RWD_PORT &= ~(1<<MOTOR_B_RWD_PIN);
    MOTOR_B_ENABLE_PORT &= ~(1<<MOTOR_B_ENABLE_PIN);
    
    //MOTOR A using TIMER0
    TCCR0A = 0x03; //OC1C and OC1B,  Fast PWM 8bit Top=0xFF
    OCR0A = 0x0; //PWM pulse width
    OCR0B = 0x0; //PWM pulse width
    TCCR0B = 0x00; //0x01 = no scaling, 0x02 = /8, 0x03 = /64, 0x04 = /256, 0x05 = /1024
    TCNT0 = 0; //Clean counter
    
    //MOTOR B using TIMER2
    TCCR2A = 0x03; //OC4B and OC4A, Fast PWM 8bit
    OCR2A = 0x0; //PWM pulse width
    OCR2B = 0x0; //PWM pulse width
    TCCR2B = 0x00; //0x01 = no scaling, 0x02 = /8, 0x03 = /64, 0x04 = /256, 0x05 = /1024
    TCNT2 = 0; //Clean counter

    //Update motor positions
    motor_current_position[MOTOR_A] = getAngle(motor_adc_mapping[MOTOR_A]);
    motor_current_position[MOTOR_B] = getAngle(motor_adc_mapping[MOTOR_B]);    
    
    GTCCR = 0x00;       //Start Counter
}

void disableMotorPWM(uint8_t motor){
    if (motor == MOTOR_A){
        MOTOR_A_ENABLE_PORT &= ~(1<<MOTOR_A_ENABLE_PIN);
        TCCR0A = 0x03; //WGM10 //Fast PWM
        TCCR0B = 0x00; //disable clock completety
        TCNT0 = 0; //Clean counter
        OCR0A = 0; 
        OCR0B = 0;
    }
    else if (motor == MOTOR_B){
        MOTOR_B_ENABLE_PORT &= ~(1<<MOTOR_B_ENABLE_PIN);
        TCCR2A = 0x03; //WGM10 //Fast PWM
        TCCR2B = 0x00; //disable clock completety'
        TCNT2 = 0; //Clean counter
        OCR2A = 0; //Set PWM to 0
        OCR2B = 0; //Set PWM to 0
    }
}

//volatile uint8_t *OCRC_ADDR[] = {&OCR1CL, &OCR4AL }
//volatile uint8_t *OCRB_ADDR[] = {&OCR1BL, &OCR4BL }

void setMotor(uint8_t motor, uint8_t dir, uint8_t pwm){
    if (motor == MOTOR_A){
        OCR0A = pwm; //PWM pulse width
        OCR0B = pwm; //PWM pulse width
        motor_current_pwm[motor] = pwm;
        if (pwm > 0){
            if (dir == FORWARD){ //Forward
                MOTOR_A_RWD_PORT &= ~(1<<MOTOR_A_RWD_PIN); //Set Reverse to LOW
                TCCR0A = MOTOR_A_FWD_COUNTER; //Enable Forward drive only, disconnect Backward
            }
            else{ //Backward
                MOTOR_A_FWD_PORT &= ~(1<<MOTOR_A_FWD_PIN); //Set Forward to LOW
                TCCR0A = MOTOR_A_RWD_COUNTER; //Enable Reverse drive only, disconnect Forward
            }
            TCCR0B = PRESCALER; 
            MOTOR_A_ENABLE_PORT |= 1<<MOTOR_A_ENABLE_PIN; //Enable motor A in general
        }
        else{
            disableMotorPWM(motor);
        }
    }
    else if (motor == MOTOR_B){
        OCR2A = pwm; //PWM pulse width
        OCR2B = pwm; //PWM pulse width
        motor_current_pwm[motor] = pwm;
        if (pwm > 0){
            if (dir == FORWARD){ //Forward
                MOTOR_B_RWD_PORT &= ~(1<<MOTOR_B_RWD_PIN); //Set Reverse to LOW
                TCCR2A = MOTOR_B_FWD_COUNTER; //Enable Forward drive only, disconnect Backward
            }
            else{ //Backward
                MOTOR_B_FWD_PORT &= ~(1<<MOTOR_B_FWD_PIN); //Set Forward to LOW
                TCCR2A = MOTOR_B_RWD_COUNTER; //Enable Reverse drive only, disconnect Forward
            }
            TCCR2B = PRESCALER; 
            MOTOR_B_ENABLE_PORT |= 1<<MOTOR_B_ENABLE_PIN; //Enable motor B in general
        }
        else{
            disableMotorPWM(motor);
        }
    }
}








