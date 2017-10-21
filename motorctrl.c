#include "motorctrl.h"
#include "ADC.h"


// Forward and backward definition, can be changed
#define FORWARD 1
#define BACKWARD 0

//Motor physical connections and PWM settings
#define MOTOR_A 0
#define MOTOR_A_FWD_PORT PORTD //OC0A PD6
#define MOTOR_A_FWD_DIR  DDRD 
#define MOTOR_A_FWD_PIN  6
#define MOTOR_A_FWD_COUNTER 0x80 + 0x03  //OC0A clear on compare match + Fast PWM

#define MOTOR_A_RWD_PORT PORTD // 0C0B PD5
#define MOTOR_A_RWD_DIR  DDRD 
#define MOTOR_A_RWD_PIN  5
#define MOTOR_A_RWD_COUNTER 0x20 + 0x03 //OC0B clear on compare match + Fast PWM

#define MOTOR_A_ENABLE_PORT PORTD //PD7 PCINT23/AIN1
#define MOTOR_A_ENABLE_DIR  DDRD
#define MOTOR_A_ENABLE_PIN  7


#define MOTOR_B 1
#define MOTOR_B_FWD_PORT PORTB //OC2A PB3 
#define MOTOR_B_FWD_DIR  DDRB 
#define MOTOR_B_FWD_PIN  3
#define MOTOR_B_FWD_COUNTER 0x80 + 0x03// OC4B Clear on compare match + Fast PWM

#define MOTOR_B_RWD_PORT PORTD //OC2B PD3
#define MOTOR_B_RWD_DIR  DDRD
#define MOTOR_B_RWD_PIN  3
#define MOTOR_B_RWD_COUNTER 0x20 + 0x03// OC2B Clear on compare match + Fast PWM

#define MOTOR_B_ENABLE_PORT PORTB //PB4 MISO PIN
#define MOTOR_B_ENABLE_DIR  DDRB
#define MOTOR_B_ENABLE_PIN  4



#define ANGLE_MAX_PWM 200 //Angle motor
#define TILT_MAX_PWM 200 //Tilt MOTOR

#define PRESCALER 0x04 //PWM frequency divider
#define NUMOFSAMPLES 8 //ADC averaging sample count

#define MOTOR_HYSTERESIS 2 //IN Degrees
#define MOTOR_ACCELERATION 5000 //wait time in micro seconds between speed increase
#define MOTOR_ACC_STEP 1 //8bit PWM step per acceleration.

#define MIN_ANGLE 90
#define ANGLE_RANGE 160
#define MIN_TILT 0
#define TILT_RANGE 80

//Define which motor is used for tilting and which for angular movements
#define ANGLE_MOTOR MOTOR_A
#define TILT_MOTOR  MOTOR_B

//Define Actuator ADC Channels for Motor_A and Motor_b
#define ACTUATOR_ADC_A 2
#define ACTUATOR_ADC_B 3

//Defines which actuator is controlling tilt and which controls angular movements
#define ANGLE_ACTUATOR ACTUATOR_ADC_A
#define TILT_ACTUATOR  ACTUATOR_ADC_B

uint8_t motor_adc_mapping[2] =            {0, 0}; //Map motor to correct ADC channel
uint8_t actuator_to_motor_mapping[8] =    {0, 0, 0, 0, 0, 0, 0, 0}; //Slots for 8 ADC positions
volatile uint8_t motor_current_pwm[2] =   {0, 0};
volatile uint8_t motor_current_dir[2] =   {0, 0};
volatile uint16_t motor_current_position[2] = {0, 0};
volatile uint16_t motor_set_position[2] = {0, 0}; //Initial position is set to midway.
uint16_t motor_acceleration_step[2] =     {MOTOR_ACC_STEP, MOTOR_ACC_STEP};
uint16_t motor_deacceleration_step[2] =   {MOTOR_ACC_STEP, MOTOR_ACC_STEP};
uint16_t motor_acceleration_time[2] =     {MOTOR_ACCELERATION, MOTOR_ACCELERATION};
uint16_t motor_deacceleration_time[2] =   {MOTOR_ACCELERATION, MOTOR_ACCELERATION};
uint8_t motor_angle_hysteresis[2] =       {MOTOR_HYSTERESIS, MOTOR_HYSTERESIS}; //motor hysteresis in degrees
uint8_t motor_max_pwm[2] =                {200, 200};
uint16_t motor_angle_min[2] =             {0, 0}; //miminum possible angle for this motor
uint16_t motor_angle_range[2] =           {0, 0}; //possible angle range from min to max/*  

/*
    Converts ADC voltage to actuator angle
    This function is responsible to converting nonlinear actuator movement to angle
*/
uint16_t getAngle(void){
    uint16_t voltage = AVGVoltage(ANGLE_ACTUATOR, 0x40, NUMOFSAMPLES);
                    // 25 + (180 - 25) = (155 / 1024) => 0.15136 * voltage = >
    uint16_t angle = motor_angle_min[actuator_to_motor_mapping[ANGLE_ACTUATOR]] + ((motor_angle_range[actuator_to_motor_mapping[ANGLE_ACTUATOR]] / 1024) * voltage); //volts per degree
    //motor_current_position[actuator] = angle; //actuator is ADC position => 2 or 3 so array position is also 2 or 3   
    return angle;
}

/*
    Converts ADC voltage to actuator tilt
    This function is responsible to converting nonlinear actuator movement to angle
*/
uint16_t getTilt(void){
    uint16_t voltage = AVGVoltage(TILT_ACTUATOR, 0x40, NUMOFSAMPLES);
                    // 25 + (180 - 25) = (155 / 1024) => 0.15136 * voltage = >
    uint16_t tilt = motor_angle_min[actuator_to_motor_mapping[TILT_ACTUATOR]] + ((motor_angle_range[actuator_to_motor_mapping[TILT_ACTUATOR]] / 1024) * voltage); //volts per degree
    //motor_current_position[actuator] = angle; //actuator is ADC position => 2 or 3 so array position is also 2 or 3   
    return tilt;
}


/*
    This function is used to set wanted Angle value
*/
void setAngle(uint16_t angle){
    //vefify that angle is in between valid range
    if (angle >= motor_angle_min[ANGLE_MOTOR] && angle <= (motor_angle_min[ANGLE_MOTOR] + motor_angle_range[ANGLE_MOTOR])){
        motor_set_position[ANGLE_MOTOR] = angle;
    }
}

/*
    This function is used to set wanted TILT angle
*/
void setTilt(uint16_t tilt){
    //vefify that angle is in between valid range
    if (tilt >= motor_angle_min[TILT_MOTOR] && tilt <= (motor_angle_min[TILT_MOTOR] + motor_angle_range[TILT_MOTOR])){
        motor_set_position[TILT_MOTOR] = tilt;
    }
}

/*
    This function controls angle and tilt motors
    It reads actual Angle and Tilt values using ADC
    Then it Adjust motor PWM to correct direction and leaves it there.
    This function must be called n. times per second
*/
void motorController(void){
    //Update current motor positions
    motor_current_position[ANGLE_MOTOR] = getAngle();
    motor_current_position[TILT_MOTOR] = getTilt();

    //Set new PWM values based on set and actual position
    motorControlLoop(ANGLE_MOTOR);
    motorControlLoop(TILT_MOTOR);
}

//This is motorcontrol loop which is called n. times per second.
void motorControlLoop(uint8_t motor){    
    //Update ACTUATOR position
    if (motor_set_position[motor] > motor_current_position[motor] + motor_angle_hysteresis[motor]){
        motorControl(motor, FORWARD, motor_max_pwm[motor]);
    }
    else if (motor_set_position[motor] < motor_current_position[motor] - motor_angle_hysteresis[motor]){
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
                if ((motor_current_pwm[motor] + motor_acceleration_step[motor]) > motor_max_pwm[motor]){
                    motor_current_pwm[motor] = motor_max_pwm[motor];
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
                if (motor_current_pwm[motor] - motor_deacceleration_step[motor] <= 0){
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
    //Init min/max values and map Correct motor to correct actuator. 
    motor_adc_mapping[ANGLE_MOTOR] = ANGLE_ACTUATOR;
    motor_adc_mapping[TILT_MOTOR]  = TILT_ACTUATOR;
    actuator_to_motor_mapping[ANGLE_ACTUATOR] = ANGLE_MOTOR;
    actuator_to_motor_mapping[TILT_ACTUATOR]  = TILT_MOTOR;
    motor_set_position[ANGLE_MOTOR] = (motor_angle_min[ANGLE_MOTOR]+motor_angle_range[ANGLE_MOTOR]/2);
    motor_set_position[TILT_MOTOR]  = (motor_angle_min[TILT_MOTOR]+motor_angle_range[TILT_MOTOR]/2);
    motor_max_pwm[ANGLE_MOTOR] = ANGLE_MAX_PWM;
    motor_max_pwm[TILT_MOTOR]  = TILT_MAX_PWM;
    motor_angle_min[ANGLE_MOTOR] = MIN_ANGLE;
    motor_angle_min[TILT_MOTOR] = MIN_TILT;
    motor_angle_range[ANGLE_MOTOR] = ANGLE_RANGE;
    motor_angle_range[TILT_MOTOR]  = TILT_RANGE;

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
    motor_current_position[ANGLE_MOTOR] = getAngle();
    motor_current_position[TILT_MOTOR] = getTilt();    
    
   // GTCCR = 0x00;       //Start Counter
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








