#include "motorctrl.h"
#include "ADC.h"

// Forward and backward definition, can be changed
#define FORWARD 1
#define BACKWARD 0

#define FAST_PWM 0x03
#define OC0A 0x80
#define OC0B 0x20
#define OC2A 0x80
#define OC2B 0x20

#define SHUTDOWN 0 //shutdown motor
#define ANGLE_MAX_PWM 200 //Angle motor
#define TILT_MAX_PWM 200 //Tilt MOTOR
#define PRESCALER 0x04 //PWM frequency divider
#define NUMOFSAMPLES 4 //ADC averaging sample count

#define MOTOR_HYSTERESIS 2.0  //IN Degrees
#define MOTOR_ACCELERATION 5000 //wait time in micro seconds between speed increase
#define MOTOR_ACC_STEP 1 //8bit PWM step per acceleration.

//SET limits for the API
#define MIN_ANGLE 100.0
#define ANGLE_RANGE 160.0
#define MIN_TILT 5.0
#define TILT_RANGE 85.0

#define ANGLE_REFERENCE 180.0 //heading by default South
#define TILT_REFERENCE 5.0   //Tilted 5 degrees upward frow vertical angle

#define ANGLE_MOTOR_TIMEOUT 15000 // in milliseconds
#define TILT_MOTOR_TIMEOUT 15000  // in milliseconds

//Angle correction factors, all values are millimeters
#define ANGLE_X 700L //Distance from center to actuator lower point
#define ANGLE_C 190L //Distance from center to actuator far end
#define ANGLE_Y 50L  //Offset distance from X to outside

#define TILT_X 540L //Distance from center to actuator lower point
#define TILT_C 170L //Distance from center to actuator far end
#define TILT_Y 50L  //Offset distance from X to outside

//Define Actuator physical measurements
#define ACTUATOR_A_MIN_LENGTH 515L //absolute value
#define ACTUATOR_A_MAX_LENGTH 890L //absolute value
#define ACTUATOR_A_MIN_LIMIT  520L //Minimum limit where actuator can go
#define ACTUATOR_A_MAX_LIMIT  880L //Maximum limit where actuator can go

#define ACTUATOR_B_MIN_LENGTH 340L  //375 515-890               //pidempi
#define ACTUATOR_B_MAX_LENGTH 540L  //200 340-540 pisimmällään  //lyhyempi
#define ACTUATOR_B_MIN_LIMIT  350L
#define ACTUATOR_B_MAX_LIMIT  530L

#define MOTOR_A 0
#define MOTOR_B 1
//Define Actuator ADC Channels for Motor_A and Motor_b

#define ACTUATOR_ADC_A 0
#define ACTUATOR_ADC_B 1
#define ACTUATOR_CURRENT_ADC_A 6 
#define ACTUATOR_CURRENT_ADC_B 7
//Defines which actuator is controlling tilt and which controls angular movements

#define ANGLE_MOTOR MOTOR_A
#define TILT_MOTOR MOTOR_B 

#define ANGLE_ACTUATOR_ADC ACTUATOR_ADC_A
#define TILT_ACTUATOR_ADC  ACTUATOR_ADC_B
#define ANGLE_ACTUATOR_CURRENT_ADC ACTUATOR_CURRENT_ADC_A
#define TILT_ACTUATOR_CURRENT_ADC ACTUATOR_CURRENT_ADC_B

#define ANGLE_ACTUATOR_MIN_LENGTH ACTUATOR_A_MIN_LENGTH
#define ANGLE_ACTUATOR_MAX_LENGTH ACTUATOR_A_MAX_LENGTH
#define ANGLE_ACTUATOR_MIN_LIMIT ACTUATOR_A_MIN_LIMIT
#define ANGLE_ACTUATOR_MAX_LIMIT ACTUATOR_A_MAX_LIMIT

#define TILT_ACTUATOR_MIN_LENGTH ACTUATOR_B_MIN_LENGTH
#define TILT_ACTUATOR_MAX_LENGTH ACTUATOR_B_MAX_LENGTH
#define TILT_ACTUATOR_MIN_LIMIT ACTUATOR_B_MIN_LIMIT
#define TILT_ACTUATOR_MAX_LIMIT ACTUATOR_B_MAX_LIMIT


//Initialize MOTOR A, Angle motor
//volatile motor motors[NUM_OF_MOTORS];
//extern motor motors[NUM_OF_MOTORS];

//returns motor final calculated position in degrees
float getMotorPosition(motor *m){
    uint16_t alen = getActuatorLength(m);
    float aoffset = m->angle_correction(alen);
    return m->angle_reference + aoffset;
    /*
        example reference angle motor = 180
        aoffset between -90 to 90
        -90 when actuator is minimum position
        90 when actuator is at max position

        ie. 180 + offset => 180 + -90 = 90  degrees
        ie. 180 +offset  => 180 + +90 = 270 degrees
    */
}

//returns motor actuator length in millimeters
uint16_t getActuatorLength(motor *m){
    uint16_t voltage = AVGVoltage(m->actuator_adc_channel, 0x40, NUMOFSAMPLES);
    float effective_range = (m->actuator_max_length - m->actuator_min_length); 
    uint16_t length =  m->actuator_min_length + (effective_range / 1024)*voltage; //volts per degree
    
    if (length >= m->actuator_max_limit){
        m->status = MAX_LIMIT;
        motorControl(m, FORWARD, SHUTDOWN); //SHUTDOWN motor
        //TODO do something when limits are crossed
    }
    else if(length <= m->actuator_min_limit){
        m->status = MIN_LIMIT;
        motorControl(m, FORWARD, SHUTDOWN);  //SHUTDOWN motor
    }
    return length;
}

uint16_t getTiltActuatorCurrentLength(void){
    return getActuatorLength(&motors[TILT_MOTOR]);
}

uint16_t getAngleActuatorCurrentLength(void){
    return getActuatorLength(&motors[ANGLE_MOTOR]);
}



//Returns angle between -90.0 - 90.0, input value is in millimeters.
float angleConversion(uint16_t f){
    return -(360L*atan((2*ANGLE_C*ANGLE_X-sqrt((-ANGLE_C*ANGLE_C + 2*ANGLE_C*f - pow(f,2.0) + ANGLE_X*ANGLE_X + ANGLE_Y*ANGLE_Y)*
            (ANGLE_C*ANGLE_C + 2*ANGLE_C*f + pow(f,2.0)- ANGLE_X*ANGLE_X - ANGLE_Y*ANGLE_Y)))/
            (ANGLE_C*ANGLE_C + 2*ANGLE_C*ANGLE_Y - pow(f,2.0) + ANGLE_X*ANGLE_X + ANGLE_Y*ANGLE_Y)))/M_PI;
}

//Returns tilt angle between 0 to 90 degrees positive. input values in millimeters 
float tiltConversion(uint16_t f){
    return 90-(360L*atan((2*TILT_C*TILT_X-sqrt((-TILT_C*TILT_C + 2*TILT_C*f - pow(f,2.0) + TILT_X*TILT_X + TILT_Y*TILT_Y)*
              (TILT_C*TILT_C + 2*TILT_C*f + pow(f,2.0)- TILT_X*TILT_X - TILT_Y*TILT_Y)))/
              (TILT_C*TILT_C + 2*TILT_C*TILT_Y - pow(f,2.0) + TILT_X*TILT_X + TILT_Y*TILT_Y)))/M_PI;
}


float getAngle(void){
    return motors[ANGLE_MOTOR].current_position;
}

float getTilt(void){
    return motors[TILT_MOTOR].current_position;
}

/*
    This function is used to set wanted Angle value
*/
uint8_t setAngle(float angle){
    return setMotorPosition(&motors[ANGLE_MOTOR], angle);
}

/*
    This function is used to set wanted TILT angle
*/
uint8_t setTilt(float tilt){
    return setMotorPosition(&motors[TILT_MOTOR], tilt);
}

uint8_t setMotorPosition(motor *m, float angle){
    //vefify that angle is in between valid range
    if (angle >= m->min_angle && angle <= (m->min_angle + m->angle_range)){
        m->set_position = angle;
        m->timeout_value = 0; //Clear timeout value on every angle change
        return 0;
    }
    else{
        return 1;
    }
}   

/*
    Returns current Angle Set value
*/
float getSetAngle(void){
    return motors[ANGLE_MOTOR].set_position;
}

/*
    Returns current Tilt Set value  
*/
float getSetTilt(void){
    return motors[TILT_MOTOR].set_position;
}

/*
    Returns Angle motor status  
*/
motor_status getAngleMotorStatus(void){
    return motors[ANGLE_MOTOR].status;
}

/*
    Returns Tilt motor status,   
*/
motor_status getTiltMotorStatus(void){
    return motors[TILT_MOTOR].status;
}

/*
    This function controls angle and tilt motors
    It reads actual Angle and Tilt values using ADC
    Then it Adjust motor PWM to correct direction and leaves it there.
    This function must be called n. times per second
*/
motor_status motorController(void){
    //Update current motor positions
    uint8_t status = 0;
    for (uint8_t i = 0; i < NUM_OF_MOTORS; i++){
        motors[i].current_position = getMotorPosition(&motors[i]); 
        motorControlLoop(&motors[i]);
        status += motors[i].status; //Collect status from all motors
    }
    return status;
}

//This is motorcontrol loop which is called n. times per second.
void motorControlLoop(motor *m){    
    //Check if we have been running too long
    if (m->timeout_value >= m->timeout_setting){
        m->status = TIMEOUT_ERROR;
        motorControl(m, BACKWARD, 0); //Shutdown motor if it has been running too long.
        return;
    }

    //Set position is higher than current
    if (m->set_position > m->current_position + m->angle_hysteresis && m->status != MAX_LIMIT){ //Check that we are not overriding motor
        motorControl(m, FORWARD, m->max_pwm);
        m->timeout_value ++; //update timeout variables
        m->status = RUNNING_FORWARD;
    }
    else if (m->set_position < m->current_position - m->angle_hysteresis && m->status != MIN_LIMIT){
        motorControl(m, BACKWARD, m->max_pwm);
        m->timeout_value ++;
        m->status = RUNNING_BACKWARD;
    }
    else{ //Motor is close enough wanted position, Shutdown motor
        motorControl(m, BACKWARD, 0);
        m->timeout_value = 0; //Clear timeout
        m->status = STATUS_OK;
    }
}

void delayLoop_us(uint16_t delay){
    for (uint16_t i = 0; i < delay/50; i++){
        _delay_us(47);
    }
}

void motorControl(motor *m, uint8_t dir, uint8_t pwm){
    // We can directly control motor if direction is correct or motor is stopped
    if (m->current_dir == dir || m->current_pwm == 0){
        if (m->current_pwm < pwm){ //Need to accelerate
            while(m->current_pwm != pwm){ //Loop until set is equal
                if ((m->current_pwm + m->acceleration_step) > m->max_pwm){
                    m->current_pwm = m->max_pwm;
                    pwm = m->max_pwm;
                }
                else{
                    m->current_pwm += m->acceleration_step;
                }
                setMotor(m, dir, m->current_pwm);
                delayLoop_us(m->acceleration_time);
            }
        }
        else{
            while(m->current_pwm != pwm){ //Loop until set is equal
                if (m->current_pwm - m->deacceleration_step <= 0){
                    m->current_pwm = 0;
                }
                else{
                    m->current_pwm += m->deacceleration_step;
                }
                setMotor(m, dir, m->current_pwm);
                delayLoop_us(m->deacceleration_time);
            }
        }
    }
    else{ //Motor is going so it need to stop before direction change
        setMotor(m, m->current_dir, 0);
        motorControl(m, dir, pwm); // Call recursively this function again when motor is stopped.
    }
}


void initMotor(void){
    motor m1 =
    {
         &PORTD, &DDRD, 6, &TCCR0A, (OC0B + FAST_PWM), &TCCR0B, 0x00, &OCR0A, //MOTOR A FORWARD
         &PORTD, &DDRD, 5, &TCCR0A, (OC0A + FAST_PWM), &TCCR0B, 0x00, &OCR0B, //MOTOR A REVERSE
         &PORTD, &DDRD, 7, //MOTOR Enable control
         ANGLE_ACTUATOR_ADC,
         ANGLE_ACTUATOR_CURRENT_ADC,
         &PORTC, &DDRC, 2,
         &PORTC, &DDRC, 3,
         0.0,       //Current position
         FORWARD,   //current dir
         0.0,       //current position
         MIN_ANGLE + (ANGLE_RANGE / 2), //Set position, half way
         MOTOR_ACC_STEP,     //Acceleration step
         MOTOR_ACC_STEP,     //Deacceleration step
         MOTOR_ACCELERATION, //Acceleration time
         MOTOR_ACCELERATION, //deacceleration time
         MOTOR_HYSTERESIS,   //Anglular hysteresis in degrees
         ANGLE_MAX_PWM,      //MAX pwm value for anglular movements
         MIN_ANGLE,          //Minimun allowed angle
         ANGLE_RANGE,        //0-100 mapping to angle values
         ANGLE_REFERENCE,    //Reference which against angle corrections are applied   
         ANGLE_ACTUATOR_MIN_LENGTH,
         ANGLE_ACTUATOR_MAX_LENGTH,
         ANGLE_ACTUATOR_MIN_LIMIT,
         ANGLE_ACTUATOR_MAX_LIMIT,
         ANGLE_MOTOR_TIMEOUT,//timeout in milliseconds
         0,                  //timeout current value starts at zero
         STATUS_OK,
         &angleConversion //angle correction function pointer
    };
    motor m2 =
    {
         &PORTB, &DDRB, 3, &TCCR2A, (OC2A + FAST_PWM), &TCCR2B, PRESCALER, &OCR2A, //MOTOR B FORWARD
         &PORTD, &DDRD, 3, &TCCR2A, (OC2B + FAST_PWM), &TCCR2B, PRESCALER, &OCR2B,//MOTOR B REVERSE
         &PORTB, &DDRB, 4, //MOTOR Enable control
         TILT_ACTUATOR_ADC,
         TILT_ACTUATOR_CURRENT_ADC,
         &PORTC, &DDRC, 4,
         &PORTC, &DDRC, 5,
         0.0,         //Current position
         FORWARD,   //current dir
         0.0, //current position
         MIN_TILT + (TILT_RANGE / 2), //Set position, half way
         MOTOR_ACC_STEP,     //Acceleration step
         MOTOR_ACC_STEP,     //Deacceleration step
         MOTOR_ACCELERATION, //Acceleration time
         MOTOR_ACCELERATION, //deacceleration time
         MOTOR_HYSTERESIS,   //Anglular hysteresis in degrees
         TILT_MAX_PWM,      //MAX pwm value for anglular movements
         MIN_TILT,          //Minimun allowed angle
         TILT_RANGE,        //0-100 mapping to angle values
         TILT_REFERENCE,
         TILT_ACTUATOR_MIN_LENGTH,
         TILT_ACTUATOR_MAX_LENGTH,
         TILT_ACTUATOR_MIN_LIMIT,
         TILT_ACTUATOR_MAX_LIMIT,
         TILT_MOTOR_TIMEOUT,//timeout in milliseconds
         0,                  //timeout current value starts at zero
         STATUS_OK,
         &tiltConversion //angle correction function pointer
    };
    motors[ANGLE_MOTOR] = m1;
    motors[TILT_MOTOR] = m2;


    for(uint8_t i = 0; i<NUM_OF_MOTORS; i++){
        motor m = motors[i];
        //Set direction and enable to output pins
        *m.fwd_dir_addr |= 1<<m.fwd_pin;
        *m.rev_dir_addr |= 1<<m.rev_pin;
        *m.enable_dir_addr |= 1<<m.enable_pin;
        
        //Set pullups correctly
        *m.fwd_port_addr &= ~(1<<m.fwd_pin);
        *m.rev_port_addr &= ~(1<<m.rev_pin);
        *m.enable_port_addr &= ~(1<<m.enable_pin);
        
        //Init forward PWM settings
        *m.fwd_TCCRA_addr = 0x00;//m.fwd_TCCRA_value;
        *m.fwd_TCCRB_addr = 0x00;//m.fwd_TCCRB_value;
        *m.fwd_OCR_addr = 0x00; //Init PWM to zero
        
        //Init reverse PWM settings
        *m.rev_TCCRA_addr = 0x00;//m.rev_TCCRA_value;
        *m.rev_TCCRB_addr = 0x00; //m.rev_TCCRB_value;
        *m.rev_OCR_addr = 0x00; //Init PWM to zero
        
        //init Actuator +5 and GND pins
        *m.actuator_1_port_addr |= 1<<m.actuator_1_pin;
        *m.actuator_1_dir_addr |= 1<<m.actuator_1_pin;
        *m.actuator_1_port_addr &= ~(1<<m.actuator_2_pin);
        *m.actuator_1_dir_addr &= ~(1<<m.actuator_2_pin);
        _delay_ms(10); //wait 10ms so ADC pins settle.

        m.current_position = getMotorPosition(&m);  
    }
       //Update motor positions
    //motors[ANGLE_MOTOR].current_position = getAngle();
    //motors[TILT_MOTOR].current_position  = getTilt();    
    
   // GTCCR = 0x00;       //Start Counter
}

void disableMotorPWM(motor *m){

    *m->enable_port_addr &= ~(1<<m->enable_pin); //Disable motor => clear enable port
    *m->fwd_TCCRA_addr = 0x00;//m.fwd_TCCRA_value;
    *m->fwd_TCCRB_addr = 0x00;//m.fwd_TCCRB_value;
    *m->fwd_OCR_addr = 0x00; //Init PWM to zero
    
    //Init reverse PWM settings
    *m->rev_TCCRA_addr = 0x00;//m.rev_TCCRA_value;
    *m->rev_TCCRB_addr = 0x00; //m.rev_TCCRB_value;
    *m->rev_OCR_addr = 0x00; //Init PWM to zero

} 
//volatile uint8_t *OCRC_ADDR[] = {&OCR1CL, &OCR4AL }
//volatile uint8_t *OCRB_ADDR[] = {&OCR1BL, &OCR4BL }

void setMotor(motor *m, uint8_t dir, uint8_t pwm){
    m->current_pwm = pwm;
    if (pwm > 0){
        if (dir == FORWARD){ //Forward
            *m->rev_OCR_addr = 0x00; //PWM pulse width
            *m->fwd_OCR_addr = pwm; 
            *m->rev_port_addr &= ~(1<<m->rev_pin);
            *m->rev_TCCRA_addr = 0x00; //Disable reverse
            *m->fwd_TCCRA_addr = m->fwd_TCCRA_value; //Activate PWM forward
            
            *m->rev_TCCRB_addr = 0x00; //Disable clock Rev
            *m->fwd_TCCRB_addr = m->fwd_TCCRB_value;
        }
        else{ //Backward
            *m->fwd_OCR_addr = 0x00;
            *m->rev_OCR_addr = pwm; 
            *m->fwd_port_addr &= ~(1<<m->fwd_pin);
            *m->fwd_TCCRA_addr = 0x00; //Disable FWD
            *m->rev_TCCRA_addr = m->rev_TCCRA_value;

            *m->fwd_TCCRB_addr = 0x00;
            *m->rev_TCCRB_addr = m->rev_TCCRB_value;
        }
        
        *m->enable_port_addr |= 1<<m->enable_pin; 
    }
    else{
        disableMotorPWM(m);
    }
}








