#include "motorctrl.h"
#include "ADC.h"

// Forward and backward definition, can be changed
#define FAST_PWM 0x03
#define OC0A 0x80
#define OC0B 0x20
#define OC2A 0x80
#define OC2B 0x20

#define DEG2RAD  M_PI / 180.0

#define CALIBRATION_HYSTERESIS 2 //in millimeters

#define SHUTDOWN 0 //shutdown motor
#define ANGLE_MAX_PWM 0xEF //Angle motor
#define TILT_MAX_PWM 0xFF //Tilt MOTOR
#define PRESCALER 0x02 //PWM frequency divider
#define NUMOFSAMPLES 4 //ADC averaging sample count

#define SET_HYSTERESIS_MM 2 
#define TILT_HYSTERESIS_MM 3 
#define ANGLE_HYSTERESIS_MM 4 //in millimeters
#define MOTOR_ACCELERATION 500 //wait time in micro seconds between speed increase
#define MOTOR_ACC_STEP 1 //8bit PWM step per acceleration.

//SET limits for the API
#define MIN_ANGLE 105.0
#define ANGLE_RANGE 120.0
#define MIN_TILT 30.0
#define TILT_RANGE 40.0

#define ANGLE_REFERENCE 165.0 //heading by default South
#define TILT_REFERENCE 0.0   //Tilted 5 degrees upward frow vertical angle

#define ANGLE_MOTOR_TIMEOUT 60000L // TODO refactor timeout mechanism
#define TILT_MOTOR_TIMEOUT 40000L  //

//Angle correction factors, all values are millimeters
#define ANGLE_X 700L //Distance from center to actuator lower point
#define ANGLE_C 190L //Distance from center to actuator far end
#define ANGLE_Y 5L  //Offset distance from X to outside

#define TILT_X 520L //Distance from center to actuator lower point
#define TILT_C 170L //Distance from center to actuator far end
#define TILT_Y 0L  //Offset distance from X to outside

//Define Actuator physical measurements
#define ACTUATOR_A_MIN_LENGTH 515L //absolute value
#define ACTUATOR_A_MAX_LENGTH 890L //absolute value
#define ACTUATOR_A_MIN_LIMIT  540L//Minimum limit where actuator can go
#define ACTUATOR_A_MAX_LIMIT  860L //Maximum limit where actuator can go

#define ACTUATOR_B_MIN_LENGTH 340L  //375 515-890               //pidempi
#define ACTUATOR_B_MAX_LENGTH 540L  //200 340-540 pisimmällään  //lyhyempi
#define ACTUATOR_B_MIN_LIMIT  380L
#define ACTUATOR_B_MAX_LIMIT  485L

#define VREF 4.7 //Reference voltage which is used for actuators
#define ACTUATOR_A_LOW_OFFSET 0.93
#define ACTUATOR_A_HIGH_OFFSET 3.75
#define ACTUATOR_B_LOW_OFFSET 1.57
#define ACTUATOR_B_HIGH_OFFSET 3.11
#define ADC_BITS 1024 //16384L

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

#define TILT_ACTUATOR_LOW_OFFSET ACTUATOR_B_LOW_OFFSET
#define TILT_ACTUATOR_HIGH_OFFSET ACTUATOR_B_HIGH_OFFSET
#define ANGLE_ACTUATOR_LOW_OFFSET ACTUATOR_A_LOW_OFFSET
#define ANGLE_ACTUATOR_HIGH_OFFSET ACTUATOR_A_HIGH_OFFSET

#define ANGLE_ACTUATOR_MIN_LENGTH ACTUATOR_A_MIN_LENGTH
#define ANGLE_ACTUATOR_MAX_LENGTH ACTUATOR_A_MAX_LENGTH
#define ANGLE_ACTUATOR_MIN_LIMIT ACTUATOR_A_MIN_LIMIT
#define ANGLE_ACTUATOR_MAX_LIMIT ACTUATOR_A_MAX_LIMIT

#define TILT_ACTUATOR_MIN_LENGTH ACTUATOR_B_MIN_LENGTH
#define TILT_ACTUATOR_MAX_LENGTH ACTUATOR_B_MAX_LENGTH
#define TILT_ACTUATOR_MIN_LIMIT ACTUATOR_B_MIN_LIMIT
#define TILT_ACTUATOR_MAX_LIMIT ACTUATOR_B_MAX_LIMIT

volatile motor motors[] = {
    {
         &PORTB, &DDRB, 3, &TCCR2A, (OC2A + FAST_PWM), &TCCR2B, PRESCALER, &OCR2A, //MOTOR B FORWARD
         &PORTD, &DDRD, 3, &TCCR2A, (OC2B + FAST_PWM), &TCCR2B, PRESCALER, &OCR2B,//MOTOR B REVERSE
         &PORTB, &DDRB, 4, //MOTOR Enable control       
 //        &PORTD, &DDRD, 6, &TCCR0A, (OC0B + FAST_PWM), &TCCR0B, PRESCALER, &OCR0B, //MOTOR A FORWARD
 //        &PORTD, &DDRD, 5, &TCCR0A, (OC0A + FAST_PWM), &TCCR0B, PRESCALER, &OCR0A, //MOTOR A REVERSE
 //        &PORTD, &DDRD, 7, //MOTOR Enable control
         ANGLE_ACTUATOR_ADC,
         ANGLE_ACTUATOR_CURRENT_ADC,
         0,       //pwm
         FORWARD,   //current dir
         0,  //current length       
         0,  //current set length
         MOTOR_ACC_STEP,     //Acceleration step
         MOTOR_ACC_STEP,     //Deacceleration step
         MOTOR_ACCELERATION, //Acceleration time
         MOTOR_ACCELERATION, //deacceleration time
         ANGLE_HYSTERESIS_MM,
         ANGLE_MAX_PWM,      //MAX pwm value for anglular movements
         MIN_ANGLE,          //Minimun allowed angle
         ANGLE_RANGE,        //0-100 mapping to angle values
         ANGLE_REFERENCE,    //Reference which against angle corrections are applied   
         ANGLE_ACTUATOR_MIN_LENGTH,
         ANGLE_ACTUATOR_MAX_LENGTH,
         ANGLE_ACTUATOR_MAX_LENGTH - ANGLE_ACTUATOR_MIN_LENGTH,
         ANGLE_ACTUATOR_MIN_LIMIT,
         ANGLE_ACTUATOR_MAX_LIMIT,
         ANGLE_MOTOR_TIMEOUT,//timeout in milliseconds
         0,                  //timeout current value starts at zero
         STATUS_OK,
         &angleConversion, //angle correction function pointer
         (ADC_BITS*ANGLE_ACTUATOR_LOW_OFFSET)/VREF, //217,87 offset
         (ADC_BITS*ANGLE_ACTUATOR_HIGH_OFFSET)/VREF, //806,12
         ((ADC_BITS*ANGLE_ACTUATOR_HIGH_OFFSET)/VREF) - ((ADC_BITS*ANGLE_ACTUATOR_LOW_OFFSET)/VREF),
         &angleDegToLength 
    },
    {
         &PORTD, &DDRD, 6, &TCCR0A, (OC0B + FAST_PWM), &TCCR0B, PRESCALER, &OCR0B, //MOTOR A FORWARD
         &PORTD, &DDRD, 5, &TCCR0A, (OC0A + FAST_PWM), &TCCR0B, PRESCALER, &OCR0A, //MOTOR A REVERSE
         &PORTD, &DDRD, 7, //MOTOR Enable control
  //       &PORTB, &DDRB, 3, &TCCR2A, (OC2A + FAST_PWM), &TCCR2B, PRESCALER, &OCR2A, //MOTOR B FORWARD
  //       &PORTD, &DDRD, 3, &TCCR2A, (OC2B + FAST_PWM), &TCCR2B, PRESCALER, &OCR2B,//MOTOR B REVERSE
  //       &PORTB, &DDRB, 4, //MOTOR Enable control
         TILT_ACTUATOR_ADC,
         TILT_ACTUATOR_CURRENT_ADC,
         0,         //PWM
         FORWARD,   //current dir
         0,  //current length       
         0,  //current set lengthMOTOR_ACC_STEP,     //Acceleration step
         MOTOR_ACC_STEP,
         MOTOR_ACC_STEP,     //Deacceleration step
         MOTOR_ACCELERATION, //Acceleration time
         MOTOR_ACCELERATION, //deacceleration time
         TILT_HYSTERESIS_MM,
         TILT_MAX_PWM,      //MAX pwm value for anglular movements
         MIN_TILT,          //Minimun allowed angle
         TILT_RANGE,        //0-100 mapping to angle values
         TILT_REFERENCE,
         TILT_ACTUATOR_MIN_LENGTH,
         TILT_ACTUATOR_MAX_LENGTH,
         TILT_ACTUATOR_MAX_LENGTH - TILT_ACTUATOR_MIN_LENGTH,
         TILT_ACTUATOR_MIN_LIMIT,
         TILT_ACTUATOR_MAX_LIMIT,
         TILT_MOTOR_TIMEOUT,//timeout in milliseconds
         0,                  //timeout current value starts at zero
         STATUS_OK,
         &tiltConversion, //angle correction function pointer
         (ADC_BITS*TILT_ACTUATOR_LOW_OFFSET)/VREF, //348,6
         (ADC_BITS*TILT_ACTUATOR_HIGH_OFFSET)/VREF, //668,9
         ((ADC_BITS*TILT_ACTUATOR_HIGH_OFFSET)/VREF) - ((ADC_BITS*TILT_ACTUATOR_LOW_OFFSET)/VREF),
         &tiltDegToLength
    }};

volatile int8_t running_motor = -1;
FILE *port;

//returns motor final calculated position in degrees
float getMotorPosition(volatile motor *m){
    float aoffset = m->angle_correction(m->current_length);
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

float getMotorSetPosition(volatile motor *m){
    float aoffset = m->angle_correction(m->set_length);
    return m->angle_reference + aoffset;
}

float getTiltMotorMinAngle(void){
    return getMotorMinAngle(&motors[TILT_MOTOR]);
}

float getAngleMotorMinAngle(void){
     return getMotorMinAngle(&motors[ANGLE_MOTOR]);
}

float getTiltMotorMaxAngle(void){
     return getMotorMaxAngle(&motors[TILT_MOTOR]);
}

float getAngleMotorMaxAngle(void){
     return getMotorMaxAngle(&motors[ANGLE_MOTOR]);
}


float getMotorMinAngle(volatile motor *m){
    return m->min_angle;
}

float getMotorMaxAngle(volatile motor *m){
    return m->min_angle + m->angle_range;
}
 

//returns motor actuator length in millimeters
uint16_t getActuatorLength(volatile motor *m){
    uint32_t voltage = AVGVoltage(m->actuator_adc_channel, 0x40, NUMOFSAMPLES);
    //uint32_t voltage = GetOverSampledVoltage(m->actuator_adc_channel, 0x40);
    voltage = (voltage - m->voltage_low_offset); //Fix minimum position starting at zero
    uint16_t length =  m->actuator_min_length + (m->actuator_range / m->voltage_range * voltage); //volts per degree
    
    //Shutdown if we cross virtual limit
    if (length >= m->actuator_max_limit){
        m->status = MAX_LIMIT;
        motorControl(m, m->current_dir, SHUTDOWN); //SHUTDOWN motor
        running_motor = -1;
    }
    else if(length <= m->actuator_min_limit){
        m->status = MIN_LIMIT;
        motorControl(m, m->current_dir, SHUTDOWN);  //SHUTDOWN motor
        running_motor = -1;
    }
    return length;
}

uint16_t getTiltActuatorCurrentLength(void){
    return getActuatorLength(&motors[TILT_MOTOR]);
}

uint16_t getAngleActuatorCurrentLength(void){
    return getActuatorLength(&motors[ANGLE_MOTOR]);
}

uint16_t getTiltActuatorSetLength(void){
    return motors[TILT_MOTOR].set_length;
}

uint16_t getAngleActuatorSetLength(void){
    return motors[ANGLE_MOTOR].set_length;
}

//Returns angle between -90.0 - 90.0, input value is in millimeters.
float angleConversion(uint16_t f){
    return -(360L*atan((2*ANGLE_C*ANGLE_X-sqrt((-ANGLE_C*ANGLE_C + 2*ANGLE_C*f - pow(f,2.0) + ANGLE_X*ANGLE_X + ANGLE_Y*ANGLE_Y)*
            (ANGLE_C*ANGLE_C + 2*ANGLE_C*f + pow(f,2.0)- ANGLE_X*ANGLE_X - ANGLE_Y*ANGLE_Y)))/
            (ANGLE_C*ANGLE_C + 2*ANGLE_C*ANGLE_Y - pow(f,2.0) + ANGLE_X*ANGLE_X + ANGLE_Y*ANGLE_Y)))/M_PI;
}

//Returns tilt angle between 0 to 90 degrees positive. input values in millimeters 
float tiltConversion(uint16_t f){
    return 90.0-(360L*atan((2L*TILT_C*TILT_X-sqrt((-TILT_C*TILT_C + 2*TILT_C*f - pow(f,2.0) + TILT_X*TILT_X + TILT_Y*TILT_Y)*
              (TILT_C*TILT_C + 2L*TILT_C*f + pow(f,2.0)- TILT_X*TILT_X - TILT_Y*TILT_Y)))/
              (TILT_C*TILT_C + 2L*TILT_C*TILT_Y - pow(f,2.0) + TILT_X*TILT_X + TILT_Y*TILT_Y)))/M_PI;
}

uint16_t angleDegToLength(float angle){
    float alfa = (angle+180)*DEG2RAD;
    return sqrt(pow((cos(alfa)*ANGLE_C+ANGLE_Y),2.0)+pow((ANGLE_X-sin(alfa)*ANGLE_C),2.0));
}

uint16_t tiltDegToLength(float angle){
    float alfa = (angle+90)*DEG2RAD;
    return sqrt(pow((cos(alfa)*TILT_C+TILT_Y),2.0)+pow((TILT_X-sin(alfa)*TILT_C),2.0));
}


float getAngle(void){
    return getMotorPosition(&motors[ANGLE_MOTOR]);
}

float getTilt(void){
    return getMotorPosition(&motors[TILT_MOTOR]);
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

/*
    Private function which sets a new position to given motor.
    Motor min/max angles are checked and min,max values are used
    if new angle is lower or higher
*/
uint8_t setMotorPosition(volatile motor *m, float angle){
    m->status = WAITING;
    //vefify that angle is in between valid range
    if (angle >= m->min_angle && angle <= (m->min_angle + m->angle_range)){
        m->timeout_value = 0; //Clear timeout value on every angle change
        uint16_t len = m->angle_to_length(angle - m->angle_reference);
    
        //fix hysteresis offset
        if (len > m->set_length){ //Setting bigger angle
            m->set_length = len + (m->length_hysteresis);
        }
        else{ //Setting lower angle
            m->set_length = len - (m->length_hysteresis);
        }
        return 0;
    }

    //angle is bigger than allowed => use max angle
    if (angle > (m->min_angle + m->angle_range)){
        m->set_length = m->angle_to_length((m->min_angle+m->angle_range) - m->angle_reference); 
    }
    //angle is smaller than allowed => use minimum angle
    else if (angle < m->min_angle){
        m->set_length = m->angle_to_length(m->min_angle - m->angle_reference); 
    }
    m->timeout_value = 0;
    return 1;
}   

/*
    Returns current Angle Set value
*/
float getSetAngle(void){
    return getMotorSetPosition(&motors[ANGLE_MOTOR]);
}

/*
    Returns current Tilt Set value  
*/
float getSetTilt(void){
    return getMotorSetPosition(&motors[TILT_MOTOR]);
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
    Shutdown all motors, this is used to activate manual mode
*/
void shutdownMotors(void){
    for (uint8_t i = 0; i<NUM_OF_MOTORS; i++){
        disableMotorPWM(&motors[i]);
        motors[i].set_length = motors[i].current_length;
    }
}


uint8_t setTiltMotorLength(uint16_t length){
    return setMotorLength(&motors[TILT_MOTOR], length);
}

uint8_t setAngleMotorLength(uint16_t length){
    return setMotorLength(&motors[ANGLE_MOTOR], length);
}

uint8_t setMotorLength(volatile motor *m, uint16_t length){
    if (length >= m->actuator_min_limit && length <= m->actuator_max_limit){
        m->set_length = length;
        m->status = WAITING;
        return 0;
    }
    else{
        return 1;
    }
}


void setLengthLoop(void){
    //uint8_t status = 0;
    for (uint8_t i = 0; i < NUM_OF_MOTORS; i++){
        volatile motor *m = &motors[i];
        m->current_length = getActuatorLength(m);

        //Check if we have been running too long
        if ((running_motor == i || running_motor == -1) && (m->status != STATUS_OK)){ //IF this motor is running or none of them is running 
            if (m->timeout_value >= m->timeout_setting){
                motorControl(m, m->current_dir, SHUTDOWN); //Shutdown motor if it has been running too lon.
                m->status = TIMEOUT_ERROR;
                fprintf(port, "timeout\n");
                m->set_length = m->current_length; //REset movement
                running_motor = -1;
            } 
            else{
                if (m->current_length > m->set_length + m->length_hysteresis){
                    motorControl(m, BACKWARD, m->max_pwm);
                    m->timeout_value ++; //update timeout variables
                    m->status = RUNNING_BACKWARD;
                    running_motor = i;
                }
                else if (m->current_length < m->set_length - m->length_hysteresis){
                    motorControl(m, FORWARD, m->max_pwm);
                    m->timeout_value ++; //update timeout variables
                    m->status = RUNNING_FORWARD;
                    running_motor = i;
                }
                else{
                    for(; m->current_pwm > 0; m->current_pwm--){
                        setMotor(m, m->current_dir, m->current_pwm);
                        delayLoop_us(m->deacceleration_time);
                    }
                    //fprintf(port,"Tval:%ld\n",m->timeout_value);
                    m->timeout_value = 0; //Clear timeout
                    m->status = STATUS_OK;
                    disableMotorPWM(m);       
                    _delay_ms(10);       
                    running_motor = -1;    
                }
            }
        }
       // status += m->status; //Collect status from all motors
    }
    //return status;
}

void forceMotors(uint8_t dir, uint8_t time){
    if (dir != FORWARD && dir != BACKWARD){
        fprintf(port,"ERR,dir\n");
        return;
    }
    fprintf(port, "Runnign motor %ds\n",time);
    for (uint8_t i=0; i< 1; i++){
        volatile motor *m = &motors[i];
        setMotor(m, dir, m->max_pwm);
    }
    for (uint16_t i=0;i<time*20;i++){
        for (uint8_t j=0;j<2;j++){
            volatile motor *m = &motors[j];
            uint16_t value = AVGVoltage(m->actuator_adc_channel, 0x40, NUMOFSAMPLES);
            fprintf(port,"j:%d,v:%d\n",j,value);
        }
        _delay_ms(50);
    }
    fprintf(port, "Shutdown\n");
    for (uint8_t i=0; i< 1; i++){
        volatile motor *m = &motors[i];
        setMotor(m, dir, 0);
    }
}

void calibrateMotors(void){
    //return;
    for(uint8_t i=0; i<NUM_OF_MOTORS;i++){
        fprintf(port, "calibrating motor:%d\n",i);
        volatile motor *m = &motors[i];
        uint16_t minval = 1024;
        uint16_t maxval = 0;
        setMotor(m, FORWARD, m->max_pwm);
        uint16_t lastval = 0; // AVGVoltage(m->actuator_adc_channel, 0x40, NUMOFSAMPLES);
        _delay_ms(500);
        for(uint8_t wait = 0; wait < 100; wait++){
            uint16_t value = AVGVoltage(m->actuator_adc_channel, 0x40, NUMOFSAMPLES);
            if (value > lastval - CALIBRATION_HYSTERESIS && value < lastval + CALIBRATION_HYSTERESIS){
                fprintf(port, "FW min: %d\n",value);
                minval = value;
                break;
            }
            lastval = value;
            _delay_ms(500);
        }
        setMotor(m, BACKWARD, m->max_pwm);
        lastval = 0; //AVGVoltage(m->actuator_adc_channel, 0x40, NUMOFSAMPLES);
        _delay_ms(500);
        for(uint8_t wait = 0; wait < 100; wait++){
            uint16_t value = AVGVoltage(m->actuator_adc_channel, 0x40, NUMOFSAMPLES);
            if (value > lastval - CALIBRATION_HYSTERESIS && value < lastval + CALIBRATION_HYSTERESIS){
                fprintf(port, "RW max: %d\n", value);
                maxval = value;
                break;
            }
            lastval = value;
            _delay_ms(500);
        }
        fprintf(port, "calibration ready\n");
        float min = minval;
        float max = maxval;
        fprintf(port, "Voltage Min:%f Max:%f\n",(4700.0/1024.0)*min,(4700.0/1024.0)*max);
    }
}

void delayLoop_us(uint16_t delay){
    for (uint16_t i = 0; i < delay/50; i++){
        _delay_us(47);
    }
}

void motorControl(volatile motor *m, uint8_t dir, uint8_t pwm){
    //verify min/max
    if (pwm > m->max_pwm){
        pwm = m->max_pwm;
    }
    
    //Set motor control
    if (m->current_pwm < pwm){ //Need to accelerate
        for(; m->current_pwm < pwm; m->current_pwm++){
            setMotor(m, dir, m->current_pwm);
            //getActuatorLength(m);
            delayLoop_us(m->acceleration_time);
        }
        setMotor(m, dir, m->current_pwm);
    }
    else if (m->current_pwm > pwm){
        for(; m->current_pwm > pwm; m->current_pwm--){
            setMotor(m, dir, m->current_pwm);
            //getActuatorLength(m);
            delayLoop_us(m->deacceleration_time);
        } 
        setMotor(m, dir, m->current_pwm);
    }
    m->current_dir = dir;
}


void initMotor(FILE *debugport){
    port = debugport;

    for(uint8_t i = 0; i<NUM_OF_MOTORS; i++){
        volatile motor *m = &motors[i];
        //Set direction and enable to output pins
        *m->fwd_dir_addr |= 1<<m->fwd_pin;
        *m->rev_dir_addr |= 1<<m->rev_pin;
        *m->enable_dir_addr |= 1<<m->enable_pin;
        
        //Set pullups correctly
        *m->fwd_port_addr &= ~(1<<m->fwd_pin);
        *m->rev_port_addr &= ~(1<<m->rev_pin);
        *m->enable_port_addr &= ~(1<<m->enable_pin);
        
        //Init forward PWM settings
        *m->fwd_TCCRA_addr = 0x00;//m.fwd_TCCRA_value;
        *m->fwd_TCCRB_addr = 0x00;//m.fwd_TCCRB_value;
        *m->fwd_OCR_addr = 0x00; //Init PWM to zero
        
        //Init reverse PWM settings
        *m->rev_TCCRA_addr = 0x00;//m.rev_TCCRA_value;
        *m->rev_TCCRB_addr = 0x00; //m.rev_TCCRB_value;
        *m->rev_OCR_addr = 0x00; //Init PWM to zero
        
        //init Actuator +5 and GND pins
        _delay_ms(10); //wait 10ms so ADC pins settle.

        m->current_length = getActuatorLength(m);
        m->set_length = m->current_length;
   } 
   // GTCCR = 0x00;       //Start Counter
}
void disableMotorPWM(volatile motor *m){

    *m->enable_port_addr &= ~(1<<m->enable_pin); //Disable motor => clear enable port
    *m->fwd_TCCRA_addr = 0x00;//m.fwd_TCCRA_value;
    *m->fwd_TCCRB_addr = 0x00;//m.fwd_TCCRB_value;
    *m->fwd_OCR_addr = 0x00; //Init PWM to zero
    
    //Init reverse PWM settings
    *m->rev_TCCRA_addr = 0x00;//m.rev_TCCRA_value;
    *m->rev_TCCRB_addr = 0x00; //m.rev_TCCRB_value;
    *m->rev_OCR_addr = 0x00; //Init PWM to zero
    m->current_pwm = 0; //set PWM to zero
    m->timeout_value = 0; //Reset timeout values
    
} 
//volatile uint8_t *OCRC_ADDR[] = {&OCR1CL, &OCR4AL }
//volatile uint8_t *OCRB_ADDR[] = {&OCR1BL, &OCR4BL }

void setMotor(volatile motor *m, uint8_t dir, uint8_t pwm){
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








