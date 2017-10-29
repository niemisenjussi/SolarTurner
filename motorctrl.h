#include <avr/io.h>	/* Device specific declarations */
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>

//#define STATUS_OK 0
//#define RUNNING_FORWARD 1
//#define RUNNING_BACKWARD 2
//#define TIMEOUT_ERROR 3

#define NUM_OF_MOTORS 2

typedef enum {
    STATUS_OK = 0,
    RUNNING_FORWARD = 1,
    RUNNING_BACKWARD = 2,
    TIMEOUT_ERROR = 3,
    MIN_LIMIT = 4,
    MAX_LIMIT = 5
} motor_status;

typedef struct{
    //Forward mode values
    volatile uint8_t *fwd_port_addr;
    volatile uint8_t *fwd_dir_addr;
    uint8_t fwd_pin;
    volatile uint8_t *fwd_TCCRA_addr;
    uint8_t fwd_TCCRA_value;
    volatile uint8_t *fwd_TCCRB_addr;
    uint8_t fwd_TCCRB_value;
    volatile uint8_t *fwd_OCR_addr; //PWM value

    //Reverse mode values
    volatile uint8_t *rev_port_addr;
    volatile uint8_t *rev_dir_addr;
    uint8_t rev_pin;
    volatile uint8_t *rev_TCCRA_addr;
    uint8_t rev_TCCRA_value;
    volatile uint8_t *rev_TCCRB_addr;
    uint8_t rev_TCCRB_value;
    volatile uint8_t *rev_OCR_addr;

    //Generic parameters for this motor
    volatile uint8_t *enable_port_addr;
    volatile uint8_t *enable_dir_addr;
    uint8_t enable_pin;
    
    //Actuator connections
    uint8_t actuator_adc_channel; //potentiometer ADC chanel
    uint8_t actuator_current_adc_channel; // Channel which measures current consumption
    volatile uint8_t *actuator_1_port_addr; //Pins where actuator potentiometer is connected
    volatile uint8_t *actuator_1_dir_addr;
    uint8_t actuator_1_pin;
    volatile uint8_t *actuator_2_dir_addr;
    volatile uint8_t *actuator_2_port_addr; //another end of potentiometer
    uint8_t actuator_2_pin;
    

    //PWM and movement parameters
    volatile uint8_t current_pwm;
    volatile uint8_t current_dir;
    volatile float current_position;
    volatile float set_position;
    uint8_t acceleration_step;
    uint8_t deacceleration_step;
    uint16_t acceleration_time;
    uint16_t deacceleration_time;
    float angle_hysteresis;
    uint8_t max_pwm;
    float min_angle;
    float angle_range;
    float angle_reference;
    uint16_t actuator_min_length;
    uint16_t actuator_max_length;
    float actuator_range;
    uint16_t actuator_min_limit;
    uint16_t actuator_max_limit;
    uint16_t timeout_setting;
    uint16_t timeout_value;
    volatile motor_status status;
    float (*angle_correction)(uint16_t f);
    uint16_t voltage_low_offset;
    uint16_t voltage_high_offset; 
    uint16_t voltage_range;
} motor;

//motor motors[NUM_OF_MOTORS];

//Public methods for accessing and setting values
void initMotor(FILE *debugport);
float getAngle(void);
float getTilt(void);
uint8_t setAngle(float angle);
uint8_t setTilt(float tilt);
float getSetAngle(void);
float getSetTilt(void);
uint16_t getTiltActuatorCurrentLength(void);
uint16_t getAngleActuatorCurrentLength(void);
motor_status getAngleMotorStatus(void);
motor_status getTiltMotorStatus(void);
motor_status motorController(void);
void shutdownMotors(void);

//private functions which are called only inside
float getMotorPosition(volatile motor *m); //returns motor final angle in degrees
uint16_t getActuatorLength(volatile motor *m); //Returns actuator length (body+actuated distance)
uint8_t setMotorPosition(volatile motor *m, float angle); //sets motor new angle
void motorControlLoop(volatile motor *m);
void motorControl(volatile motor *m, uint8_t dir, uint8_t pwm);
void disableMotorPWM(volatile motor *m);
void setMotor(volatile motor *m, uint8_t dir, uint8_t pwm);
void delayLoop_us(uint16_t delay); 
float angleConversion(uint16_t y);
float tiltConversion(uint16_t y);




