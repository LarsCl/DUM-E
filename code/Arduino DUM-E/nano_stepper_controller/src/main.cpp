#include <Arduino.h>
#include <Wire.h>
#include <PCF8574.h>
#include <stdlib.h>

// Define PCF8574 addresses
#define PCF_COUNT 4

/* From pcf lib, all high except DRV8825 mode pins */
#define PCF8574_INITIAL_VALUE 0x53

// PCF8574 Pin Assignments
#define NFLT_PIN 0
#define NRST_PIN 1
#define M2_PIN 2
#define M1_PIN 3
#define NEN_PIN 4
#define M0_PIN 5
#define NSLP_PIN 6
#define DIR_PIN 7

// Step pins for each motor
#define STEP0_PIN 2
#define STEP1_PIN 3
#define STEP2_PIN 4
#define STEP3_PIN 5

/* NANO PCBA nets*/
#define SERVO0 8
#define SERVO1 7
#define SERVO2 6
#define DIGITAL0 9
#define DIGITAL1 10

#define ANALOG0 0
#define ANALOG1 1
#define ANALOG2 2
#define ANALOG3 3
#define ANALOG6 6
#define ANALOG7 7

/* Frequency of stepper control loop in Hertz */
#define CL_FREQ 1000
#define CL_PERIOD_US (0xF4240 / CL_FREQ)

#define DEFAULT_MAX_ROT_VEL 500 /*60RPM*/
#define DEFAULT_ROT_ACCEL 15   /*15RPM*/
#define DEFAULT_DRV8825_MODE 0 /*fullstep*/

// Gear ratios and stepcount
static int SteppermotorGear = 18;
static int BigPullyGear = 120;
static int FullRotationStepCount = 200;
static int DefaultSpeed = 500;

const uint8_t step_pins[PCF_COUNT] = {STEP0_PIN, STEP1_PIN, STEP2_PIN, STEP3_PIN};

/*globalinorion PCF objects*/
PCF8574 pcfs[PCF_COUNT] = {
    PCF8574(0x20, &Wire),
    PCF8574(0x21, &Wire),
    PCF8574(0x22, &Wire),
    PCF8574(0x23, &Wire)};

// Stepper Configuration
struct MotorConfig
{
    /*Memory of motor vars*/
    int position_setpoint;    // Target position in step pulses.
    int current_position;     // Current step position
    int max_rot_velocity;     // Maximum rotational velocity RPM
    int current_rot_velocity; /**/
    int accel_rate;           // Acceleration rate
    // int decel;       // Deceleration rate
    bool homed; /*Is this axis homed?*/

    /*Motor ctrl info*/
    uint8_t pcf_addr;
    uint8_t step_pulse_pin;
    /* Associated PCF of this unit*/
    PCF8574 *unit_pcf;
    uint8_t stepper_mode; /*0-5 defines microstepping config of DRV8825*/

    /* Memory for control loop*/
};

MotorConfig stepper_motor[PCF_COUNT];

void stepper_enable(MotorConfig *unit_config);

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    pinMode(STEP0_PIN, OUTPUT);
    pinMode(STEP1_PIN, OUTPUT);
    pinMode(STEP2_PIN, OUTPUT);
    pinMode(STEP3_PIN, OUTPUT);

    pinMode(SERVO0, OUTPUT);
    pinMode(SERVO1, OUTPUT);
    pinMode(SERVO2, OUTPUT);
    pinMode(DIGITAL0, OUTPUT);
    pinMode(DIGITAL1, OUTPUT);

    digitalWrite(STEP0_PIN, LOW);
    digitalWrite(STEP1_PIN, LOW);
    digitalWrite(STEP2_PIN, LOW);
    digitalWrite(STEP3_PIN, LOW);

    digitalWrite(SERVO0, LOW);
    digitalWrite(SERVO1, LOW);
    digitalWrite(SERVO2, LOW);
    digitalWrite(DIGITAL0, LOW);
    digitalWrite(DIGITAL1, LOW);

    /*PCB GPIO*/

    // Wire.setClock(80000);
    // Wire.beginTransmission(0x22); // Transmit to device number 44 (0x2C)
    // Wire.write(0b00111100);
    // Wire.endTransmission();

    // Initialize all PCF8574 expanders
    for (int i = 0; i < PCF_COUNT; i++)
    {
        stepper_motor[i].pcf_addr = pcfs[i].getAddress();
        stepper_motor[i].unit_pcf = &pcfs[i];

        //Serial.print("Initializing PCF8574 at address 0x");
        //Serial.println(pcfs[i].getAddress(), HEX);

        stepper_motor[i].unit_pcf->begin();

        /*Assign initial values*/
        stepper_motor[i].position_setpoint = 0;
        stepper_motor[i].current_position = 0;
        stepper_motor[i].max_rot_velocity = DEFAULT_MAX_ROT_VEL;
        stepper_motor[i].current_rot_velocity = 0;
        stepper_motor[i].accel_rate = DEFAULT_MAX_ROT_VEL;
        stepper_motor[i].homed = false;

        /*Test if PCF is OK*/
        if (!stepper_motor[i].unit_pcf->isConnected())
        {
            Serial.println("Failed!, unit can't be found on I2C bus.");
        }
        else
        {
            //Serial.println("Init suc6");
        }
        /* Seems that initial value define doesn't do it?*/
        // stepper_motor[i].unit_pcf->write8(0b01010011);
        stepper_motor[i].unit_pcf->write8(0b11111111);
        // stepper_motor[i].unit_pcf->write(NEN_PIN, 1);
        
        stepper_motor[i].unit_pcf->write(NEN_PIN, 0);
    stepper_motor[i].unit_pcf->write(NSLP_PIN, 1);
    stepper_motor[i].unit_pcf->write(NRST_PIN, 1);
    stepper_motor[i].unit_pcf->write(DIR_PIN, 1);
    stepper_motor[i].unit_pcf->write(M0_PIN, 0);
    stepper_motor[i].unit_pcf->write(M1_PIN, 0);
    stepper_motor[i].unit_pcf->write(M2_PIN, 0);
    }

    // BASIC MOTOR COMMAND "M P0d S500d A100d D100d P0d S500d A100d D100d P0d S500d A100d D100d P0d S500d A100d D100d"
    // Serial.println("All stepper motors initialized!");
    // stepper_enable(&stepper_motor[2]);
    stepper_motor[1].unit_pcf->write(DIR_PIN, 0);
}

void calculateMotorPositions(double angle, double rotation)
{
    
    int gearRatio = FullRotationStepCount * (1 / 1 / 1);
    double stepsmotor1 = angle * (((BigPullyGear / SteppermotorGear) * gearRatio) / 360);
    double stepsmotor2 = 0 - stepsmotor1;

    double rotationsteps = rotation * (((BigPullyGear / SteppermotorGear) * gearRatio) / 360);
    stepper_motor[2].position_setpoint = stepsmotor1 - rotationsteps;
    stepper_motor[3].position_setpoint = stepsmotor2 - rotationsteps;
    Serial.print(stepper_motor[2].position_setpoint);
    Serial.print(" ");
    Serial.println(stepper_motor[3].position_setpoint);
}

void moveMotorsSimultaneously()
{
    int maxSteps = 0;
    int allStepsRemaining[PCF_COUNT];
    for (int i = 0; i < PCF_COUNT; i++)
    {
        
        int stepsRemaining = abs(stepper_motor[i].position_setpoint - stepper_motor[i].current_position);
        allStepsRemaining[i] = stepsRemaining;
        if (stepsRemaining > maxSteps)
        {
            maxSteps = stepsRemaining;
        }
    }

    for (int step = 0; step < maxSteps; step++)
    {
        
        for (int i = 0; i < PCF_COUNT; i++)
        {
            int stepsRemaining = abs(stepper_motor[i].position_setpoint - stepper_motor[i].current_position);
            if (stepsRemaining > 0)
            {
                
                // Set direction
                if (stepper_motor[i].position_setpoint > stepper_motor[i].current_position)
                {
                    stepper_motor[i].unit_pcf->write(DIR_PIN, HIGH);
                }
                else
                {
                    Serial.println(i);
                    stepper_motor[i].unit_pcf->write(DIR_PIN, LOW);
                    Serial.println(i);
                }
                
                // Step the motor
                digitalWrite(step_pins[i], HIGH);
                delayMicroseconds(stepper_motor[i].max_rot_velocity);
                digitalWrite(step_pins[i], LOW);
                delayMicroseconds(stepper_motor[i].max_rot_velocity);
                // Update current position
                stepper_motor[i].current_position += (stepper_motor[i].position_setpoint > stepper_motor[i].current_position) ? 1 : -1;
                
            }
        }
    }

    // Serial.println("All motors reached target positions.");
}

void loop()
{
    calculateMotorPositions(0,90);
    stepper_motor[2].unit_pcf->write(DIR_PIN, HIGH);
    stepper_motor[1].unit_pcf->write(DIR_PIN, HIGH);
    for(int i =0; i < 200; i++){
        digitalWrite(STEP2_PIN, 1);
        delayMicroseconds(1000);
        digitalWrite(STEP2_PIN, 0);
        delayMicroseconds(1000);
    }
    stepper_motor[2].unit_pcf->write(DIR_PIN, LOW);
    stepper_motor[1].unit_pcf->write(DIR_PIN, LOW);
    for(int i =0; i < 200; i++){
        digitalWrite(STEP2_PIN, 1);
        delayMicroseconds(1000);
        digitalWrite(STEP2_PIN, 0);
        delayMicroseconds(1000);
    }

    
}

// void turnsimple(int motorindex)
// {
//     pcf[motorindex].write(DIR_PIN, HIGH); // Forward direction

//     // Move the motor
//     for (int i = 0; i < 100; i++)
//     { // Adjust steps based on microstepping
//         digitalWrite(step_pins[motorindex], HIGH);
//         delay(1); // Speed control
//         digitalWrite(step_pins[motorindex], LOW);
//         delay(1);
//     }

//     delay(1000); // Pause

//     // Change direction
//     pcf[motorindex].write(DIR_PIN, LOW);

//     for (int i = 0; i < 100; i++)
//     {
//         digitalWrite(step_pins[motorindex], HIGH);
//         delay(1);
//         digitalWrite(step_pins[motorindex], LOW);
//         delay(1);
//     }
//     delay(1000); // Pause
// }

/*Enable the stepper driver*/
void stepper_enable(MotorConfig *unit_config)
{
    unit_config->unit_pcf->write(NEN_PIN, 0);
}

void stepper_control_loop()
{

    static uint64_t last_executed;

    /*calculate how much to increment frequency by. HW timer for PWM rate? or sw PWM*/

    if ((micros() - last_executed) > CL_PERIOD_US)
    {
    }
}
