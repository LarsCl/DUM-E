#include <Wire.h>
#include <PCF8574.h>



// Define PCF8574 addresses
#define PCF_COUNT 4
uint8_t PCF_ADDRESSES[PCF_COUNT] = {0x23, 0x22, 0x21, 0x20};

// Create an array of PCF8574 objects
PCF8574 pcf[PCF_COUNT] = {
    PCF8574(0x23),
    PCF8574(0x22),
    PCF8574(0x21),
    PCF8574(0x20)
};

//Gear ratios and stepcount
static int SteppermotorGear = 18;
static int BigPullyGear = 120;
static int FullRotationStepCount = 200;
static int DefaultSpeed = 500;

// Step pins for each motor
#define STEP0_PIN 2
#define STEP1_PIN 3
#define STEP2_PIN 4
#define STEP3_PIN 5
uint8_t step_pins[PCF_COUNT] = {STEP0_PIN, STEP1_PIN, STEP2_PIN, STEP3_PIN};

// PCF8574 Pin Assignments
#define FLT_PIN   0
#define RST_PIN   1
#define M2_PIN    2
#define M1_PIN    3
#define EN_PIN    4
#define M0_PIN    5
#define SLP_PIN   6
#define DIR_PIN   7

// Stepper Configuration
struct MotorConfig {
    int position;    // Target position
    int speed;       // Delay per step (µs)
    int accel;       // Acceleration rate
    int decel;       // Deceleration rate
    int current_pos; // Current step position
};

MotorConfig motors[PCF_COUNT];

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize all PCF8574 expanders
    for (int i = 0; i < PCF_COUNT; i++) {
        Serial.print("Initializing PCF8574 at address 0x");
        Serial.println(PCF_ADDRESSES[i], HEX);

        // Reset and enable stepper driver
        pcf[i].write(RST_PIN, HIGH);  // Keep driver out of reset
        pcf[i].write(SLP_PIN, HIGH);  // Wake up driver
        pcf[i].write(EN_PIN, LOW);    // Enable motor (LOW = enabled)

        // Set microstepping (Full Step Mode)
        pcf[i].write(M0_PIN, LOW);
        pcf[i].write(M1_PIN, LOW);
        pcf[i].write(M2_PIN, LOW);

        // Set direction (initial)
        pcf[i].write(DIR_PIN, HIGH);

        // Set step pin as output
        pinMode(step_pins[i], OUTPUT);
        digitalWrite(step_pins[i], LOW);

        // Initialize motor configurations
        motors[i] = {0, 500, 0, 0, 0}; // Default values
    }
    //BASIC MOTOR COMMAND "M P0d S500d A100d D100d P0d S500d A100d D100d P0d S500d A100d D100d P0d S500d A100d D100d"
    Serial.println("All stepper motors initialized!");
}

void loop() {
    for(int i = 0; i< 4; i++){
      pcf[i].write(SLP_PIN, HIGH);
      pcf[i].write(RST_PIN, HIGH);
      pcf[i].write(EN_PIN, LOW);
    }
    if (Serial.read() != -1) {
      calculateMotorPositions(0,720);
    }
    
    moveMotorsSimultaneously();
}

void moveMotorsSimultaneously() {
    int maxSteps = 0;
    int allStepsRemaining[PCF_COUNT]; 
    for (int i = 0; i < PCF_COUNT; i++) {
        int stepsRemaining = abs(motors[i].position - motors[i].current_pos);
        allStepsRemaining[i] = stepsRemaining;
        if (stepsRemaining > maxSteps) {
            maxSteps = stepsRemaining;
        }
    }

    // for (int i = 0; i < PCF_COUNT; i++) {
    //     if(allStepsRemaining > 1){
    //       motors[i].speed = ((double)maxSteps * (double)DefaultSpeed) / (double)allStepsRemaining[i];
    //     }
    // }

    for (int step = 0; step < maxSteps; step++) {
        for (int i = 0; i < PCF_COUNT; i++) {
            int stepsRemaining = abs(motors[i].position - motors[i].current_pos);
            if (stepsRemaining > 0) {
                // Set direction
                if (motors[i].position > motors[i].current_pos) {
                    pcf[i].write(DIR_PIN, HIGH);  
                } else {
                    pcf[i].write(DIR_PIN, LOW);   
                }

                // Step the motor
                digitalWrite(step_pins[i], HIGH);
                delayMicroseconds(motors[i].speed);
                digitalWrite(step_pins[i], LOW);
                delayMicroseconds(motors[i].speed);

                // Acceleration / Deceleration Handling
                if (step < stepsRemaining / 3) {
                    motors[i].speed -= motors[i].accel;
                } else if (step > (stepsRemaining - (stepsRemaining / 3))) {
                    motors[i].speed += motors[i].decel;
                }

                // Update current position
                motors[i].current_pos += (motors[i].position > motors[i].current_pos) ? 1 : -1;
            }
        }
    }

    //Serial.println("All motors reached target positions.");
}

void calculateMotorPositions(double angle, double rotation){
  int gearRatio = FullRotationStepCount * (1/1/1);
  double stepsmotor1 = angle * (((BigPullyGear/ SteppermotorGear) * gearRatio) /360);
  double stepsmotor2 = 0 - stepsmotor1;
  
  double rotationsteps = rotation * (((BigPullyGear/ SteppermotorGear) * gearRatio) /360);
  motors[2].position = stepsmotor1 - rotationsteps;
  motors[3].position = stepsmotor2 - rotationsteps;
  Serial.print(motors[2].position);
  Serial.print(" ");
  Serial.print(motors[3].position);
} 

void turnsimple(int motorindex){
  pcf[motorindex].write(DIR_PIN, HIGH);  // Forward direction

    // Move the motor
    for (int i = 0; i < 100; i++) {  // Adjust steps based on microstepping
        digitalWrite(step_pins[motorindex], HIGH);
        delay(1);  // Speed control
        digitalWrite(step_pins[motorindex], LOW);
        delay(1);
    }

    delay(1000);  // Pause

    // Change direction
    pcf[motorindex].write(DIR_PIN, LOW);

    for (int i = 0; i < 100; i++) {
        digitalWrite(step_pins[motorindex], HIGH);
        delay(1);
        digitalWrite(step_pins[motorindex], LOW);
        delay(1);
    }
    delay(1000);  // Pause
}
