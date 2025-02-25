#include <Wire.h>
#include <PCF8574.h>

// Define PCF8574 addresses
#define PCF_COUNT 4
uint8_t PCF_ADDRESSES[PCF_COUNT] = {0x20, 0x21, 0x22, 0x23};

// Create an array of PCF8574 objects
PCF8574 pcf[PCF_COUNT] = {
    PCF8574(0x20),
    PCF8574(0x21),
    PCF8574(0x22),
    PCF8574(0x23)
};

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

        // Initialize motor configurations
        motors[i] = {0, 500, 100, 100, 0}; // Default values
    }

    Serial.println("All stepper motors initialized!");
}

void loop() {
    if (Serial.available()) {
        parseSerialCommand();
    }

    moveMotorsSimultaneously();
}

void parseSerialCommand() {
    String command = Serial.readStringUntil('\n');  // Read command
    command.trim();  // Remove whitespace

    if (command.length() == 0 || command.charAt(0) != 'M') return;

    int params[PCF_COUNT * 4]; // Stores P, S, A, D for each motor

    int matched = sscanf(command.c_str(), "M P%d S%d A%d D%d P%d S%d A%d D%d P%d S%d A%d D%d P%d S%d A%d D%d",
        &params[0], &params[1], &params[2], &params[3],
        &params[4], &params[5], &params[6], &params[7],
        &params[8], &params[9], &params[10], &params[11],
        &params[12], &params[13], &params[14], &params[15]);

    if (matched == 16) {
        for (int i = 0; i < PCF_COUNT; i++) {
            motors[i].position = params[i * 4 + 0];
            motors[i].speed = params[i * 4 + 1];
            motors[i].accel = params[i * 4 + 2];
            motors[i].decel = params[i * 4 + 3];

            Serial.print("Motor "); Serial.print(i);
            Serial.print(" -> P: "); Serial.print(motors[i].position);
            Serial.print(", S: "); Serial.print(motors[i].speed);
            Serial.print(", A: "); Serial.print(motors[i].accel);
            Serial.print(", D: "); Serial.println(motors[i].decel);
        }
    } else {
        Serial.println("Invalid command format!");
    }
}

void moveMotorsSimultaneously() {
    int maxSteps = 0;

    for (int i = 0; i < PCF_COUNT; i++) {
        int stepsRemaining = abs(motors[i].position - motors[i].current_pos);
        if (stepsRemaining > maxSteps) {
            maxSteps = stepsRemaining;
        }
    }

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

    Serial.println("All motors reached target positions.");
}
