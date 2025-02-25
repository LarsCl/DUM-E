#include <Wire.h>
#include <PCF8574.h>

#define PCF_ADDRESS 0x23  // PCF8574P Address
PCF8574 pcf(PCF_ADDRESS);

#define STEP_PIN 2  // Step signal directly from Arduino

// PCF8574 Pin Assignments
#define FLT_PIN   0
#define RST_PIN   1
#define M2_PIN    2
#define M1_PIN    3
#define EN_PIN    4
#define M0_PIN    5
#define SLP_PIN   6
#define DIR_PIN   7

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize PCF8574 outputs (Write LOW for OUTPUT, HIGH for INPUT mode)
    pcf.write(RST_PIN, HIGH);  // HIGH to keep driver out of reset
    pcf.write(SLP_PIN, HIGH);  // HIGH to wake up the driver
    pcf.write(EN_PIN, LOW);    // LOW to enable the driver

    // Set microstepping (now set to FUll step)
    pcf.write(M0_PIN, LOW);
    pcf.write(M1_PIN, LOW);
    pcf.write(M2_PIN, LOW);

    Serial.println("Stepper motor setup done...");
    
    pinMode(STEP_PIN, OUTPUT); // Step pin is directly controlled by Arduino
}

void loop() {
    // Set direction
    pcf.write(DIR_PIN, HIGH);  // Forward direction

    // Move the motor
    for (int i = 0; i < 200; i++) {  // Adjust steps based on microstepping
        digitalWrite(STEP_PIN, HIGH);
        delay(1);  // Speed control
        digitalWrite(STEP_PIN, LOW);
        delay(1);
    }

    delay(1000);  // Pause

    // Change direction
    pcf.write(DIR_PIN, LOW);

    for (int i = 0; i < 200; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delay(1);
        digitalWrite(STEP_PIN, LOW);
        delay(1);
    }
    Serial.print(pcf.read(FLT_PIN));
    delay(1000);  // Pause
}
