#include <Wire.h>
#include <Adafruit_PCF8574.h>

Adafruit_PCF8574 pcf;

// pcf address and pins

#define STEP_PIN 2  // Pin P0 for STEP
// #define DIR_PIN  1    // Pin P1 for DIR
// #define ENABLE_PIN 2  // Pin P2 for ENABLE (optional)

void setup() {
  Wire.begin();
  pcf.begin(0x20);
  
  // Set the pins as outputs
  pcf.pinMode(STEP_PIN, OUTPUT);
  // pcf.pinMode(DIR_PIN, OUTPUT);
  // pcf.pinMode(ENABLE_PIN, OUTPUT);
  
  // Enable the driver (optional, depending on your configuration)
  // pcf.digitalWrite(ENABLE_PIN, LOW); // Low typically enables the driver
}

void loop() {
  // Set motor direction (high = one direction, low = the other)
  // pcf.digitalWrite(DIR_PIN, HIGH); // Clockwise
  
  // Make the motor step by pulsing the STEP pin
  for (int i = 0; i < 200; i++) { // 200 steps for one revolution
    pcf.digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(1000);  // Delay between steps
    pcf.digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1000);
  }
  
  delay(1000); // Wait before changing direction
  
  // Change motor direction
  // pcf.digitalWrite(DIR_PIN, LOW); // Counterclockwise
  
  // Step again
  // for (int i = 0; i < 200; i++) {
  //   pcf.digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(1000);
  //   pcf.digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(1000);
  // }
  
  // delay(1000); // Wait before the next operation
}
