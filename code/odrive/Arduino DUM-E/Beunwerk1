#include <Wire.h>
#include <Adafruit_PCF8574.h>

Adafruit_PCF8574 pcf;

#define STEP_PIN0 5
#define STEP_PIN3 6
#define STEP_PIN2 7
#define STEP_PIN1 8
#define DIR_PIN 1
#define ENABLE_PIN 2

void setup() {
  Wire.begin();           //I2C
  pcf.begin(0x20);

  pcf.pinMode(STEP_PIN0, OUTPUT);
  pcf.pinMode(STEP_PIN3, OUTPUT);
  pcf.pinMode(STEP_PIN2, OUTPUT);
  pcf.pinMode(STEP_PIN1, OUTPUT);
  pcf.pinMode(DIR_PIN, OUTPUT);
  pcf.pinMode(ENABLE_PIN, OUTPUT);

  pcf.digitalWrite(ENABLE_PIN, LOW); // LOW to enable, HIGH to disable
}

void loop() {
//   pcf8574.digitalWrite(DIR_PIN, HIGH);
  
//   for (int i = 0; i < 1000; i++) {
//     pcf.digitalWrite(STEP_PIN, HIGH);
//     delayMicroseconds(1000);
//     pcf.digitalWrite(STEP_PIN, LOW);
//     delayMicroseconds(1000);
//   }

//   pcf.digitalWrite(DIR_PIN, LOW);
//   delay(1000);


// Wire.beginTransmission(8); // transmit to device #8
// Wire.write(ledVal);        // sends the given value
// Wire.endTransmission();    // stop transmitting
}