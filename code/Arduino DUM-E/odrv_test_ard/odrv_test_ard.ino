#include <ODriveUART.h>

// Use hardware UART (TX=1, RX=0) on Arduino Nano 33 IoT
#define ODRIVE_SERIAL Serial1  // Use Serial1 instead of SoftwareSerial
unsigned long baudrate = 115200;  // Match this with ODrive settings

ODriveUART odrive(ODRIVE_SERIAL);

void setup() {
    // Start serial communication
    ODRIVE_SERIAL.begin(baudrate);  // ODrive UART
    Serial.begin(115200);  // Serial Monitor debugging

    delay(10);

    Serial.println("Waiting for ODrive...");
    while (odrive.getState() == AXIS_STATE_UNDEFINED) {
        delay(100);
    }

    Serial.println("Found ODrive!");

    Serial.print("DC voltage: ");
    Serial.println(odrive.getParameterAsFloat("vbus_voltage"));

    Serial.println("Enabling closed-loop control...");
    while (odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        odrive.clearErrors();
        odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
        delay(10);
    }

    Serial.println("ODrive running!");
}

void loop() {
    float SINE_PERIOD = 2.0f;  // Period of sine wave motion in seconds
    float t = millis() * 0.001;  // Convert to seconds
    float phase = t * (TWO_PI / SINE_PERIOD);

    odrive.setPosition(
        sin(phase),  // Position command
        cos(phase) * (TWO_PI / SINE_PERIOD)  // Velocity feedforward (optional)
    );

    ODriveFeedback feedback = odrive.getFeedback();
    Serial.print("pos: ");
    Serial.print(feedback.pos);
    Serial.print(", vel: ");
    Serial.println(feedback.vel);

    delay(10);  // Small delay for smooth operation
}
