#include "motorStruct.cpp"
#include <Arduino.h>
#include <stdlib.h>

#ifndef __MOTOMOTO___


#define DIR_PIN 7

// Gear ratios and stepcount
static int SteppermotorGear = 18;
static int BigPullyGear = 120;
static int FullRotationStepCount = 200;
static int DefaultSpeed = 500;


// void moveMotorsSimultaneously(MotorConfig motors[])
// {
//     int maxSteps = 0;
//     int allStepsRemaining[motors.le];
//     for (int i = 0; i < PCF_COUNT; i++)
//     {
//         int stepsRemaining = abs(motors[i].position - motors[i].current_pos);
//         allStepsRemaining[i] = stepsRemaining;
//         if (stepsRemaining > maxSteps)
//         {
//             maxSteps = stepsRemaining;
//         }
//     }

//     // for (int i = 0; i < PCF_COUNT; i++) {
//     //     if(allStepsRemaining > 1){
//     //       motors[i].speed = ((double)maxSteps * (double)DefaultSpeed) /
//     (double)allStepsRemaining[i];
//     //     }
//     // }

//     for (int step = 0; step < maxSteps; step++)
//     {
//         for (int i = 0; i < PCF_COUNT; i++)
//         {
//             int stepsRemaining = abs(motors[i].position -
//             motors[i].current_pos); if (stepsRemaining > 0)
//             {
//                 // Set direction
//                 if (motors[i].position > motors[i].current_pos)
//                 {
//                     pcf[i].write(DIR_PIN, HIGH);
//                 }
//                 else
//                 {
//                     pcf[i].write(DIR_PIN, LOW);
//                 }

//                 // Step the motor
//                 digitalWrite(step_pins[i], HIGH);
//                 delayMicroseconds(motors[i].speed);
//                 digitalWrite(step_pins[i], LOW);
//                 delayMicroseconds(motors[i].speed);

//                 // Acceleration / Deceleration Handling
//                 if (step < stepsRemaining / 3)
//                 {
//                     motors[i].speed -= motors[i].accel;
//                 }
//                 else if (step > (stepsRemaining - (stepsRemaining / 3)))
//                 {
//                     motors[i].speed += motors[i].decel;
//                 }

//                 // Update current position
//                 motors[i].current_pos += (motors[i].position >
//                 motors[i].current_pos) ? 1 : -1;
//             }
//         }
//     }

//     // Serial.println("All motors reached target positions.");
// }

void calculateMotorPositions(double angle, double rotation, MotorConfig motors[])
{
    int gearRatio = FullRotationStepCount * (1 / 1 / 1);
    double stepsmotor1 = angle * (((BigPullyGear / SteppermotorGear) *
    gearRatio) / 360); double stepsmotor2 = 0 - stepsmotor1;

    double rotationsteps = rotation * (((BigPullyGear / SteppermotorGear) *
    gearRatio) / 360); motors[2].position_setpoint = stepsmotor1 - rotationsteps;
    motors[3].position_setpoint = stepsmotor2 - rotationsteps;
    Serial.print(motors[2].position_setpoint);
    Serial.print(" ");
    Serial.print(motors[3].position_setpoint);
}

void turnsimple(int motorindex, MotorConfig motors[])
{
    motors[motorindex].unit_pcf->write(DIR_PIN, HIGH); // Forward direction

    // Move the motor
    for (int i = 0; i < 100; i++)
    { // Adjust steps based on microstepping
        digitalWrite(motors[motorindex].step_pulse_pin, HIGH);
        delay(1); // Speed control
        digitalWrite(motors[motorindex].step_pulse_pin, LOW);
        delay(1);
    }

    delay(1000); // Pause

    // Change direction
    motors[motorindex].unit_pcf->write(DIR_PIN, LOW);

    for (int i = 0; i < 100; i++)
    { // Adjust steps based on microstepping
        digitalWrite(motors[motorindex].step_pulse_pin, HIGH);
        delay(1); // Speed control
        digitalWrite(motors[motorindex].step_pulse_pin, LOW);
        delay(1);
    }
    delay(1000); // Pause
}


#endif