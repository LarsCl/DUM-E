#include <Arduino.h>
#include <PCF8574.h>

struct MotorConfig {
    /*Memory of motor vars*/
    int position_setpoint;  // Target position in step pulses.
    int current_position;   // Current step position
  
    int max_rot_velocity;     // Maximum rotational velocity RPM
    int current_rot_velocity; /**/
  
    uint32_t next_step_time;
    uint32_t set_exec_time;
  
    int accel_rate;  // Acceleration rate
  
    bool homed; /*Is this axis homed?*/
  
    /*Motor ctrl info*/
    uint8_t pcf_addr;
    uint8_t step_pulse_pin;
    /* Associated PCF of this unit*/
    PCF8574 *unit_pcf;
    uint8_t stepper_mode; /*0-5 defines microstepping config of DRV8825*/
  
    /* Memory for control loop*/
  };