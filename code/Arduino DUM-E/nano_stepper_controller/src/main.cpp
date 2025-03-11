#include <Arduino.h>
#include <PCF8574.h>
#include <Wire.h>

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

// Step pins for each motor (ARDUINO)
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

/* Is (60*10^6)/ (RPM * 200), returns uS per rotation */
#define RPM_TO_STEP_US(x) (0x3938700 / (x * 200))

/* Frequency of stepper control loop in Hertz */
#define CL_FREQ 1000
#define CL_PERIOD_US (0xF4240 / CL_FREQ)

/* Maximum STEP pulse frequency: 1/((60/120)/200)=400Hz */
#define DEFAULT_MAX_ROT_VEL 120 /*120RPM*/
/* Acceleration is (45/60) (rev/min)/min, thus 0.75 min/s^2 */
#define DEFAULT_ROT_ACCEL 45 /*45RPM // 0.75 rev/min^2 */

#define DEFAULT_DRV8825_MODE 0 /*fullstep*/

// Gear ratios and stepcount
static int SteppermotorGear = 18;
static int BigPullyGear = 120;
static int FullRotationStepCount = 200;
static int DefaultSpeed = 500;

long timer = 0;

const uint8_t step_pins[PCF_COUNT] = {STEP0_PIN, STEP1_PIN, STEP2_PIN,
                                      STEP3_PIN};

/*globalinorion PCF objects*/
PCF8574 pcfs[PCF_COUNT] = {PCF8574(0x20, &Wire), PCF8574(0x21, &Wire),
                           PCF8574(0x22, &Wire), PCF8574(0x23, &Wire)};

// Stepper Configuration
struct MotorConfig {
  /*Memory of motor vars*/
  int position_setpoint;  // Target position in step pulses.
  int current_position;   // Current step position

  int max_rot_velocity; /* Maximum rotational velocity in RPM used to limit */
  int current_rot_velocity; /* Current velocity in RPM*/
  int accel_rate;           // Acceleration rate

  /* A scale to slow accel and velocity for synchronous movement of axes*/
  float movement_speed_scale;

  uint32_t next_step_time;
  uint32_t step_toggle_time;
  uint32_t set_exec_time;

  bool homed;         /*Is this axis homed?*/
  bool block_control; /* Flag to software block driving this motor. true is
                         blocking*/

  /*Motor ctrl info*/
  uint8_t pcf_addr;
  uint8_t step_pulse_pin;
  /* Associated PCF of this unit*/
  PCF8574 *unit_pcf;
  uint8_t stepper_mode; /*0-5 defines microstepping config of DRV8825*/
};

MotorConfig stepper_motor[PCF_COUNT];

/* Hardware enable or disable stepper motor, 1 is ON*/
void set_stepper_drive(MotorConfig *unit_config, bool onoffvalue);
/* Software block driving of a specific motor*/
void set_stepper_block(MotorConfig *unit_config, bool onoffvalue);

/* 0 is fullstep, 1 is 1/2, 2 is 1/4, 3 is 1/8, 4 is 1/16, 5 is 1/32 */
void set_stepper_stepsize(MotorConfig *unit_config, uint8_t stepsize);
/* Position to move this axis to a certain setpoint position*/
void set_stepper_setpoint(MotorConfig *unit_config, int setpointpulse);
/* Change acceleration rate of a motor */
void set_stepper_rate(MotorConfig *unit_config, int accelrate);
/* Change max velocity of motor */
void set_stepper_velocity(MotorConfig *unit_config, int maxvelocity);

/* Polling function that controls pulses, calculates when the next pulse should
 * be when a step is done.
 */
void stepper_control_loop();

void setup() {
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

  // Initialize all PCF8574 expanders and cfg arduino pin
  for (int i = 0; i < PCF_COUNT; i++) {
    stepper_motor[i].pcf_addr = pcfs[i].getAddress();
    stepper_motor[i].unit_pcf = &pcfs[i];

    Serial.print("Initializing PCF8574 at address 0x");
    Serial.println(pcfs[i].getAddress(), HEX);

    stepper_motor[i].unit_pcf->begin();

    /*Assign initial values*/
    stepper_motor[i].position_setpoint = 0;
    stepper_motor[i].current_position = 0;
    stepper_motor[i].max_rot_velocity = DEFAULT_MAX_ROT_VEL;
    stepper_motor[i].current_rot_velocity = 0;
    stepper_motor[i].accel_rate = DEFAULT_MAX_ROT_VEL;
    stepper_motor[i].movement_speed_scale = 1.0f;
    stepper_motor[i].homed = false;
    stepper_motor[i].block_control = true;
    stepper_motor[i].step_pulse_pin = step_pins[i];

    /* Set initial PCF GPIO */
    stepper_motor[i].unit_pcf->write(NEN_PIN, 0);
    stepper_motor[i].unit_pcf->write(NSLP_PIN, 1);
    stepper_motor[i].unit_pcf->write(NRST_PIN, 1);
    stepper_motor[i].unit_pcf->write(DIR_PIN, 1);
    stepper_motor[i].unit_pcf->write(M0_PIN, 0);
    stepper_motor[i].unit_pcf->write(M1_PIN, 0);
    stepper_motor[i].unit_pcf->write(M2_PIN, 0);

    /*Test if PCF is OK (reachable on I2C)*/
    if (!stepper_motor[i].unit_pcf->isConnected()) {
      Serial.println("Failed!, unit can't be found on I2C bus.");
    } else {
      Serial.println("Init suc6");
    }

    /* TODO, this is for debug operation. This enables the motor and sets the
     * micro steppng to max */
    stepper_motor[i].unit_pcf->write8(0b11111111);
  }

  // BASIC MOTOR COMMAND "M P0d S500d A100d D100d P0d S500d A100d D100d P0d
  // S500d A100d D100d P0d S500d A100d D100d" Serial.println("All stepper motors
  // initialized!"); stepper_enable(&stepper_motor[2]);
}

void calculateMotorPositions(double angle, double rotation) {
  int gearRatio = FullRotationStepCount * (1 / 1 / 1);
  double stepsmotor1 =
      angle * (((BigPullyGear / SteppermotorGear) * gearRatio) / 360);
  double stepsmotor2 = 0 - stepsmotor1;

  double rotationsteps =
      rotation * (((BigPullyGear / SteppermotorGear) * gearRatio) / 360);
  stepper_motor[2].position_setpoint = stepsmotor1 - rotationsteps;
  stepper_motor[3].position_setpoint = stepsmotor2 - rotationsteps;
  Serial.print(stepper_motor[2].position_setpoint);
  Serial.print(" ");
  Serial.println(stepper_motor[3].position_setpoint);
  stepper_motor[2].next_step_time =
      micros() + stepper_motor[2].max_rot_velocity;
  stepper_motor[3].next_step_time =
      micros() + stepper_motor[3].max_rot_velocity;
}

void moveMotorsSimultaneously() {
  int maxSteps = 0;
  int allStepsRemaining[PCF_COUNT];
  for (int i = 0; i < PCF_COUNT; i++) {
    int stepsRemaining = abs(stepper_motor[i].position_setpoint -
                             stepper_motor[i].current_position);
    allStepsRemaining[i] = stepsRemaining;
    if (stepsRemaining > maxSteps) {
      maxSteps = stepsRemaining;
    }
  }

  for (int step = 0; step < maxSteps; step++) {
    for (int i = 0; i < PCF_COUNT; i++) {
      int stepsRemaining = abs(stepper_motor[i].position_setpoint -
                               stepper_motor[i].current_position);
      if (stepsRemaining > 0) {
        // Set direction
        if (stepper_motor[i].position_setpoint >
            stepper_motor[i].current_position) {
          stepper_motor[i].unit_pcf->write(DIR_PIN, HIGH);
        } else {
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
        stepper_motor[i].current_position +=
            (stepper_motor[i].position_setpoint >
             stepper_motor[i].current_position)
                ? 1
                : -1;
      }
    }
  }

  // Serial.println("All motors reached target positions.");
}

void movemotors() {
  for (int i = 0; i < PCF_COUNT; i++) {
    int stepsRemaining = abs(stepper_motor[i].position_setpoint -
                             stepper_motor[i].current_position);
    if (stepsRemaining > 0 && micros() >= stepper_motor[i].next_step_time) {
      // Set direction
      if (stepper_motor[i].position_setpoint >
          stepper_motor[i].current_position) {
        stepper_motor[i].unit_pcf->write(DIR_PIN, HIGH);
      } else {
        stepper_motor[i].unit_pcf->write(DIR_PIN, LOW);
      }
      // TODO: check current velocity, calculate and compare whether (setpoint -
      // current position) is higher than distance needed to decelarate from
      // current velocity.
      stepper_motor[i].set_exec_time = micros();
      stepper_motor[i].next_step_time = micros() + 2000;
      // Update current position
      stepper_motor[i].current_position += (stepper_motor[i].position_setpoint >
                                            stepper_motor[i].current_position)
                                               ? 1
                                               : -1;
    } else if (stepsRemaining > 0 &&
               micros() >= stepper_motor[i].set_exec_time) {
      if (micros() <=
          stepper_motor[i].set_exec_time + (stepper_motor[i].next_step_time -
                                            stepper_motor[i].set_exec_time) /
                                               2) {
        // Serial.print("1");
        digitalWrite(step_pins[i], LOW);
      } else {
        // Step the motor HIGH
        // Serial.print("2");
        digitalWrite(step_pins[i], HIGH);
      }
    }
  }
}

void loop() {
  // if(stepper_motor[2].current_position ==
  // stepper_motor[2].position_setpoint){
  //     calculateMotorPositions(90,0);
  // }

  // stepper_motor[2].unit_pcf->write(DIR_PIN, HIGH);
  // stepper_motor[1].unit_pcf->write(DIR_PIN, HIGH);
  // for (int i = 0; i < 200; i++) {
  //   digitalWrite(STEP2_PIN, 1);
  //   delayMicroseconds(1000);
  //   digitalWrite(STEP2_PIN, 0);
  //   delayMicroseconds(1000);
  // }
  // stepper_motor[2].unit_pcf->write(DIR_PIN, LOW);
  // stepper_motor[1].unit_pcf->write(DIR_PIN, LOW);
  // for (int i = 0; i < 200; i++) {
  //   digitalWrite(STEP2_PIN, 1);
  //   delayMicroseconds(1000);
  //   digitalWrite(STEP2_PIN, 0);
  //   delayMicroseconds(1000);
  // }
  // movemotors();
}

/*Control the enable pin oft the stepper driver module*/
void set_stepper_drive(MotorConfig *unit_config, bool on_off_value) {
  unit_config->unit_pcf->write(NEN_PIN, 0);
}

void set_stepper_stepsize(MotorConfig *unit_config, uint8_t stepsize) {
  if ((stepsize > 5)) {
    Serial.println("ERROR! Requested set step size out of bounds.");
    return;
  }
  /* bit manipulation baybay*/
  unit_config->unit_pcf->write(M0_PIN, (stepsize && 1));
  unit_config->unit_pcf->write(M1_PIN, ((stepsize >> 1) && 1));
  unit_config->unit_pcf->write(M2_PIN, ((stepsize >> 2) && 1));
}

void set_stepper_setpoint(MotorConfig *unit_config, int setpointpulse) {}

void set_stepper_rate(MotorConfig *unit_config, int accelerate) {
  /*Beware for too high acceleration rates*/
  unit_config->accel_rate = accelerate;
}

void set_stepper_velocity(MotorConfig *unit_config, int maxvelocity) {
  unit_config->max_rot_velocity = maxvelocity;
}

void stepper_control_loop() {
  static uint8_t mtr_index;

  /* operation thoughts:
  The function starts off by checking whether [current position == setpoint]
  if setpoint value is higher or lower, corresponding movement direction is set
  by setting or clearing the DIR flag.
  Then depending on set acceleration value and current velocity, the clock time
  of when the motor should perform a whole STEP is calculated.

  This ensures, that the step time is changed only when a single PERIOD of step
  has been taken. And the control loop does not need extra timing memory of when
  the next stepper pulse should occur (as this is in the motor struct.)

  Thus TLDR:
  Calculate the next STEP time, every step pulse period, using acceleration val
  and current velocity.

  1-> check setpoint - current pos
  2-> if not 0, set direction pin
  3-> check if current millis time >= next step time
  4-> if yes, check current velocity, calculate and compare
      whether (setpoint - current position) is higher than distance needed to
      decelarate from current velocity.
  5-> If error is higher, check if current velocity + acceleration step is under
      maximum velocity.
  6-> If there is room to accelerate, do it, if no, dont.
  7-> update execution time and next step update time.


  -> In case millis time was (3) under next step time, check if current millis
    is above or below ((next update time - exec time)/2) + exec time. If under,
    STEP pulse pin is LOW, if above, step pulse pin is HIGH. This ensures async
    operation.
*/

  if (stepper_motor[mtr_index].block_control) {
    // Serial.println("This unit is blocked, skipping");
    mtr_index++;
    mtr_index %= (PCF_COUNT + 1);
    return;
  }

  if (!stepper_motor[mtr_index].homed) {
    // Serial.println("This unit isnt homed, skipping");
    mtr_index++;
    mtr_index %= (PCF_COUNT + 1);
    return;
  }

  /* We check if there is a position error*/

  int pos_error = (stepper_motor[mtr_index].position_setpoint -
                   stepper_motor[mtr_index].current_position);

  if (pos_error == 0) {
    Serial.println("Is on SP.");
    mtr_index++;
    mtr_index %= (PCF_COUNT + 1);
    return;
  }

  /* A step elapsed, time to set next time goal*/
  if (micros() >= stepper_motor[mtr_index].next_step_time) {
    /*set direction of movement*/
    if (pos_error > 0) {
      stepper_motor[mtr_index].unit_pcf->write(
          stepper_motor[mtr_index].step_pulse_pin, 1);
    } else {
      stepper_motor[mtr_index].unit_pcf->write(
          stepper_motor[mtr_index].step_pulse_pin, 0);
    }

    /*calculate current step period time from velocity  */
    /* uS per rotation is (60*10^6)/rpm
     * and (uS/Rev)/200 = step period in uS, TODO, autoadjust with microstep
     * setting  */

    /*TODO, maybe using Rev/M was a bad idea, why not use Rev/uS? or uS/rev
     * directly? ask teammates
     */

    uint32_t step_per_us =
        RPM_TO_STEP_US(stepper_motor[mtr_index].current_rot_velocity);

    /* Acceleration increment*/
    /* Get how much to change this step time/freq by how much time elapsed since
     *last accelaration increment */
    uint32_t time_delta_us = micros() - stepper_motor[mtr_index].set_exec_time;

    /*TODO, for very small increments (very slow accelaration), float may be
     * needed for consistency, re calculated, we deffo need it*/
    uint32_t accel_inc_us =
        RPM_TO_STEP_US(stepper_motor[mtr_index].accel_rate) / time_delta_us;

    /*Check if we can accelarate further, keeping in mind for leaving 1 accel
     * increment of headroom from current velocity */
    stepper_motor[mtr_index].set_exec_time = micros();

    if ((step_per_us - accel_inc_us) >
        RPM_TO_STEP_US(stepper_motor[mtr_index].max_rot_velocity)) {
      stepper_motor[mtr_index].next_step_time =
          stepper_motor[mtr_index].set_exec_time + (step_per_us - accel_inc_us);
    } else {
      stepper_motor[mtr_index].next_step_time =
          stepper_motor[mtr_index].set_exec_time + step_per_us;
    }

    /*set toggle moment */
    stepper_motor[mtr_index].step_toggle_time =
        stepper_motor[mtr_index].set_exec_time + (step_per_us / 2);
  }

  /*poll for step pin toggle*/

  if (micros() > stepper_motor[mtr_index].step_toggle_time) {
    digitalWrite(stepper_motor[mtr_index].step_pulse_pin, HIGH);
  } else {
    digitalWrite(stepper_motor[mtr_index].step_pulse_pin, LOW);
  }

  /*TODO decelerate*/
}
