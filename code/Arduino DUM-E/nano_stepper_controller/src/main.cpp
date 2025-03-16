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
#define STEP1_PIN 5
#define STEP2_PIN 4
#define STEP3_PIN 3

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
/* Works RPM -> STEP uS and vice versa/*/
// #define RPM_TO_STEP_US(x) (0x3938700 / (x * 200))
#define RPM_TO_STEP_US_FLT(x) (600000.0f / (x * 2.0f))
#define RPM_TO_STEP_US_SQRD(x) (600000000000.0 / (x * 2.0))
/* As we have 'infinite' resolution, we need to define a MINIMUM and MAXIMUM
 * STEP PERIOD TIME. Otherwisea RPM of 0 equals infinite period time.*/
// #define STEPPER_MIN_RPM 10

/* Frequency of stepper control loop in Hertz */
// #define CL_FREQ 1000
// #define CL_PERIOD_US (0xF4240 / CL_FREQ)

/* Maximum STEP pulse frequency: 1/((60/120)/200)=400Hz */
#define DEFAULT_MAX_ROT_VEL 120 /*120RPM*/
/* Acceleration is (45/60) (rev/min)/min, thus 0.75 min/s^2 */
#define DEFAULT_ROT_ACCEL 45 /*45RPM // 0.75 rev/min^2 */

#define TESTAXES 0
#define SETPOINTTEST 200

#define DEFAULT_DRV8825_MODE 0 /*fullstep*/

// Gear ratios and stepcount
static int SteppermotorGear = 20;
static int BigPullyGear = 104;

static int BaseBigGear = 365;
static int BaseStepperGear = 16;

static int RotationBigGear = 40;
static int RotationSmallGear = 10;

static int FullRotationStepCount = 200;
static int DefaultSpeed = 500;

long timer = 0;

const uint8_t step_pins[PCF_COUNT] = {STEP3_PIN, STEP2_PIN, STEP1_PIN,
                                      STEP0_PIN};

/*globalinorion PCF objects*/
PCF8574 pcfs[PCF_COUNT] = {PCF8574(0x20, &Wire), PCF8574(0x21, &Wire),
                           PCF8574(0x22, &Wire), PCF8574(0x23, &Wire)};

// Stepper Configuration
struct MotorConfig {
  /*Memory of motor vars*/
  int position_setpoint;  // Target position in step pulses.
  int current_position;   // Current step position

  double accel_rate;      /*Accel/Decel rate modifier in uS per motor step*/
  float accel_sqrt;       /* sqrt(Taccel) for convenient/fast calcs */
  float max_rot_velocity; /* Holds minimum step period time in uS. */
  float min_rot_velocity; /* Calculated from accel value, Tmax < sqrt(Taccel)*/
  float current_rot_velocity;

  /* Memory for motor driver */
  uint32_t next_step_time;
  uint32_t step_toggle_time;
  uint32_t set_exec_time;
  float movement_speed_scale; /* For the synchonizer, movement vel scale*/

  /* Memory for movement profiler */
  uint32_t accel_dowto; /*Accel until x steps*/
  uint32_t decel_from;  /*Start decel from x steps*/

  bool on_sp;         /*on setpoint flag*/
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

/*Struct holding poses for apply_pose(), STEP count 0 is 180 degrees.*/
// struct SystemPose {
//   uint16_t angle_0;
//   uint16_t angle_1;
//   uint16_t angle_2;
//   uint16_t angle_3;
// } motor_angle;
/*Angle positions desired for each motor*/

uint16_t motor_angles[PCF_COUNT];

/* Hardware enable or disable stepper motor, 1 is ON*/
void set_stepper_drive(MotorConfig *unit_config, bool onoffvalue);
/* Software block driving of a specific motor*/
void set_stepper_block(MotorConfig *unit_config, bool onoffvalue);
/* Eep. */
void set_stepper_sleep(MotorConfig *unit_config, bool on_off_value);

/* 0 is fullstep, 1 is 1/2, 2 is 1/4, 3 is 1/8, 4 is 1/16, 5 is 1/32 */
void set_stepper_stepsize(MotorConfig *unit_config, uint8_t stepsize);
/* Position to move this axis to a certain setpoint position*/
void set_stepper_setpoint(MotorConfig *unit_config, int setpointpulse);
/* Change acceleration rate of a motor */
void set_stepper_rate(MotorConfig *unit_config, int accelrate);
/* Change max velocity of motor */
void set_stepper_velocity(MotorConfig *unit_config, int maxvelocity);

/*Return current pos - setpoint error*/
int get_stepper_error(MotorConfig *unit_config);

// bool check_faults(MotorConfig *unit_config);

void motion_profiler(MotorConfig *unit_config);

/*Configures system speed scale for motors from current to setpoint positions,
 * also sets setpoints. Angles are in deca degrees. 180 => 1800*/
void apply_pose(uint16_t wanted_pose[PCF_COUNT]);

/* Polling function that controls pulses, calculates when the next pulse
 * should be when a step is done.
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

    stepper_motor[i].accel_rate = RPM_TO_STEP_US_SQRD(DEFAULT_ROT_ACCEL);
    stepper_motor[i].accel_sqrt = sqrt(stepper_motor[i].accel_rate);

    stepper_motor[i].max_rot_velocity =
        (float)RPM_TO_STEP_US_FLT(DEFAULT_MAX_ROT_VEL) /
        (float)pow(2, DEFAULT_DRV8825_MODE);
    stepper_motor[i].min_rot_velocity = stepper_motor[i].accel_sqrt - 1.0;
    stepper_motor[i].current_rot_velocity = stepper_motor[i].min_rot_velocity;

    stepper_motor[i].next_step_time = 0;
    stepper_motor[i].step_toggle_time = 0;
    stepper_motor[i].set_exec_time = 0;

    stepper_motor[i].movement_speed_scale = 1.0f;

    stepper_motor[i].accel_dowto = 0;
    stepper_motor[i].decel_from = 0;

    stepper_motor[i].on_sp = true;
    stepper_motor[i].homed = false;
    stepper_motor[i].block_control = true;
    stepper_motor[i].step_pulse_pin = step_pins[i];

    stepper_motor[i].stepper_mode = DEFAULT_DRV8825_MODE;
    /* Set initial PCF GPIO */
    stepper_motor[i].unit_pcf->write(NEN_PIN, 1);  /*1 is disabled*/
    stepper_motor[i].unit_pcf->write(NSLP_PIN, 0); /*0 is sleeping*/
    stepper_motor[i].unit_pcf->write(NRST_PIN, 0); /*0 is reseting*/
    stepper_motor[i].unit_pcf->write(DIR_PIN, 1);
    stepper_motor[i].unit_pcf->write(M0_PIN, 0); /*fullstep*/
    stepper_motor[i].unit_pcf->write(M1_PIN, 0); /*fullstep*/
    stepper_motor[i].unit_pcf->write(M2_PIN, 0); /*fullstep*/

    /*Test if PCF is OK (reachable on I2C)*/
    if (!stepper_motor[i].unit_pcf->isConnected()) {
      Serial.println("Failed!, unit can't be found on I2C bus.");
    } else {
      Serial.println("Init suc6");
    }
  }
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
  static int test_states = 0;
  static uint32_t testing_timer;
  switch (test_states) {
    case 0:
      if (millis() - testing_timer > 2000) {
        /*Enable hardware, but block SW control*/
        for (int i = 0; i < PCF_COUNT; i++) {
          set_stepper_block(&stepper_motor[i], 1);
          set_stepper_drive(&stepper_motor[i], 1);
          set_stepper_sleep(&stepper_motor[i], 0);

          set_stepper_stepsize(&stepper_motor[i], 1);

          set_stepper_rate(&stepper_motor[i], 60);
          set_stepper_velocity(&stepper_motor[i], 120);

          stepper_motor[i].homed = true;
          Serial.print("Cfg'd motor: ");
          Serial.println(i);
        }

        /*Reduce accel vlas of one*/
        set_stepper_rate(&stepper_motor[0], 30);
        set_stepper_velocity(&stepper_motor[0], 60);

        testing_timer = millis();
        test_states++;
      }

      break;

    case 1:
      if (millis() - testing_timer > 1000) {
        uint16_t motor_angles[] = {2700, 2700, 2700, 2700};

        apply_pose(motor_angles);

        Serial.print("Applied angles");
        testing_timer = millis();
        test_states++;
      }
      break;

    case 2:

      // if (millis() - testing_timer > 1000) {
      //   for (int i = 0; i < PCF_COUNT; i++) {
      //     motion_profiler(&stepper_motor[i]);
      //     Serial.print("Computed motion for ");
      //     Serial.println(i);
      //   }

      //   Serial.println("Time to grab the emergency STOP button...");
      //   Serial.println("Motor HOT in 1 sec.");
      //   testing_timer = millis();
      //   test_states++;
      // }
      test_states++;

      break;

    case 3:

      // if (millis() - testing_timer > 1000) {
      //   for (int i = 0; i < PCF_COUNT; i++) {
      //     set_stepper_block(&stepper_motor[i], 0);
      //   }

      //   Serial.println("Moving");
      //   testing_timer = millis();
      //   test_states++;
      // }
      test_states++;

      break;

    case 4:

      if (!(millis() % 300)) {
        // Serial.print("Current err: ");
        // Serial.println(stepper_motor[TESTAXES].position_setpoint -
        //                stepper_motor[TESTAXES].current_position);
        // Serial.print("Period time uS: ");
        // Serial.println(stepper_motor[TESTAXES].current_rot_velocity);

        // Serial.println(stepper_motor[1].current_rot_velocity);

        // Serial.print("SP0 ");
        // Serial.print(stepper_motor[0].on_sp);
        // Serial.print(" SP 1");
        // Serial.print(stepper_motor[1].on_sp);
        // Serial.print(" SP 2");
        // Serial.print(stepper_motor[2].on_sp);
        // Serial.print(" SP 3");
        // Serial.println(stepper_motor[3].on_sp);
      }

      if (stepper_motor[0].on_sp && stepper_motor[1].on_sp &&
          stepper_motor[2].on_sp && stepper_motor[3].on_sp) {
        for (int i = 0; i < PCF_COUNT; i++) {
          set_stepper_block(&stepper_motor[i], 1);
        }

        Serial.println("We've arrived.");
        Serial.println(stepper_motor[TESTAXES].current_rot_velocity);
        testing_timer = millis();
        test_states++;
      }

      break;

    case 5:
      if (millis() - testing_timer > 1000) {
        uint16_t motor_angles[] = {1800, 900, 900, 900};

        apply_pose(motor_angles);

        Serial.print("Applied angles 180");
        testing_timer = millis();
        test_states++;
      }

      break;

    case 6:
      // if (millis() - testing_timer > 1) {
      //   for (int i = 0; i < PCF_COUNT; i++) {
      //     motion_profiler(&stepper_motor[i]);
      //   }
      //   Serial.println("Computed motion.");
      //   Serial.println("Time to grab the emergency STOP button...");
      //   Serial.println("Motor HOT in 0 sec.");
      //   testing_timer = millis();
      //   test_states++;
      // }
      test_states++;

      break;

    case 7:

      // if (millis() - testing_timer > 1) {
      //   for (int i = 0; i < PCF_COUNT; i++) {
      //     set_stepper_block(&stepper_motor[i], 0);
      //   }
      //   Serial.println("Moving");
      //   testing_timer = millis();
      //   test_states++;
      // }

      test_states++;

      break;

    case 8:

      if (stepper_motor[0].on_sp && stepper_motor[1].on_sp &&
          stepper_motor[2].on_sp && stepper_motor[3].on_sp) {
        Serial.println("We've arrived.");
        // Serial.println(stepper_motor[TESTAXES].current_rot_velocity);
        testing_timer = millis();
        test_states++;

        // for (int i = 0; i < PCF_COUNT; i++) {
        //   set_stepper_block(&stepper_motor[i], 1);
        // }
      }

      if (!(millis() % 300)) {
        // Serial.print("Current err: ");
        // Serial.println(stepper_motor[TESTAXES].position_setpoint -
        //                stepper_motor[TESTAXES].current_position);
        // Serial.print("Period time uS: ");
        // Serial.println(stepper_motor[TESTAXES].current_rot_velocity);
        // Serial.println(stepper_motor[1].current_rot_velocity);

        // Serial.print("SP0");
        // Serial.print(stepper_motor[0].on_sp);
        // Serial.print(" SP1");
        // Serial.print(stepper_motor[1].on_sp);
        // Serial.print(" SP2");
        // Serial.print(stepper_motor[2].on_sp);
        // Serial.print(" SP3");
        // Serial.println(stepper_motor[3].on_sp);
      }

      break;

    case 9:

      Serial.println("Reloop");
      test_states = 1;
      break;

    case 10:
      apply_pose(motor_angles);
      test_states++;

      break;

    default:
      break;
  }

  /*constant polling*/
  stepper_control_loop();
}

/*Control the enable pin oft the stepper driver module*/
void set_stepper_drive(MotorConfig *unit_config, bool on_off_value) {
  unit_config->unit_pcf->write(NEN_PIN, !on_off_value);
}

/*Set to sleep and reset of unit*/
void set_stepper_sleep(MotorConfig *unit_config, bool on_off_value) {
  unit_config->unit_pcf->write(NSLP_PIN, !on_off_value);
  unit_config->unit_pcf->write(NRST_PIN, !on_off_value);
}

void set_stepper_block(MotorConfig *unit_config, bool on_off_value) {
  unit_config->block_control = on_off_value;
}

void set_stepper_stepsize(MotorConfig *unit_config, uint8_t stepsize) {
  if ((stepsize > 5)) {
    Serial.println("ERROR! Requested set step size out of bounds.");
    return;
  }
  /* bit manipulation baybay*/
  unit_config->stepper_mode = stepsize;
  unit_config->unit_pcf->write(M0_PIN, (stepsize & 1));
  unit_config->unit_pcf->write(M1_PIN, ((stepsize >> 1) & 1));
  unit_config->unit_pcf->write(M2_PIN, ((stepsize >> 2) & 1));
  // Serial.print("M0 is: ");
  // Serial.print((stepsize & 1));
  // Serial.print(" M1 is: ");
  // Serial.print(((stepsize >> 1) & 1));
  // Serial.print(" M2s is: ");
  // Serial.println(((stepsize >> 2) & 1));
}

void set_stepper_setpoint(MotorConfig *unit_config, int setpointpulse) {
  unit_config->position_setpoint = setpointpulse;

  /* Update whether on SP immidiately*/
  int pos_error =
      (unit_config->position_setpoint - unit_config->current_position);
  if (pos_error == 0) {
    unit_config->on_sp = true;
  } else {
    unit_config->on_sp = false;
  }
}

/*Set stepper acceleration profile, unit in RPM*/
void set_stepper_rate(MotorConfig *unit_config, int accelerate) {
  /*Beware for too high acceleration rates*/
  unit_config->accel_rate = RPM_TO_STEP_US_SQRD(accelerate) /
                            (float)(pow(2, unit_config->stepper_mode));
}

void set_stepper_velocity(MotorConfig *unit_config, int maxvelocity) {
  unit_config->max_rot_velocity = RPM_TO_STEP_US_FLT(maxvelocity) /
                                  (float)(pow(2, unit_config->stepper_mode));
}

/*Return current pos - setpoint error*/
int get_stepper_error(MotorConfig *unit_config) {
  return unit_config->position_setpoint - unit_config->current_position;
}

/*Compute one-time knowledge for driver. Movement accel/decel profile from a
 * startpoint up to end point.*/
void motion_profiler(MotorConfig *unit_config) {
  int steperror =
      unit_config->position_setpoint - unit_config->current_position;

  Serial.print("Error is: ");
  Serial.println(steperror);

  /*Set the direction pin of unit*/
  /*set direction of movement*/
  if (steperror > 0) {
    unit_config->unit_pcf->write(DIR_PIN, 1);
    Serial.println("Set dir to 1");
  } else {
    unit_config->unit_pcf->write(DIR_PIN, 0);
    Serial.println("Set dir to 0");
  }

  // steperror = abs(steperror);
  uint32_t abserror = (uint32_t)abs(steperror);

  /* Deceleration equation yapping below:
   *
   * Acceleration as we know it : a = (Vf - V0)/t_delta
   * We accelerate the frequency ; f = f_0+a*t
   * As we accelerate per period, t = prev pulse period time, f_0 is current
   * velocity in period time. And we want the next period time. So
   * 1/Tn+1 = (1/Tn) + a*Tn
   * T_(n+1) = 1/ (1/((1/Tn)+a*t))
   * T_(n+1) = 1/((1/Tn)+(Tn/Tacc))
   * Finally: T_(n+1) = (Tn*Tacc)/(Tn^2 + Tacc);
   * Above is for acceleration, for decelaration, its  (Tn*Tacc)/(Tn^2 - Tacc);
   * But beware, deceleration only works while Tn < sqrd(Tacc)
   * Which is ok mostly, just make sure to clip the period time between these.
   *
   * This implies,  sqrd(Tacc) - 1 is out minimum speed using this eq.
   */

  /* Error goes from x until 0 as motor moves.*/
  uint16_t ramp_steps = 0;
  uint16_t max_iter = 5000;
  unit_config->accel_sqrt = sqrt(unit_config->accel_rate);
  unit_config->min_rot_velocity = unit_config->accel_sqrt - 1;

  Serial.print("minT: ");
  Serial.println(unit_config->min_rot_velocity);

  float sim_period = unit_config->min_rot_velocity;

  /* 5000 is max iteration*/

  while ((sim_period > unit_config->max_rot_velocity) &&
         (ramp_steps < max_iter)) {
    ramp_steps++;
    sim_period = ((sim_period * unit_config->accel_rate) /
                  ((sim_period * sim_period) + unit_config->accel_rate));

    if (ramp_steps >= max_iter) {
      Serial.println("Reached max iter");
    }
  }

  Serial.print("Ramp is ");
  Serial.println(ramp_steps);

  /*Check if 2*ramp is < error */
  if (abserror > (uint32_t)(2 * ramp_steps)) {
    /*We will reach cruising speed*/
    unit_config->accel_dowto = abserror - ramp_steps;
    unit_config->decel_from = ramp_steps;
  } else {
    /*We wont. accel and decel vals can't be the same!!*/
    if (abserror % 2) {
      /*UNEVEN, we'll allow 1 step cruising for this to be even*/
      unit_config->accel_dowto = ((abserror + 1) / 2) + 1;
      unit_config->decel_from = ((abserror + 1) / 2) - 1;
    } else {
      unit_config->accel_dowto = (abserror / 2) + 1;
      unit_config->decel_from = abserror / 2;
    }
  }

  Serial.print("Will accelerate until: ");
  Serial.println(unit_config->accel_dowto);
  Serial.print("Will decelerate from: ");
  Serial.println(unit_config->decel_from);
}

void apply_pose(uint16_t wanted_pose[PCF_COUNT]) {
  /*Block all motors*/
  for (int i = 0; i < PCF_COUNT; i++) {
    set_stepper_block(&stepper_motor[i], 1);
  }

  for (int i = 0; i < PCF_COUNT; i++) {
    /*Get motor step movement range and map it to 360 degrees.*/
    uint16_t steprange = (200 << stepper_motor[i].stepper_mode);

    float mapping_factor = ((float)steprange / 3600.0f);

    /*((1600/3600)*wantedangle)-800*/
    int pose_step_setpoint = (int)((mapping_factor * (float)wanted_pose[i]) -
                                   (float)(steprange >> 1));
    set_stepper_setpoint(&stepper_motor[i], pose_step_setpoint);
    motion_profiler(&stepper_motor[i]);
  }

  /*Unblock all motors when done*/
  for (int i = 0; i < PCF_COUNT; i++) {
    set_stepper_block(&stepper_motor[i], 0);
  }
}

void stepper_control_loop() {
  static uint8_t mtr_index;

  int pos_error = (stepper_motor[mtr_index].position_setpoint -
                   stepper_motor[mtr_index].current_position);

  if (pos_error == 0) {
    // Serial.println("Is on SP.");
    stepper_motor[mtr_index].on_sp = true;
    stepper_motor[mtr_index].current_rot_velocity =
        stepper_motor[mtr_index].min_rot_velocity;
    mtr_index++;
    mtr_index %= (PCF_COUNT);
    return;
  } else {
    stepper_motor[mtr_index].on_sp = false;
  }

  if (stepper_motor[mtr_index].block_control) {
    // Serial.println("This unit is blocked, skipping");
    mtr_index++;
    mtr_index %= (PCF_COUNT);
    return;
  }

  if (!stepper_motor[mtr_index].homed) {
    // Serial.println("This unit isnt homed, skipping");
    mtr_index++;
    mtr_index %= (PCF_COUNT);
    return;
  }

  /* We check if there is a position error*/

  /* A step elapsed, time to set next time goal*/
  if (micros() >= stepper_motor[mtr_index].next_step_time) {
    /*capture execution time*/
    stepper_motor[mtr_index].set_exec_time = micros();
    /*increment pos*/
    if (pos_error > 0) {
      stepper_motor[mtr_index].current_position++;
    } else {
      stepper_motor[mtr_index].current_position--;
    }

    uint16_t abs_pos_err = abs(pos_error);

    /*Check if we should be accelerating or decelerating*/
    if (abs_pos_err >= stepper_motor[mtr_index].accel_dowto) {
      /*Increase velocity*/
      stepper_motor[mtr_index].current_rot_velocity =
          (stepper_motor[mtr_index].current_rot_velocity *
           stepper_motor[mtr_index].accel_rate) /
          (stepper_motor[mtr_index].accel_rate +
           (stepper_motor[mtr_index].current_rot_velocity *
            stepper_motor[mtr_index].current_rot_velocity));

    } else if (abs_pos_err <= stepper_motor[mtr_index].decel_from) {
      // /*decrease velocity*/
      float newper = (stepper_motor[mtr_index].current_rot_velocity *
                      stepper_motor[mtr_index].accel_rate) /
                     (stepper_motor[mtr_index].accel_rate -
                      (stepper_motor[mtr_index].current_rot_velocity *
                       stepper_motor[mtr_index].current_rot_velocity));

      /*Limit / clip*/
      if ((stepper_motor[mtr_index].min_rot_velocity + 1) > newper) {
        stepper_motor[mtr_index].current_rot_velocity = newper;
      }
    }

    /*Set when the step ends*/
    stepper_motor[mtr_index].next_step_time =
        stepper_motor[mtr_index].set_exec_time +
        (uint32_t)stepper_motor[mtr_index].current_rot_velocity;

    /*set toggle moment */
    stepper_motor[mtr_index].step_toggle_time =
        stepper_motor[mtr_index].set_exec_time +
        ((uint32_t)stepper_motor[mtr_index].current_rot_velocity / 2);
  }

  /*poll for step pin toggle*/
  if (micros() > stepper_motor[mtr_index].step_toggle_time) {
    digitalWrite(stepper_motor[mtr_index].step_pulse_pin, HIGH);
  } else {
    digitalWrite(stepper_motor[mtr_index].step_pulse_pin, LOW);
  }

  mtr_index++;
  mtr_index %= (PCF_COUNT);
}