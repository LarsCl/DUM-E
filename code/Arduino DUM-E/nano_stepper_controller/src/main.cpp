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
#define RPM_TO_STEP_US_FLT(x) (600000.0f / (x * 2.0f))
#define RPM_TO_STEP_US_SQRD(x) (600000000000.0 / (x * 2.0))

/* Maximum STEP pulse frequency: 1/((60/120)/200)=400Hz */
#define DEFAULT_MAX_ROT_VEL 120 /*120RPM*/
/* Acceleration is (45/60) (rev/min)/min, thus 0.75 min/s^2 */
#define DEFAULT_ROT_ACCEL 45 /*45RPM // 0.75 rev/min^2 */

#define DEFAULT_DRV8825_MODE 0 /*fullstep*/

// Gear ratios and stepcount
/* rotor gear tooth / driven gear tooths*/
#define motor_0_ratio (20.0f / 104.0f) /*end eff right*/
#define motor_1_ratio (20.0f / 104.0f) /*end eff left*/
#define motor_2_ratio (10.0f / 40.0f)  /*"wrist" rotator*/
#define motor_3_ratio (16.0f / 365.0f) /*Base motor */

long timer = 0;

const uint8_t step_pins[PCF_COUNT] = {STEP3_PIN, STEP2_PIN, STEP1_PIN,
                                      STEP0_PIN};
const float gears_map[PCF_COUNT] = {motor_0_ratio, motor_1_ratio, motor_2_ratio,
                                    motor_3_ratio};

/*globalinorion PCF objects*/
PCF8574 pcfs[PCF_COUNT] = {PCF8574(0x20, &Wire), PCF8574(0x21, &Wire),
                           PCF8574(0x22, &Wire), PCF8574(0x23, &Wire)};

// Stepper Configuration
struct MotorConfig {
  /*Memory of motor vars*/
  int position_setpoint;  // Target position in step pulses.
  int current_position;   // Current step position

  /* Original specifications, others are scaled during applying poses*/
  double origin_accel_rate; /*Accel/Decel rate modifier in uS per motor step*/
  float origin_max_rot_velocity; /* Holds minimum step period time in uS. */

  float shaft_gear_ratio; /* motorgear/robotgear ratio */

  /*System operation memory, scaled down for synchronisation*/
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
  uint32_t maneuver_time;

  volatile bool on_sp;         /*on setpoint flag*/
  volatile bool homed;         /*Is this axis homed?*/
  volatile bool block_control; /* Flag to software block driving this motor.
  true is blocking*/

  /*Motor ctrl info*/
  uint8_t pcf_addr;
  uint8_t step_pulse_pin;
  /* Associated PCF of this unit*/
  PCF8574 *unit_pcf;
  uint8_t stepper_mode; /*0-5 defines microstepping config of DRV8825*/
};

MotorConfig stepper_motor[PCF_COUNT];

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

void motion_profiler(MotorConfig *unit_config);
/*Configures system speed scale for motors from current to setpoint positions,
 * also sets setpoints. Angles are in deca degrees. 180 => 1800*/
void apply_pose(uint16_t wanted_pose[PCF_COUNT]);
/* Looks at how long a movement takes with default values of a device, and slow
 * it down until they all approximately match.*/
void synchromizer();

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

    stepper_motor[i].shaft_gear_ratio = gears_map[i];

    /* Original specifications, others are scaled during applying poses*/
    stepper_motor[i].origin_accel_rate = RPM_TO_STEP_US_SQRD(DEFAULT_ROT_ACCEL);
    stepper_motor[i].origin_max_rot_velocity =
        (float)RPM_TO_STEP_US_FLT(DEFAULT_MAX_ROT_VEL) /
        (float)(1 << DEFAULT_DRV8825_MODE);

    stepper_motor[i].accel_rate = stepper_motor[i].origin_accel_rate;
    stepper_motor[i].accel_sqrt = sqrt(stepper_motor[i].accel_rate);

    stepper_motor[i].max_rot_velocity =
        stepper_motor[i].origin_max_rot_velocity;

    stepper_motor[i].min_rot_velocity = stepper_motor[i].accel_sqrt - 1.0;
    stepper_motor[i].current_rot_velocity = stepper_motor[i].min_rot_velocity;

    stepper_motor[i].next_step_time = 0;
    stepper_motor[i].step_toggle_time = 0;
    stepper_motor[i].set_exec_time = 0;

    stepper_motor[i].movement_speed_scale = 1.0f;

    stepper_motor[i].accel_dowto = 0;
    stepper_motor[i].decel_from = 0;
    stepper_motor[i].maneuver_time = 0; /*in us*/

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
      Serial.println("Failed!, PCF unit can't be found on I2C bus.");
    } else {
      Serial.println("PCF init suc6");
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
          set_stepper_stepsize(&stepper_motor[i], 2);

          set_stepper_rate(&stepper_motor[i], 120);
          set_stepper_velocity(&stepper_motor[i], 120);

          stepper_motor[i].homed = true;
          Serial.print("Cfg'd motor: ");
          Serial.println(i);
        }

        testing_timer = millis();
        test_states++;
      }

      break;

    case 1:
      if (millis() - testing_timer > 1000) {
        uint16_t motor_angles[] = {180, 180, 180, 180};
        apply_pose(motor_angles);

        // Serial.print("Applied angles");
        testing_timer = millis();
        test_states++;
      }
      break;

    case 2:

      if (stepper_motor[0].on_sp && stepper_motor[1].on_sp &&
          stepper_motor[2].on_sp && stepper_motor[3].on_sp) {
        test_states++;
      }

      break;

    case 3: {
      uint16_t motor_angles[] = {180, 180, 180, 180};
      apply_pose(motor_angles);
      test_states++;
    } break;

    case 4:

      if (stepper_motor[0].on_sp && stepper_motor[1].on_sp &&
          stepper_motor[2].on_sp && stepper_motor[3].on_sp) {
        test_states++;
      }
      break;

    case 5:

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
}

void set_stepper_setpoint(MotorConfig *unit_config, int setpointpulse) {
  /*apply new setpoint*/
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
  unit_config->origin_accel_rate =
      RPM_TO_STEP_US_SQRD(accelerate) /
      (1.0f / (float)(1 << unit_config->stepper_mode));
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
  /* Error goes from x until 0 as motor moves.*/
  int steperror =
      unit_config->position_setpoint - unit_config->current_position;

  /*white text*/
  Serial.print("\033[0m Profiler \n\r");
  Serial.print("Error is: ");
  Serial.println(steperror);

  /*Set the direction pin of unit*/
  /*set direction of movement*/
  if (steperror > 0) {
    unit_config->unit_pcf->write(DIR_PIN, 1);
  } else {
    unit_config->unit_pcf->write(DIR_PIN, 0);
  }

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
   * Above is for acceleration, for decelaration, its  (Tn*Tacc)/(Tn^2 -
   * Tacc); But beware, deceleration only works while Tn < sqrd(Tacc) Which is
   * ok mostly, just make sure to clip the period time between these.
   *
   * This implies,  sqrd(Tacc) - 1 is out minimum speed using this eq.
   */

  /* apply speed scale*/
  unit_config->accel_rate = unit_config->origin_accel_rate *
                            (1.0 / unit_config->movement_speed_scale);
  /*and microstepping compensator*/
  unit_config->accel_rate *= (1.0 / (float)(1 << unit_config->stepper_mode));

  /*apply speed scale */
  unit_config->max_rot_velocity = unit_config->origin_max_rot_velocity *
                                  (1.0 / unit_config->movement_speed_scale);
  /*apply microstepp config comp. */
  unit_config->max_rot_velocity *=
      (1.0 / (float)(1 << unit_config->stepper_mode));

  unit_config->accel_sqrt = sqrt(unit_config->accel_rate);
  unit_config->min_rot_velocity = unit_config->accel_sqrt - 1;

  uint32_t ramp_steps = 0;
  uint16_t max_iter = 5000;

  // Serial.print("minT: ");
  // Serial.println(unit_config->min_rot_velocity);

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

  /*Check if 2*ramp is < error */
  if (abserror > (2 * ramp_steps)) {
    /*We will reach cruising speed*/
    unit_config->accel_dowto = abserror - ramp_steps;
    unit_config->decel_from = ramp_steps;

    /*calculate how long this move will take. (Tacc/Tvmax)+err*Tvmax*/
    unit_config->maneuver_time =
        (uint32_t)((unit_config->accel_rate / unit_config->max_rot_velocity) +
                   (float)abserror * unit_config->max_rot_velocity);

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
    /*calculate this movement 2*sqrt(step*Tacc)*/
    unit_config->maneuver_time =
        (uint32_t)(2.0 * sqrt(abserror * unit_config->accel_rate));
  }

  Serial.print("Time is: ");
  Serial.println(unit_config->maneuver_time);
}

/*Applies a pose and lets the motors loose.*/
void apply_pose(uint16_t wanted_pose[PCF_COUNT]) {
  /*Block all motors*/
  for (int i = 0; i < PCF_COUNT; i++) {
    set_stepper_block(&stepper_motor[i], 1);
  }

  /*apply setpoint from converted degrees*/
  for (int i = 0; i < PCF_COUNT; i++) {
    /*Get motor step movement range and map it to 360 degrees.*/
    float steprange = (float)(200 << stepper_motor[i].stepper_mode) /
                      stepper_motor[i].shaft_gear_ratio;

    /* Dont forget input is in deca DEGREES*/
    float mapping_factor = ((float)steprange / 3600.0f);

    /*((1600/3600)*wantedangle)-800*/
    int pose_step_setpoint = (int)((mapping_factor * (float)wanted_pose[i]) -
                                   (float)(steprange / 2));
    set_stepper_setpoint(&stepper_motor[i], pose_step_setpoint);
  }

  /*Run synchromizer to adjust all movements to end at the same time*/
  synchromizer();

  /*Unblock all motors when done*/
  for (int i = 0; i < PCF_COUNT; i++) {
    set_stepper_block(&stepper_motor[i], 0);
  }
}

void synchromizer() {
  /*Synchromizer aims to utilize motion profiler to:
   * Calculate how long movement from current pos to setpoint will take
   * find longest process
   * calculate time ratio of all movement to the longest taking one
   * adjust movement speed scale
   * re-run profiler to calculate the new slowed down movement
   * */

  /*start green texts*/
  Serial.println("\033[1;32m Synchromizer DEBUG START");

  // Serial.println("\033[1;32m bold PRE-SETUP TEST \033[0m COLORS\n");
  // Serial.println("\033[1;32m Green");
  // Serial.println("\033[31;1;4m RedUnder!\033[0m");

  /*run everything with scale 1*/

  for (int i = 0; i < PCF_COUNT; i++) {
    Serial.println("Re-set scale to 100%");
    stepper_motor[i].movement_speed_scale = 1.0f;
    motion_profiler(&stepper_motor[i]);
    Serial.println("\033[1;32m");
  }

  uint32_t longest_time = 0;

  for (int i = 0; i < PCF_COUNT; i++) {
    if (stepper_motor[i].maneuver_time > longest_time) {
      Serial.print("Unit ");
      Serial.print(i);
      Serial.print(" takes about ");
      longest_time = stepper_motor[i].maneuver_time;
      Serial.print(stepper_motor[i].maneuver_time);
      Serial.println(" uS");
    }
  }

  /*then, run everything as a fraction of the longest time. */
  Serial.print("Longest time found: ");
  Serial.println(longest_time);

  for (int i = 0; i < PCF_COUNT; i++) {
    stepper_motor[i].movement_speed_scale =
        ((float)stepper_motor[i].maneuver_time / (float)longest_time);
    motion_profiler(&stepper_motor[i]);
    Serial.print("\033[1;32m");
    Serial.print(" Unit ");
    Serial.print(i);
    Serial.print(" scaler val is ");
    Serial.print(stepper_motor[i].movement_speed_scale);
  }

  /*End green texts*/
  Serial.println("\033[0m");
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

  /* A step elapsed, time to set next time goal*/
  if (micros() >= stepper_motor[mtr_index].next_step_time) {
    /*capture execution time*/
    stepper_motor[mtr_index].set_exec_time = micros();
    /*increment pos*/
    noInterrupts();
    if (pos_error > 0) {
      stepper_motor[mtr_index].current_position++;
    } else {
      stepper_motor[mtr_index].current_position--;
    }
    interrupts();

    uint32_t abs_pos_err = abs(pos_error);

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