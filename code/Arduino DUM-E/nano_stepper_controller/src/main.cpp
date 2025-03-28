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

#define SERIAL_BUF_SIZE 32 /*bytes to hold in buf*/

#define BASENUM 3
#define WRISTNUM 2
#define ENDLNUM 1
#define ENDRNUM 0

/* Is (60*10^6)/ (RPM * 200), returns uS per rotation */
/* Works RPM -> STEP uS and vice versa/*/
#define RPM_TO_STEP_US_FLT(x) (600000.0f / (x * 2.0f))
#define RPM_TO_STEP_US_SQRD(x) (600000000000.0 / (x * 2.0))

/* Maximum STEP pulse frequency: 1/((60/120)/200)=400Hz */
#define DEFAULT_MAX_ROT_VEL 60 /*120RPM*/
/* Acceleration is (45/60) (rev/min)/min, thus 0.75 min/s^2 */
#define DEFAULT_ROT_ACCEL 30 /*45RPM // 0.75 rev/min^2 */

#define DEFAULT_DRV8825_MODE 2 /*fullstep*/

// Gear ratios and stepcount
/* rotor gear tooth / driven gear tooths*/
#define motor_0_ratio (20.0f / 104.0f) /*end eff right*/
#define motor_1_ratio (20.0f / 104.0f) /*end eff left*/
#define motor_2_ratio (10.0f / 40.0f)  /*"wrist" rotator*/
#define motor_3_ratio (16.0f / 365.0f) /*Base motor */

long timer = 0;

/*used to detect when homing threshold is passed*/
uint16_t adc0_initial_value;
uint16_t adc1_initial_value;

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
  volatile bool is_homing;

  /*Motor ctrl info*/
  uint8_t pcf_addr;
  uint8_t step_pulse_pin;
  /* Associated PCF of this unit*/
  PCF8574 *unit_pcf;
  uint8_t stepper_mode; /*0-5 defines microstepping config of DRV8825*/
};

MotorConfig stepper_motor[PCF_COUNT];

uint8_t serial_buffer[SERIAL_BUF_SIZE];

/* pose value in decadegrees which are:
 * [end effector left] [end effector right] [wrist motor] [base motor]
 */
int16_t system_pose[PCF_COUNT] = {0, 0, 0, 0};

/*system state to put it in certain modes.
 */
enum system_state_t { UNINIT, INIT, HOMING } system_state;

/* Hardware enable or disable stepper motor, 1 is ON*/
void set_stepper_drive(MotorConfig *unit_config, bool onoffvalue);
/* Software block driving of a specific motor*/
void set_stepper_block(MotorConfig *unit_config, bool onoffvalue);
/* Eep. */
void set_stepper_sleep(MotorConfig *unit_config, bool on_off_value);

void start_stepper_homing();

/* 0 is fullstep, 1 is 1/2, 2 is 1/4, 3 is 1/8, 4 is 1/16, 5 is 1/32 */
void set_stepper_stepsize(MotorConfig *unit_config, uint8_t stepsize);
/* Position to move this axis to a certain setpoint position*/
void set_stepper_setpoint(MotorConfig *unit_config, int setpointpulse);
/* Change acceleration rate of a motor */
void set_stepper_rate(MotorConfig *unit_config, float accelrate);
/* Change max velocity of motor */
void set_stepper_velocity(MotorConfig *unit_config, float maxvelocity);
/*Return current pos - setpoint error*/
int get_stepper_error(MotorConfig *unit_config);

void motion_profiler(MotorConfig *unit_config);

/* A little dirty, but this overwrites angle values of end effector motors which
 * are kept in the system pose memory variable. Return true when all is good.*/
bool calculate_end_angles(int16_t angle, int16_t rotation);

/*Configures system speed scale for motors from current to setpoint positions,
 * also sets setpoints. Angles are in deca degrees. 180 => 1800*/
void apply_pose(int16_t wanted_pose[PCF_COUNT]);

/* Looks at how long a movement takes with default values of a device, and slow
 * it down until they all approximately match.*/
void synchromizer();

/* Polling function that controls pulses, calculates when the next pulse
 * should be when a step is done.
 */
void stepper_control_loop();

void homing_surveyor();

void serial_buffer_unloader();
void process_packet();

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
        RPM_TO_STEP_US_FLT(DEFAULT_MAX_ROT_VEL);

    /* pre calculated values at motion profiler and actual used values.*/
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
    stepper_motor[i].is_homing = false;
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

    /*Use functions*/
    set_stepper_stepsize(&stepper_motor[i], 2);
    set_stepper_drive(&stepper_motor[i], 1);
    set_stepper_block(&stepper_motor[i], 0);
    set_stepper_sleep(&stepper_motor[i], 0);

    /*Test if PCF is OK (reachable on I2C)*/
    if (!stepper_motor[i].unit_pcf->isConnected()) {
      Serial.println("Failed!, PCF unit can't be found on I2C bus.");
    } else {
      Serial.println("PCF init suc6");
    }
  }

  /* this one should be slow*/
  set_stepper_velocity(&stepper_motor[BASENUM], 4);
  set_stepper_rate(&stepper_motor[BASENUM], 5);

  /*Default overrides*/

  /*Have to test gearbox ratio/accelo*/
  /*If that is good, check homing*/
  system_state = INIT;
}

void loop() {
  /*Tracks button states and whether homing is done*/
  // homing_surveyor();
  /* Unload and handle packages.*/
  serial_buffer_unloader();
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

/*runs homing on all axes.*/
void start_stepper_homing() {
  if (system_state == UNINIT) {
    system_pose[BASENUM] = 3600;
    system_pose[WRISTNUM] = 3600;

    /*FINDOUT HOW MANY KILOOHMS POTS THEY ARE, I ASSUME 10k, thus,
    if pot is at center, its voltage will be 1/4, so 1,25V
    adc value will be 1.25/(5/1023) = ~256
    */

    /*TODO, CHECK IF THIS IS THE RIGHT DIRECTION and RIGHT POT FOR MOTOR*/

    adc0_initial_value = analogRead(ANALOG0);
    adc1_initial_value = analogRead(ANALOG1);

    if (adc0_initial_value > 256) {
      system_pose[ENDLNUM] = 900;
    } else if (adc0_initial_value < 256) {
      system_pose[ENDLNUM] = -900;
    }

    if (adc1_initial_value > 256) {
      system_pose[ENDRNUM] = 900;
    } else if (adc1_initial_value < 256) {
      system_pose[ENDRNUM] = -900;
    }

    /* enable them all and RUN */
    for (int i = 0; i < PCF_COUNT; i++) {
      set_stepper_block(&stepper_motor[i], 0);
      set_stepper_drive(&stepper_motor[i], 1);
      set_stepper_sleep(&stepper_motor[i], 0);
      stepper_motor[i].is_homing = true;
    }

    apply_pose(system_pose);

    system_state = HOMING;
  }
}

/* observe button states and finish homing when done*/
void homing_surveyor() {
  /*return if not homing */
  if (system_state != HOMING) {
    return;
  }

  /*TODO check if this IS for base, is button logic good?*/
  if (digitalRead(DIGITAL0) && stepper_motor[BASENUM].is_homing) {
    stepper_motor[BASENUM].is_homing = false;
    stepper_motor[BASENUM].current_position = 0;
    stepper_motor[BASENUM].position_setpoint = 0;
    stepper_motor[BASENUM].on_sp = true;
  }

  if (digitalRead(DIGITAL1) && stepper_motor[WRISTNUM].is_homing) {
    stepper_motor[WRISTNUM].is_homing = false;
    stepper_motor[WRISTNUM].current_position = 0;
    stepper_motor[WRISTNUM].position_setpoint = 0;
    stepper_motor[WRISTNUM].on_sp = true;
  }

  /* check adc 0*/
  if (stepper_motor[ENDLNUM].is_homing) {
    uint16_t adc0_now = analogRead(ANALOG0);
    if (adc0_initial_value > 256) {
      if (adc0_now < 256) {
        stepper_motor[ENDLNUM].is_homing = false;
        stepper_motor[ENDLNUM].current_position = 0;
        stepper_motor[ENDLNUM].position_setpoint = 0;
        stepper_motor[ENDLNUM].on_sp = true;
      }
    } else if (adc0_initial_value < 256) {
      if (adc0_now > 256) {
        stepper_motor[ENDLNUM].is_homing = false;
        stepper_motor[ENDLNUM].current_position = 0;
        stepper_motor[ENDLNUM].position_setpoint = 0;
        stepper_motor[ENDLNUM].on_sp = true;
      }
    }
  }

  /* check adc 1*/
  if (stepper_motor[ENDRNUM].is_homing) {
    uint16_t adc1_now = analogRead(ANALOG1);
    if (adc1_initial_value > 256) {
      if (adc1_now < 256) {
        stepper_motor[ENDRNUM].is_homing = false;
        stepper_motor[ENDRNUM].current_position = 0;
        stepper_motor[ENDRNUM].position_setpoint = 0;
        stepper_motor[ENDRNUM].on_sp = true;
      }
    } else if (adc0_initial_value < 256) {
      if (adc1_now > 256) {
        stepper_motor[ENDRNUM].is_homing = false;
        stepper_motor[ENDRNUM].current_position = 0;
        stepper_motor[ENDRNUM].position_setpoint = 0;
        stepper_motor[ENDRNUM].on_sp = true;
      }
    }
  }

  /* if all are done.*/
  if (!stepper_motor[BASENUM].is_homing && !stepper_motor[WRISTNUM].is_homing &&
      !stepper_motor[ENDLNUM].is_homing && !stepper_motor[ENDRNUM].is_homing) {
    /*update syspose for safe measure*/
    for (int i = 0; i < PCF_COUNT; i++) {
      system_pose[i] = 0;
    }
    apply_pose(system_pose);
    system_state = INIT;
  }
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
void set_stepper_rate(MotorConfig *unit_config, float accelerate) {
  /*Beware for too high acceleration rates*/
  unit_config->origin_accel_rate = RPM_TO_STEP_US_SQRD(accelerate);
}

/* In rpm to internal variables.*/
void set_stepper_velocity(MotorConfig *unit_config, float maxvelocity) {
  unit_config->origin_max_rot_velocity = RPM_TO_STEP_US_FLT(maxvelocity);
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

  /* Dont compute anything if there is nothing to do */
  if (abserror == 0) {
    unit_config->maneuver_time = 0;
    return;
  }

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

  /* apply speed scale acceleration*/
  unit_config->accel_rate = unit_config->origin_accel_rate *
                            (1.0 / unit_config->movement_speed_scale);
  /*and microstepping compensator*/
  unit_config->accel_rate *= (1.0 / (float)(1 << unit_config->stepper_mode));

  /*TODO TEST BUGFIX */
  /* APPLY GEAR RATIO, , potential bug, worse gear ratio means quicker
   * advancing of RPM between cycles in order to compensate. */
  unit_config->accel_rate *= (unit_config->shaft_gear_ratio);

  /*apply speed scale to velocity */
  unit_config->max_rot_velocity = unit_config->origin_max_rot_velocity *
                                  (1.0 / unit_config->movement_speed_scale);

  /*apply microstepp config comp. */
  unit_config->max_rot_velocity *=
      (1.0 / (float)(1 << unit_config->stepper_mode));

  /*  TODO TEST BUGFIX */
  /* Apply the gear ratio modifier TODO same issue as accel rate.*/
  unit_config->max_rot_velocity *= unit_config->shaft_gear_ratio;

  unit_config->accel_sqrt = sqrt(unit_config->accel_rate);
  unit_config->min_rot_velocity = unit_config->accel_sqrt - 1;

  /* 5000 is max iteration*/
  uint32_t ramp_steps = 0;
  uint16_t max_iter = 5000;

  float sim_period = unit_config->min_rot_velocity;

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

/* angles are in decadegrees!
 * angle can go from +90 downto -90
 * rotation can be total +720 downto -720 (4 rot range, arb. defined)*/
bool calculate_end_angles(int16_t angle, int16_t rotation) {
  /* rotation - motors turn the same dir | COMMON MODE.
   * angle -    motors turn against | DIFFERENTIAL.
   */

  /* Filter out value ranges*/
  if ((angle > 900) || (angle < -900)) {
    return false;
  }
  if ((rotation > 7200) || (angle < -7200)) {
    return false;
  }

  /* apply angle first (abspos) */
  int16_t effleft = angle;
  int16_t effright = -angle;

  /* apply rotation over it*/
  effleft += rotation;
  effright += rotation;

  /* throw it into the syspos*/
  system_pose[0] = effleft;
  system_pose[1] = effright;

  /* return true when all ok.*/
  return true;
}

/*Applies a pose in decadegrees and lets the motors loose.*/
void apply_pose(int16_t wanted_pose[PCF_COUNT]) {
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

    /*((1600/3600)*wantedangle). Thus, 0 degree position is setpoint 0 steps.
     * When degree is 90, step would be in the example above 400.
     */
    int pose_step_setpoint = (int)(mapping_factor * (float)wanted_pose[i]);
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

  /*run everything with scale 1*/
  for (int i = 0; i < PCF_COUNT; i++) {
    Serial.print("Re-set scale to 1.0 of U: ");
    Serial.println(i);
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

void serial_buffer_unloader() {
  static uint8_t bufferIndex;
  static uint32_t lastByteTime;

  while (Serial.available() > 0) {
    serial_buffer[bufferIndex] = Serial.read();
    lastByteTime = millis();  // Reset timeout timer

    bufferIndex++;

    /*Processing and overflow protection*/
    if (bufferIndex >= 14) {
      process_packet();
      bufferIndex = 0;
    } else if (bufferIndex >= BUFFER_LENGTH) {
      bufferIndex = 0;
    }
  }
  /* For timo */
  if (bufferIndex > 0 && (millis() - lastByteTime) > 100) {
    bufferIndex = 0;  // Discard partial data
  }
}

void process_packet() {
  // Validate markers (critical for alignment)
  if (serial_buffer[0] != 'C' || serial_buffer[2] != 'B' ||
      serial_buffer[5] != 'W' || serial_buffer[8] != 'R' ||
      serial_buffer[11] != 'A') {
    return;  // Invalid packet
  }

  int16_t data0 = (serial_buffer[3] << 8) | serial_buffer[4];
  int16_t data1 = (serial_buffer[6] << 8) | serial_buffer[7];
  int16_t data2 = (serial_buffer[9] << 8) | serial_buffer[10];
  int16_t data3 = (serial_buffer[12] << 8) | serial_buffer[13];

  uint8_t command = serial_buffer[1];

  switch (command) {
    case 0x01: /* Set pose command */

      /*Ignore this command while a homing procedure is taking place*/
      if (system_state == HOMING) {
        return;
      }

      /*  data0       data1      data2      data3
       *  [BASE ROT] [WRIST ROT] [END ROT] [END ANGLE]
       */

      calculate_end_angles(data3, data2);
      system_pose[2] = data1;
      system_pose[3] = data0;
      apply_pose(system_pose);
      break;

    case 0x02: /* UNIT/MOTOR CONFIGURE package. */
      /*  data0       data1      data2      data3
       *  [UNIT NUM] [RATE SET] [VELO SET] [BITMASK SLP/BLK/DRV]
       */

      /*check if unit number data is in range*/
      /* unit number 0-3 are individual ctrl, if 4 this applies to ALL units, */
      if ((data0 < 0) || data0 > PCF_COUNT) {
        /*Non existing unit is selected*/
        return;
      }

      /*rate and velo can't be negative*/
      if ((data1 < 0) | (data2 < 0) | (data3 < 0)) {
        return;
      }

      /*Apply the shits, ignore if applying this if value is 0.*/
      if (data1 != 0) {
        /* if data is 4, apply this change to all units.*/
        if (data0 == PCF_COUNT) {
          for (int i = 0; i < PCF_COUNT; i++) {
            set_stepper_rate(&stepper_motor[i], (float)data1);
          }
        } else {
          set_stepper_rate(&stepper_motor[data0], (float)data1);
        }
      }

      if (data2 != 0) {
        /* if data is 4, apply this change to all units.*/
        if (data0 == PCF_COUNT) {
          for (int i = 0; i < PCF_COUNT; i++) {
            set_stepper_velocity(&stepper_motor[i], (float)data2);
          }
        } else {
          set_stepper_velocity(&stepper_motor[data0], (float)data2);
        }
      }

      /*at no time, can this be 0.*/
      if (data3 != 0) {
        if (data0 == PCF_COUNT) {
          for (int i = 0; i < PCF_COUNT; i++) {
            set_stepper_drive(&stepper_motor[i], (((uint16_t)data3 >> 0) & 1));
            set_stepper_block(&stepper_motor[i], (((uint16_t)data3 >> 1) & 1));
            set_stepper_sleep(&stepper_motor[i], (((uint16_t)data3 >> 2) & 1));
          }
        } else {
          set_stepper_drive(&stepper_motor[data0],
                            (((uint16_t)data3 >> 0) & 1));
          set_stepper_block(&stepper_motor[data0],
                            (((uint16_t)data3 >> 1) & 1));
          set_stepper_sleep(&stepper_motor[data0],
                            (((uint16_t)data3 >> 2) & 1));
        }
      }
      break;

    case 0x03: /*start homing*/
      if (data0 == 10) {
        start_stepper_homing();
      }

      break;

    default:
      break;
  }
}
