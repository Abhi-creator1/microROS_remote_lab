/*
  ASRS Robot Teensy 4.1 micro-ROS firmware (Jan 2026)
  UPDATED v11.1  (v11 + encoder glitch rejection + cleanup)

  v11.1 includes:
  ✅ Reject UNSTABLE encoder samples (prevents the -37° -> +112° teleport)
     - If readAbsDegStable() returns false => DO NOT update that joint this cycle.
  ✅ CAPHOME uses stable sampling (retries) so home itself is not “poisoned”.
  ✅ Fix bug: removed the repeated "any_need_move=true" spam.
  ✅ Simple PWM gripper servo on pin 10
     - Topic: /teensy_jaw_target (std_msgs/Float64)
     - 0.0 = CLOSE, 1.0 = OPEN (tunable mapping)
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <SCServo.h>
#include <Servo.h>   // NEW: for PWM gripper servo

// micro-ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/bool.h>

// ---------------------- QUICK FIX SWITCHES ----------------------
static const bool FORCE_BASE_DIR_INVERT = true;   // flips base stepper DIR output only
static const bool FORCE_BASE_ENC_INVERT = false;  // flips base encoder angle only

// ---------------------- helpers ----------------------
static inline float deg2rad(float d){ return d * 0.01745329252f; }
static inline float rad2deg(float r){ return r * 57.295779513f; }

static inline float clampf(float x, float lo, float hi){
  if(x < lo) return lo;
  if(x > hi) return hi;
  return x;
}
static inline float clampSym(float x, float lim){
  return clampf(x, -lim, +lim);
}

static float wrap360(float x){
  while(x >= 360.0f) x -= 360.0f;
  while(x <  0.0f)   x += 360.0f;
  return x;
}

// shortest signed delta (deg) from current -> target, in [-180,+180]
static float angleDiffDeg(float target_deg, float current_deg){
  float t = deg2rad(target_deg);
  float c = deg2rad(current_deg);
  float d = atan2f(sinf(t - c), cosf(t - c));
  return rad2deg(d);
}

static float meanAngleDeg(float a_deg, float b_deg){
  float a = deg2rad(a_deg);
  float b = deg2rad(b_deg);
  float sx = cosf(a) + cosf(b);
  float sy = sinf(a) + sinf(b);
  float ang = atan2f(sy, sx);
  if(ang < 0) ang += TWO_PI;
  return wrap360(rad2deg(ang));
}

static inline uint16_t adcRead12(int pin){
  (void)analogRead(pin);
  uint16_t raw = (uint16_t)analogRead(pin);
  if(raw >= 4095) raw = 4094;
  return raw;
}

// ---------------------- joints ----------------------
enum : uint8_t { ARM1 = 0, BASE = 1, ARM2 = 2 };
static const uint8_t NS = 3;

// ROS joint indices:
enum : uint8_t { RJ_CONT=0, RJ_REV1=1, RJ_REV2=2, RJ_REV3=3 };

// ---------------------- pins ----------------------
static const bool SWAP_BASE_ARM1_MOTORS   = false;
static const bool SWAP_BASE_ARM1_ENCODERS = false;

static const int STEP_PIN[NS] = {
  SWAP_BASE_ARM1_MOTORS ? 6 : 3,
  SWAP_BASE_ARM1_MOTORS ? 3 : 6,
  9
};
static const int DIR_PIN[NS]  = {
  SWAP_BASE_ARM1_MOTORS ? 5 : 2,
  SWAP_BASE_ARM1_MOTORS ? 2 : 5,
  8
};

static const int ENC_PIN[NS]  = {
  SWAP_BASE_ARM1_ENCODERS ? A2 : A0,
  SWAP_BASE_ARM1_ENCODERS ? A0 : A2,
  A1
};

// -------- OPTIONAL DRIVER ENABLE (ONLY IF WIRED) --------
static const int EN_PIN[NS] = { -1, -1, -1 };   // {ARM1_EN, BASE_EN, ARM2_EN}
static const bool EN_ACTIVE_LOW = true;
static const bool ALLOW_IDLE_DRIVER_DISABLE = false;
static const uint32_t IDLE_DISABLE_MS = 1200;

static inline void driverEnable(uint8_t j, bool en){
  if(EN_PIN[j] < 0) return;
  pinMode(EN_PIN[j], OUTPUT);
  if(EN_ACTIVE_LOW){
    digitalWrite(EN_PIN[j], en ? LOW : HIGH);
  }else{
    digitalWrite(EN_PIN[j], en ? HIGH : LOW);
  }
}
static inline void enableAllDrivers(bool en){
  for(uint8_t j=0;j<NS;j++) driverEnable(j, en);
}

// ---------------------- scaling ----------------------
static const int BASE_MOTOR_TEETH = 20;
static const int BASE_JOINT_TEETH = 80;

static float BASE_GEAR_MOTOR_PER_JOINT = (float)BASE_JOINT_TEETH / (float)BASE_MOTOR_TEETH;

static float SPR_ARM1_JOINT = 64000.0f;
static float SPR_ARM2_JOINT = 8000.0f;
static float SPR_BASE_MOTOR = 1600.0f;

// Hardware inversions (pins only)
static bool DIR_INVERT[NS]  = { false, false, false };
static bool ENC_INVERT[NS]  = { false, false, false };
static bool STEP_INVERT[NS] = { false, false, false };

// ROS sign inversions
static bool ROS_CMD_INV[NS] = { false, false, false }; // arm1, base, arm2
static bool ROS_FB_INV[NS]  = { false, false, true };

static inline float ROS_CMD_SIGN(uint8_t j){ return ROS_CMD_INV[j] ? -1.0f : +1.0f; }
static inline float ROS_FB_SIGN(uint8_t j){ return ROS_FB_INV[j]  ? -1.0f : +1.0f; }

// Base joint limits
static const float BASE_MIN_ROS_RAD   = deg2rad(-170.0f);
static const float BASE_MAX_ROS_RAD   = deg2rad(+170.0f);
static const float BASE_MIN_JOINT_DEG = -170.0f;
static const float BASE_MAX_JOINT_DEG = +170.0f;

static inline float baseMotorMinDeg(){ return BASE_MIN_JOINT_DEG * BASE_GEAR_MOTOR_PER_JOINT; }
static inline float baseMotorMaxDeg(){ return BASE_MAX_JOINT_DEG * BASE_GEAR_MOTOR_PER_JOINT; }

static inline float stepsPerDeg(uint8_t j){
  if(j==ARM1) return SPR_ARM1_JOINT / 360.0f;
  if(j==BASE) return (SPR_BASE_MOTOR * BASE_GEAR_MOTOR_PER_JOINT) / 360.0f;
  return SPR_ARM2_JOINT / 360.0f;
}

static const uint16_t MIN_PULSE_US = 5;

// These are stepper caps (steps/s) for AccelStepper:
static float MAX_SPEED_N[NS] = { 5000, 1800, 1800 };
static float ACCEL_N[NS]     = {  600,  100,  100 };

AccelStepper stp[NS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[0], DIR_PIN[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[1], DIR_PIN[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[2], DIR_PIN[2])
};

static void forcePinsSafeLow(){
  for(uint8_t j=0;j<NS;j++){
    pinMode(STEP_PIN[j], OUTPUT);
    pinMode(DIR_PIN[j], OUTPUT);
    digitalWrite(STEP_PIN[j], LOW);
    digitalWrite(DIR_PIN[j], LOW);
  }
}

static void applyStepperConfig(){
  for(uint8_t j=0;j<NS;j++){
    stp[j].setMinPulseWidth(MIN_PULSE_US);
    stp[j].setPinsInverted(DIR_INVERT[j], STEP_INVERT[j], false);
    stp[j].setMaxSpeed(MAX_SPEED_N[j]);
    stp[j].setAcceleration(ACCEL_N[j]); // not used by runSpeed(), kept harmless
    stp[j].setSpeed(0.0f);
  }
}

// ---------------------- wrist (SCS15) ----------------------
SCSCL sc;
static const uint8_t WRIST_ID   = 1;
static const long    WRIST_BAUD = 1000000;

static const int   WRIST_POS_MIN = 0;
static const int   WRIST_POS_MAX = 1023;
static const float WRIST_DEG_MAX = 220.0f;

static const float REV3_MIN_ROS_RAD = -2.600f;
static const float REV3_MAX_ROS_RAD =  0.000f;

static const bool  WRIST_INV_TICK = false;
static const bool  WRIST_ROS_INV  = false;

static const uint16_t WRIST_SPEED = 400;
static const uint8_t  WRIST_ACCEL = 4;

static uint16_t wrist_home_tick = 750;
static uint16_t wrist_present_tick = 750;
static float    wrist_present_rel_rad = 0.0f;

static int wrist_last_tgt_tick = -1;
static int wrist_last_dtick = 0;
static int wrist_last_lo = 0;
static int wrist_last_hi = 0;

static inline uint16_t wristClampAbsTick(int t){
  if(t < WRIST_POS_MIN) t = WRIST_POS_MIN;
  if(t > WRIST_POS_MAX) t = WRIST_POS_MAX;
  return (uint16_t)t;
}

static int wristReadPosRetry(int tries=5){
  for(int i=0;i<tries;i++){
    int p = sc.ReadPos(WRIST_ID);
    if(p >= 0) return p;
    delay(4);
  }
  return -1;
}

static inline float rev3RosToServoRad(float ros_rad){
  return WRIST_ROS_INV ? -ros_rad : ros_rad;
}
static inline float rev3ServoToRosRad(float servo_rad){
  return WRIST_ROS_INV ? -servo_rad : servo_rad;
}

static inline int wristRadToDeltaTick(float servo_rad){
  float deg = rad2deg(servo_rad);
  return (int)lroundf(deg * ((float)WRIST_POS_MAX / WRIST_DEG_MAX));
}

static inline float wristDeltaTickToRad(int dtick){
  float deg = ((float)dtick) * (WRIST_DEG_MAX / (float)WRIST_POS_MAX);
  return deg2rad(deg);
}

static inline void wristGetServoDeltaLimits(int &min_dtick, int &max_dtick){
  float a = rev3RosToServoRad(REV3_MIN_ROS_RAD);
  float b = rev3RosToServoRad(REV3_MAX_ROS_RAD);
  float servo_min = (a < b) ? a : b;
  float servo_max = (a < b) ? b : a;

  min_dtick = wristRadToDeltaTick(servo_min);
  max_dtick = wristRadToDeltaTick(servo_max);
  if(min_dtick > max_dtick){
    int tmp = min_dtick; min_dtick = max_dtick; max_dtick = tmp;
  }
}

static inline void wristGetEffectiveDeltaWindow(int &lo, int &hi){
  int ros_lo, ros_hi;
  wristGetServoDeltaLimits(ros_lo, ros_hi);

  int abs_lo = -(int)wrist_home_tick;
  int abs_hi = (int)WRIST_POS_MAX - (int)wrist_home_tick;

  lo = (ros_lo > abs_lo) ? ros_lo : abs_lo;
  hi = (ros_hi < abs_hi) ? ros_hi : abs_hi;

  wrist_last_lo = lo;
  wrist_last_hi = hi;
}

static bool wristPollPresent(){
  int p = wristReadPosRetry(3);
  if(p < 0) return false;

  wrist_present_tick = (uint16_t)p;
  int dtick = (int)wrist_present_tick - (int)wrist_home_tick;
  if(WRIST_INV_TICK) dtick = -dtick;

  float servo_rad = wristDeltaTickToRad(dtick);
  float ros_rad   = rev3ServoToRosRad(servo_rad);

  ros_rad = clampf(ros_rad, REV3_MIN_ROS_RAD, REV3_MAX_ROS_RAD);
  wrist_present_rel_rad = ros_rad;
  return true;
}

static bool wristSendTargetRelRad(float ros_rad){
  ros_rad = clampf(ros_rad, REV3_MIN_ROS_RAD, REV3_MAX_ROS_RAD);

  float servo_rad = rev3RosToServoRad(ros_rad);

  int lo, hi;
  wristGetEffectiveDeltaWindow(lo, hi);

  int dtick = wristRadToDeltaTick(servo_rad);
  if(dtick < lo) dtick = lo;
  if(dtick > hi) dtick = hi;
  if(WRIST_INV_TICK) dtick = -dtick;

  int tgt = (int)wrist_home_tick + dtick;
  uint16_t tgt_tick = wristClampAbsTick(tgt);

  wrist_last_tgt_tick = (int)tgt_tick;
  wrist_last_dtick = dtick;

  sc.WritePos(WRIST_ID, (int)tgt_tick, WRIST_ACCEL, WRIST_SPEED);
  return true;
}

// ---------------------- EEPROM settings ----------------------
struct Settings {
  uint32_t magic;

  uint8_t  home_set;
  float    home_abs_deg[NS];

  uint8_t  enc_inv[NS], dir_inv[NS], step_inv[NS];
  float spr_arm1_joint, spr_base_motor, spr_arm2_joint;
  float base_gear;

  uint8_t  ros_inv[NS];
  uint8_t  ros_fb_inv[NS];

  uint8_t  wrist_home_valid;
  uint16_t wrist_home_tick;

  uint8_t  swap_motors_flag;
  uint8_t  swap_encoders_flag;
};

static Settings S;
static const uint32_t MAGIC = 0x41535296u;

static void saveSettings(){
  S.magic = MAGIC;

  S.base_gear = BASE_GEAR_MOTOR_PER_JOINT;
  S.spr_arm1_joint = SPR_ARM1_JOINT;
  S.spr_base_motor = SPR_BASE_MOTOR;
  S.spr_arm2_joint = SPR_ARM2_JOINT;

  for(uint8_t i=0;i<NS;i++){
    S.enc_inv[i]   = ENC_INVERT[i] ? 1 : 0;
    S.dir_inv[i]   = DIR_INVERT[i] ? 1 : 0;
    S.step_inv[i]  = STEP_INVERT[i]? 1 : 0;
    S.ros_inv[i]   = ROS_CMD_INV[i]? 1 : 0;
    S.ros_fb_inv[i]= ROS_FB_INV[i] ? 1 : 0;
  }

  S.wrist_home_valid = 1;
  S.wrist_home_tick  = wrist_home_tick;

  S.swap_motors_flag   = SWAP_BASE_ARM1_MOTORS ? 1 : 0;
  S.swap_encoders_flag = SWAP_BASE_ARM1_ENCODERS ? 1 : 0;

  EEPROM.put(0, S);
}

static void factoryResetEEPROM(){
  Settings Z;
  memset(&Z, 0, sizeof(Z));
  Z.magic = 0;
  EEPROM.put(0, Z);

  memset(&S, 0, sizeof(S));
  S.magic = MAGIC;

  BASE_GEAR_MOTOR_PER_JOINT = (float)BASE_JOINT_TEETH / (float)BASE_MOTOR_TEETH;
  SPR_ARM1_JOINT = 64000.0f;
  SPR_ARM2_JOINT = 8000.0f;
  SPR_BASE_MOTOR = 1600.0f;

  for(uint8_t i=0;i<NS;i++){
    ENC_INVERT[i] = DIR_INVERT[i] = STEP_INVERT[i] = false;
    ROS_CMD_INV[i] = ROS_FB_INV[i] = false;
  }

  int p = wristReadPosRetry(8);
  wrist_home_tick = (p >= 0) ? (uint16_t)wristClampAbsTick(p) : 750;

  S.home_set = 0;
  S.wrist_home_valid = 1;
  S.wrist_home_tick = wrist_home_tick;
  saveSettings();
}

static void loadSettings(){
  EEPROM.get(0, S);
  if(S.magic != MAGIC){
    memset(&S, 0, sizeof(S));
    S.magic = MAGIC;
    return;
  }

  for(uint8_t i=0;i<NS;i++){
    ENC_INVERT[i]  = (S.enc_inv[i]!=0);
    DIR_INVERT[i]  = (S.dir_inv[i]!=0);
    STEP_INVERT[i] = (S.step_inv[i]!=0);
    ROS_CMD_INV[i] = (S.ros_inv[i]!=0);
    ROS_FB_INV[i]  = (S.ros_fb_inv[i]!=0);
  }

  if(isfinite(S.base_gear) && S.base_gear > 0.2f && S.base_gear < 50.0f) BASE_GEAR_MOTOR_PER_JOINT = S.base_gear;
  if(isfinite(S.spr_arm1_joint) && S.spr_arm1_joint > 100) SPR_ARM1_JOINT = S.spr_arm1_joint;
  if(isfinite(S.spr_base_motor) && S.spr_base_motor > 50)  SPR_BASE_MOTOR = S.spr_base_motor;
  if(isfinite(S.spr_arm2_joint) && S.spr_arm2_joint > 100) SPR_ARM2_JOINT = S.spr_arm2_joint;

  if(S.wrist_home_valid == 1 && S.wrist_home_tick <= WRIST_POS_MAX){
    wrist_home_tick = S.wrist_home_tick;
  }
}

// ---------------------- encoder feedback ----------------------
static const uint16_t FB_NAVG_ARM  = 3;
static const uint16_t FB_NAVG_BASE = 3;

static const float FB_ALPHA_ARM  = 0.20f;
static const float FB_ALPHA_BASE = 0.25f;

static const float STABLE_JUMP_MAX_DEG[NS] = { 6.0f, 12.0f, 20.0f };

static float readAbsDeg(uint8_t j, uint16_t n=3){
  float sx=0, sy=0;
  const int pin = ENC_PIN[j];

  for(uint16_t i=0;i<n;i++){
    uint16_t raw = adcRead12(pin);
    float a = (raw/4095.0f) * (float)TWO_PI;
    sx += cosf(a);
    sy += sinf(a);
  }
  float ang = atan2f(sy,sx);
  if(ang < 0) ang += (float)TWO_PI;

  float deg = wrap360(rad2deg(ang));
  if(ENC_INVERT[j]) deg = wrap360(360.0f - deg);
  return deg;
}

static bool readAbsDegStable(uint8_t j, uint16_t navg, float max_jump_deg, float &out_deg){
  float a = readAbsDeg(j, navg);
  delayMicroseconds(250);
  float b = readAbsDeg(j, navg);
  float d = fabsf(angleDiffDeg(b, a));
  if(d > max_jump_deg){
    out_deg = b;
    return false;
  }
  out_deg = meanAngleDeg(a, b);
  return true;
}

// physical joint deg relative to HOME (filtered for control)
static float fb_joint_deg[NS] = {0,0,0};

// ALSO keep "raw rel" for watchdog movement detection (unfiltered)
static float fb_joint_deg_rawrel[NS] = {0,0,0};

// Base unwrapping state (motor shaft)
static bool  base_track_init = false;
static float base_prev_raw_deg = 0.0f;
static float base_motor_unwrapped_deg = 0.0f;

// ---------------------- SAFETY / FAULT ----------------------
static volatile bool fault_latched = false;
static volatile int  fault_code = 0;

static void stopAllSpeeds(){
  for(uint8_t j=0;j<NS;j++){
    stp[j].setSpeed(0.0f);
  }
}

static void faultStop(int code){
  fault_latched = true;
  fault_code = code;
  stopAllSpeeds();
}

// CAPHOME gating
static bool homed_this_boot = false;
static bool have_cmd_since_home = false;

// ---------------------- motion tracking for executed status ----------------------
static bool motion_in_progress = false;

// ---------------------- base unwrap ----------------------
static inline float baseMotorRelShortFromHome(float raw_deg){
  return -angleDiffDeg(S.home_abs_deg[BASE], raw_deg);
}

static float baseUpdateMotorUnwrapped(float raw_deg){
  if(!base_track_init){
    base_prev_raw_deg = raw_deg;
    base_motor_unwrapped_deg = baseMotorRelShortFromHome(raw_deg);
    base_track_init = true;
    return base_motor_unwrapped_deg;
  }

  float d = angleDiffDeg(raw_deg, base_prev_raw_deg);
  if(fabsf(d) < 0.08f) d = 0.0f;

  base_motor_unwrapped_deg += d;
  base_prev_raw_deg = raw_deg;
  return base_motor_unwrapped_deg;
}

static inline float baseMotorUnwrappedToJointDeg(float motor_deg){
  return motor_deg / BASE_GEAR_MOTOR_PER_JOINT;
}

// v11.1: REJECT unstable samples
static bool encoderJointDegPhysicalShort_nonBase(uint8_t j, float &out_deg){
  if(!S.home_set) return false;

  float raw = 0.0f;
  bool ok = readAbsDegStable(j, FB_NAVG_ARM, STABLE_JUMP_MAX_DEG[j], raw);
  if(!ok) return false;

  float rel_deg = -angleDiffDeg(S.home_abs_deg[j], raw);
  out_deg = rel_deg;
  return true;
}

// --- encoder-alive tracking ---
static float enc_last_deg[NS] = {0,0,0};
static uint32_t enc_last_change_ms[NS] = {0,0,0};

static const float  ENC_MOVE_EPS_DEG = 0.50f;
static const uint32_t ENC_STUCK_MS   = 4000;
static const uint32_t MOVE_GRACE_MS  = 600;

static const float WD_V_THRESH_DEG_S = 6.0f;
static const float WD_ERR_THRESH_DEG = 2.0f;

static uint32_t wd_arm_ms[NS] = {0,0,0};

// Throttle heavy encoder updates
static const uint32_t ENC_UPDATE_MS = 25;
static uint32_t enc_last_update_ms = 0;

static void updateFeedbackFromEncoders_Throttled(){
  if(!homed_this_boot || !S.home_set) return;

  uint32_t now_ms = millis();
  if((uint32_t)(now_ms - enc_last_update_ms) < ENC_UPDATE_MS) return;
  enc_last_update_ms = now_ms;

  // BASE
  {
    float raw_abs = 0.0f;
    bool ok = readAbsDegStable(BASE, FB_NAVG_BASE, STABLE_JUMP_MAX_DEG[BASE], raw_abs);
    if(ok){
      float motor_deg = baseUpdateMotorUnwrapped(raw_abs);
      float joint_deg = baseMotorUnwrappedToJointDeg(motor_deg);

      fb_joint_deg_rawrel[BASE] = joint_deg;

      float mlo = baseMotorMinDeg() - 15.0f;
      float mhi = baseMotorMaxDeg() + 15.0f;
      if(motor_deg < mlo || motor_deg > mhi) faultStop(101);

      if(joint_deg < (BASE_MIN_JOINT_DEG - 8.0f) || joint_deg > (BASE_MAX_JOINT_DEG + 8.0f)) faultStop(102);

      float prev = fb_joint_deg[BASE];
      fb_joint_deg[BASE] = prev + FB_ALPHA_BASE * (joint_deg - prev);

      if(fabsf(joint_deg - enc_last_deg[BASE]) > ENC_MOVE_EPS_DEG){
        enc_last_deg[BASE] = joint_deg;
        enc_last_change_ms[BASE] = now_ms;
      }
    }
  }

  // ARM1
  {
    float rawrel_deg = 0;
    if(encoderJointDegPhysicalShort_nonBase(ARM1, rawrel_deg)){
      fb_joint_deg_rawrel[ARM1] = rawrel_deg;

      float prev = fb_joint_deg[ARM1];
      float d = angleDiffDeg(rawrel_deg, prev);
      float filt = prev + FB_ALPHA_ARM * d;
      while(filt >  180.0f) filt -= 360.0f;
      while(filt < -180.0f) filt += 360.0f;
      fb_joint_deg[ARM1] = filt;

      if(fabsf(angleDiffDeg(rawrel_deg, enc_last_deg[ARM1])) > ENC_MOVE_EPS_DEG){
        enc_last_deg[ARM1] = rawrel_deg;
        enc_last_change_ms[ARM1] = now_ms;
      }
    }
  }

  // ARM2
  {
    float rawrel_deg = 0;
    if(encoderJointDegPhysicalShort_nonBase(ARM2, rawrel_deg)){
      fb_joint_deg_rawrel[ARM2] = rawrel_deg;

      float prev = fb_joint_deg[ARM2];
      float d = angleDiffDeg(rawrel_deg, prev);
      float filt = prev + FB_ALPHA_ARM * d;
      while(filt >  180.0f) filt -= 360.0f;
      while(filt < -180.0f) filt += 360.0f;
      fb_joint_deg[ARM2] = filt;

      if(fabsf(angleDiffDeg(rawrel_deg, enc_last_deg[ARM2])) > ENC_MOVE_EPS_DEG){
        enc_last_deg[ARM2] = rawrel_deg;
        enc_last_change_ms[ARM2] = now_ms;
      }
    }
  }
}

// PHYS(rad)->ROS(rad) uses only ROS_FB_INV
static inline float physRadToRosRadRaw(uint8_t axis, float phys_rad){
  float s = ROS_FB_SIGN(axis);
  if(s == 0.0f) s = 1.0f;
  return phys_rad / s;
}

// ---------------------- CLOSED-LOOP SPEED CONTROL ----------------------
static float tgt_ros_rad[NS] = {0.0f, 0.0f, 0.0f};
static float v_cur_deg_s[NS] = {0.0f, 0.0f, 0.0f};

// Hysteresis
static bool  at_target[NS]       = { true, true, true };

static float STOP_TOL_DEG[NS]    = { 1.8f, 2.0f, 1.0f };
static float START_TOL_DEG[NS]   = { 2.6f, 3.0f, 1.6f };

static const float STREAM_STOP_TOL_DEG  = 0.9f;
static const float STREAM_START_TOL_DEG = 1.6f;

static float KP_V[NS]            = { 2.5f, 3.0f, 2.5f };
static float VMAX_DEG_S[NS]      = { 8.0f, 5.0f, 4.0f };
static float AMAX_DEG_S2[NS]     = { 60.0f, 40.0f, 5.0f };

static float VMIN_DEG_S[NS]      = { 4.0f,  2.0f,  0.0f };
static float VMIN_ENABLE_ERR[NS] = { 6.0f,  8.0f,  999.0f };

static float VMIN_STREAM_DEG_S[NS]   = { 1.2f, 1.0f, 1.2f };
static float VMIN_STREAM_ERR_DEG[NS] = { 2.0f, 2.0f, 2.0f };

static uint32_t ctl_last_us = 0;
static const uint32_t CTL_PERIOD_US = 10000; // 100 Hz

static inline float rosTargetToPhysDeg(uint8_t axis, float ros_rad){
  return rad2deg(ROS_CMD_SIGN(axis) * ros_rad);
}

// ---------------------- command stream state ----------------------
static bool ros_stream_active = false;
static uint32_t ros_last_rx_ms = 0;
static const uint32_t ROS_STREAM_TIMEOUT_MS = 60000;
static const uint32_t STREAM_ACTIVE_MS = 200;

static uint32_t last_any_motion_ms = 0;

static void controlTickClosedLoop(){
  if(fault_latched){
    stopAllSpeeds();
    return;
  }
  if(!homed_this_boot || !have_cmd_since_home || !S.home_set){
    stopAllSpeeds();
    for(uint8_t j=0;j<NS;j++){
      v_cur_deg_s[j] = 0.0f;
      at_target[j] = true;
      wd_arm_ms[j] = 0;
    }
    return;
  }

  uint32_t now = micros();
  if((uint32_t)(now - ctl_last_us) < CTL_PERIOD_US) return;

  float dt = (ctl_last_us == 0) ? (CTL_PERIOD_US * 1e-6f) : ((now - ctl_last_us) * 1e-6f);
  ctl_last_us = now;
  if(dt < 1e-4f) dt = 1e-4f;

  uint32_t now_ms = millis();
  bool streaming_now = ros_stream_active && ((uint32_t)(now_ms - ros_last_rx_ms) <= STREAM_ACTIVE_MS);

  bool any_need_move = false;

  for(uint8_t axis=0; axis<NS; axis++){
    float tgt_deg  = rosTargetToPhysDeg(axis, tgt_ros_rad[axis]);

    if(axis == BASE){
      tgt_deg = clampf(tgt_deg, BASE_MIN_JOINT_DEG, BASE_MAX_JOINT_DEG);
    }else if(axis == ARM1){
      tgt_deg = clampf(tgt_deg, rad2deg(-0.50f), rad2deg(0.20f));
    }else if(axis == ARM2){
      tgt_deg = clampf(tgt_deg, rad2deg(-0.7853f), rad2deg(1.0472f));
    }

    float meas_deg;
    if(axis == ARM2){
      meas_deg = fb_joint_deg[axis];
    }else{
      meas_deg = fb_joint_deg[axis];
    }

    float err_deg = tgt_deg - meas_deg;
    float ae = fabsf(err_deg);

    float stop_tol  = streaming_now ? STREAM_STOP_TOL_DEG  : STOP_TOL_DEG[axis];
    float start_tol = streaming_now ? STREAM_START_TOL_DEG : START_TOL_DEG[axis];

    if(at_target[axis]){
      if(ae >= start_tol) at_target[axis] = false;
    }else{
      if(ae <= stop_tol) at_target[axis] = true;
    }

    if(at_target[axis]){
      v_cur_deg_s[axis] = 0.0f;
      stp[axis].setSpeed(0.0f);
      if(fabsf(err_deg) > 6.0f){
        faultStop(210 + axis);
      }
      enc_last_deg[axis] = fb_joint_deg_rawrel[axis];
      wd_arm_ms[axis] = 0;
      float tol = (axis == ARM2) ? 5.0f : 10.0f;
      if(fabsf(err_deg) > tol){
        faultStop(203 + axis);
      }
      continue;
    }

    any_need_move = true;

    float v_des = clampSym(KP_V[axis] * err_deg, VMAX_DEG_S[axis]);

    if(streaming_now){
      if(ae >= VMIN_STREAM_ERR_DEG[axis] && fabsf(v_des) < VMIN_STREAM_DEG_S[axis]){
        v_des = (v_des >= 0.0f) ? VMIN_STREAM_DEG_S[axis] : -VMIN_STREAM_DEG_S[axis];
      }
    }else{
      if(ae >= VMIN_ENABLE_ERR[axis] && fabsf(v_des) < VMIN_DEG_S[axis]){
        v_des = (v_des >= 0.0f) ? VMIN_DEG_S[axis] : -VMIN_DEG_S[axis];
      }
    }

    float dv = v_des - v_cur_deg_s[axis];
    float dv_max = AMAX_DEG_S2[axis] * dt;
    dv = clampSym(dv, dv_max);
    v_cur_deg_s[axis] += dv;

    bool wd_on = (have_cmd_since_home &&
                  ros_stream_active &&
                  (fabsf(v_cur_deg_s[axis]) >= WD_V_THRESH_DEG_S) &&
                  (ae >= WD_ERR_THRESH_DEG));

    if(wd_on){
      if(wd_arm_ms[axis] == 0){
        wd_arm_ms[axis] = now_ms;
        enc_last_change_ms[axis] = now_ms;
        enc_last_deg[axis] = fb_joint_deg_rawrel[axis];
      }

      uint32_t t_on = (uint32_t)(now_ms - wd_arm_ms[axis]);
      uint32_t t_since_change = (uint32_t)(now_ms - enc_last_change_ms[axis]);

      if(t_on > MOVE_GRACE_MS && t_since_change > ENC_STUCK_MS){
        faultStop(200 + axis);
      }
    }else{
      wd_arm_ms[axis] = 0;
    }

    float steps_s = v_cur_deg_s[axis] * stepsPerDeg(axis);
    float steps_s_max = VMAX_DEG_S[axis] * stepsPerDeg(axis);
    steps_s = clampf(steps_s, -steps_s_max, +steps_s_max);
    stp[axis].setSpeed(steps_s);
  }

  if(any_need_move){
    last_any_motion_ms = millis();
    enableAllDrivers(true);
  }
}

// ---------------------- CAPHOME ----------------------
static float caphomeReadStable(uint8_t j, uint16_t navg, float max_jump){
  float raw = 0.0f;
  for(int k=0;k<10;k++){
    bool ok = readAbsDegStable(j, navg, max_jump, raw);
    if(ok) return raw;
    delay(5);
  }
  return raw;
}

static void doCaphomeNow(){
  enableAllDrivers(true);
  stopAllSpeeds();
  for(uint8_t j=0;j<NS;j++){
    v_cur_deg_s[j] = 0.0f;
    at_target[j] = true;
    wd_arm_ms[j] = 0;
  }

  float a0=0, bm=0, a2=0;
  a0 = caphomeReadStable(ARM1, 20, STABLE_JUMP_MAX_DEG[ARM1]);
  bm = caphomeReadStable(BASE, 20, STABLE_JUMP_MAX_DEG[BASE]);
  a2 = caphomeReadStable(ARM2, 20, STABLE_JUMP_MAX_DEG[ARM2]);

  S.home_abs_deg[ARM1] = a0;
  S.home_abs_deg[BASE] = bm;
  S.home_abs_deg[ARM2] = a2;
  S.home_set = 1;

  int wp = wristReadPosRetry(10);
  if(wp >= 0) wrist_home_tick = (uint16_t)wristClampAbsTick(wp);

  saveSettings();

  fb_joint_deg[ARM1] = 0.0f;
  fb_joint_deg[BASE] = 0.0f;
  fb_joint_deg[ARM2] = 0.0f;

  fb_joint_deg_rawrel[ARM1] = 0.0f;
  fb_joint_deg_rawrel[BASE] = 0.0f;
  fb_joint_deg_rawrel[ARM2] = 0.0f;

  base_track_init = true;
  base_prev_raw_deg = bm;
  base_motor_unwrapped_deg = 0.0f;

  uint32_t now_ms = millis();
  for(uint8_t j=0;j<NS;j++){
    enc_last_deg[j] = 0.0f;
    enc_last_change_ms[j] = now_ms;
    wd_arm_ms[j] = 0;
  }

  enc_last_update_ms = 0;

  tgt_ros_rad[ARM1] = 0.0f;
  tgt_ros_rad[BASE] = 0.0f;
  tgt_ros_rad[ARM2] = 0.0f;
  have_cmd_since_home = false;

  fault_latched = false;
  fault_code = 0;

  homed_this_boot = true;

  ros_stream_active = false;
  ros_last_rx_ms = millis();

  (void)wristPollPresent();

  last_any_motion_ms = millis();
}

// ---------------------- command application ----------------------
static void applyArmTargetsRosRad(float cont_ros, float rev1_ros, float rev2_ros){
  if(fault_latched) return;
  if(!homed_this_boot || !S.home_set) return;

  cont_ros = clampf(cont_ros, BASE_MIN_ROS_RAD, BASE_MAX_ROS_RAD);
  rev1_ros = clampf(rev1_ros, -0.50f,    0.20f);
  rev2_ros = clampf(rev2_ros, -0.7853f,  1.0472f);

  enableAllDrivers(true);

  ros_stream_active = true;
  ros_last_rx_ms = millis();

  tgt_ros_rad[BASE] = cont_ros;
  tgt_ros_rad[ARM1] = rev1_ros;
  tgt_ros_rad[ARM2] = rev2_ros;

  at_target[ARM1] = false;
  at_target[BASE] = false;
  at_target[ARM2] = false;

  have_cmd_since_home = true;
  last_any_motion_ms = millis();
}

static void applyGripperTargetRosRad(float rev3_ros){
  if(fault_latched) return;
  if(!homed_this_boot || !S.home_set) return;

  rev3_ros = clampf(rev3_ros, REV3_MIN_ROS_RAD, REV3_MAX_ROS_RAD);

  ros_stream_active = true;
  ros_last_rx_ms = millis();

  (void)wristSendTargetRelRad(rev3_ros);
  have_cmd_since_home = true;
}

static void rosStreamWatchdog(){
  if(!ros_stream_active) return;
  uint32_t now = millis();
  if(now - ros_last_rx_ms > ROS_STREAM_TIMEOUT_MS){
    ros_stream_active = false;
    stopAllSpeeds();
    for(uint8_t j=0;j<NS;j++){
      v_cur_deg_s[j] = 0.0f;
      at_target[j] = true;
      wd_arm_ms[j] = 0;
    }
  }
}

// ---------------------- simple gripper servo on pin 10 ----------------------
static const int GRIPPER_SERVO_PIN = 10;
static Servo gripper_servo;

static const float GRIPPER_CLOSE_DEG = 135.0f;   // tune
static const float GRIPPER_OPEN_DEG  = 65.0f;  // tune

static void setGripperFromFloat(float v){
  float alpha = clampf(v, 0.0f, 1.0f);     // 0.0 = close, 1.0 = open
  float deg = GRIPPER_CLOSE_DEG + (GRIPPER_OPEN_DEG - GRIPPER_CLOSE_DEG) * alpha;
  gripper_servo.write((int)deg);
}

// ---------------------- micro-ROS objects ----------------------
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;

static rcl_publisher_t js_pub;
static rcl_publisher_t wrist_dbg_pub;
static rcl_publisher_t status_pub;

static rcl_subscription_t arm_sub;
static rcl_subscription_t grip_sub;
static rcl_subscription_t caphome_sub;
static rcl_subscription_t reset_sub;

// NEW: jaw servo topic
static rcl_subscription_t jaws_sub;

static rcl_timer_t js_timer;
static rclc_executor_t executor;

static sensor_msgs__msg__JointState js_msg;
static std_msgs__msg__Float64MultiArray arm_msg_in;
static std_msgs__msg__Float64 grip_msg_in;
static std_msgs__msg__Bool caphome_msg_in;
static std_msgs__msg__Bool reset_msg_in;
static std_msgs__msg__Float64MultiArray wrist_dbg_msg;
static std_msgs__msg__Float64MultiArray status_msg;

// NEW jaws message
static std_msgs__msg__Float64 jaws_msg_in;

// message backing storage
static double js_pos[4];
static double js_vel[4];

static rosidl_runtime_c__String js_names[4];
static char n0[] = "continuous";
static char n1[] = "revolute1";
static char n2[] = "revolute2";
static char n3[] = "revolute3";
static char frame_id[] = "";

static double arm_in_buf[4];
static double wrist_dbg_buf[6];
static double status_buf[24];

static void init_jointstate_message(){
  js_msg.header.frame_id.data = frame_id;
  js_msg.header.frame_id.size = 0;
  js_msg.header.frame_id.capacity = sizeof(frame_id);

  js_msg.name.data = js_names;
  js_msg.name.size = 4;
  js_msg.name.capacity = 4;

  js_names[0].data = n0; js_names[0].size = strlen(n0); js_names[0].capacity = sizeof(n0);
  js_names[1].data = n1; js_names[1].size = strlen(n1); js_names[1].capacity = sizeof(n1);
  js_names[2].data = n2; js_names[2].size = strlen(n2); js_names[2].capacity = sizeof(n2);
  js_names[3].data = n3; js_names[3].size = strlen(n3); js_names[3].capacity = sizeof(n3);

  js_msg.position.data = js_pos;
  js_msg.position.size = 4;
  js_msg.position.capacity = 4;

  js_msg.velocity.data = js_vel;
  js_msg.velocity.size = 4;
  js_msg.velocity.capacity = 4;

  js_msg.effort.data = NULL;
  js_msg.effort.size = 0;
  js_msg.effort.capacity = 0;
}

static void init_arm_in_message(){
  arm_msg_in.layout.dim.data = NULL;
  arm_msg_in.layout.dim.size = 0;
  arm_msg_in.layout.dim.capacity = 0;
  arm_msg_in.layout.data_offset = 0;

  arm_msg_in.data.data = arm_in_buf;
  arm_msg_in.data.size = 0;
  arm_msg_in.data.capacity = 4;
}

static void init_wrist_dbg_message(){
  wrist_dbg_msg.layout.dim.data = NULL;
  wrist_dbg_msg.layout.dim.size = 0;
  wrist_dbg_msg.layout.dim.capacity = 0;
  wrist_dbg_msg.layout.data_offset = 0;

  wrist_dbg_msg.data.data = wrist_dbg_buf;
  wrist_dbg_msg.data.size = 6;
  wrist_dbg_msg.data.capacity = 6;
}

static void init_status_message(){
  status_msg.layout.dim.data = NULL;
  status_msg.layout.dim.size = 0;
  status_msg.layout.dim.capacity = 0;
  status_msg.layout.data_offset = 0;

  status_msg.data.data = status_buf;
  status_msg.data.size = 24;
  status_msg.data.capacity = 24;
}

static void error_blink_loop(){
  pinMode(LED_BUILTIN, OUTPUT);
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(120);
  }
}

#define RCCHECK(fn) do { rcl_ret_t rc = (fn); if(rc != RCL_RET_OK){ error_blink_loop(); } } while(0)
#define RCSOFTCHECK(fn) do { rcl_ret_t rc = (fn); (void)rc; } while(0)

// ---------------------- micro-ROS callbacks ----------------------
static void arm_sub_cb(const void * msgin){
  const std_msgs__msg__Float64MultiArray *m = (const std_msgs__msg__Float64MultiArray*)msgin;
  if(m->data.size < 3) return;
  applyArmTargetsRosRad((float)m->data.data[0], (float)m->data.data[1], (float)m->data.data[2]);
  if(m->data.size >= 4) applyGripperTargetRosRad((float)m->data.data[3]);
}

static void grip_sub_cb(const void * msgin){
  const std_msgs__msg__Float64 *m = (const std_msgs__msg__Float64*)msgin;
  applyGripperTargetRosRad((float)m->data);
}

// NEW: jaw servo callback – Float64 0.0..1.0 on /teensy_jaw_target
static void jaws_sub_cb(const void * msgin){
  const std_msgs__msg__Float64 *m = (const std_msgs__msg__Float64*)msgin;
  setGripperFromFloat((float)m->data);
}

static void caphome_sub_cb(const void * msgin){
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool*)msgin;
  if(!m->data) return;
  doCaphomeNow();
}

static void reset_sub_cb(const void * msgin){
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool*)msgin;
  if(!m->data) return;

  factoryResetEEPROM();

  S.home_set = 0;
  homed_this_boot = false;
  have_cmd_since_home = false;

  fault_latched = false;
  fault_code = 0;

  stopAllSpeeds();
  for(uint8_t j=0;j<NS;j++){
    v_cur_deg_s[j] = 0.0f;
    at_target[j] = true;
    wd_arm_ms[j] = 0;
    fb_joint_deg_rawrel[j] = 0.0f;
    fb_joint_deg[j] = 0.0f;
  }

  base_track_init = false;
  base_motor_unwrapped_deg = 0.0f;
  base_prev_raw_deg = 0.0f;

  tgt_ros_rad[ARM1] = tgt_ros_rad[BASE] = tgt_ros_rad[ARM2] = 0.0f;

  ros_stream_active = false;
  ros_last_rx_ms = millis();

  enc_last_update_ms = 0;

  enableAllDrivers(true);
  last_any_motion_ms = millis();
}

static inline float physDegToRosPos(uint8_t axis, float phys_deg){
  return physRadToRosRadRaw(axis, deg2rad(phys_deg));
}

static inline float physDegToRosVel(uint8_t axis, float phys_deg_s){
  return physRadToRosRadRaw(axis, deg2rad(phys_deg_s));
}

// Throttle wrist polling to 10 Hz
static uint32_t wrist_last_poll_ms = 0;

static void js_timer_cb(rcl_timer_t * timer, int64_t /*last_call_time*/){
  if(timer == NULL) return;

  uint32_t ms = millis();

  if((uint32_t)(ms - wrist_last_poll_ms) >= 100){
    wrist_last_poll_ms = ms;
    (void)wristPollPresent();
  }

  float cont_pos = 0.0f, rev1_pos = 0.0f, rev2_pos = 0.0f;
  float cont_vel = 0.0f, rev1_vel = 0.0f, rev2_vel = 0.0f;

  if(homed_this_boot && S.home_set){
    cont_pos = physDegToRosPos(BASE, clampf(fb_joint_deg[BASE], BASE_MIN_JOINT_DEG, BASE_MAX_JOINT_DEG));
    cont_pos = clampf(cont_pos, BASE_MIN_ROS_RAD, BASE_MAX_ROS_RAD);

    rev1_pos = physDegToRosPos(ARM1, fb_joint_deg[ARM1]);
    rev2_pos = physDegToRosPos(ARM2, fb_joint_deg[ARM2]);

    cont_vel = physDegToRosVel(BASE, v_cur_deg_s[BASE]);
    rev1_vel = physDegToRosVel(ARM1, v_cur_deg_s[ARM1]);
    rev2_vel = physDegToRosVel(ARM2, v_cur_deg_s[ARM2]);
  }

  float rev3_pos = clampf(wrist_present_rel_rad, REV3_MIN_ROS_RAD, REV3_MAX_ROS_RAD);
  float rev3_vel = 0.0f;

  js_pos[RJ_CONT] = (double)cont_pos;
  js_pos[RJ_REV1] = (double)rev1_pos;
  js_pos[RJ_REV2] = (double)rev2_pos;
  js_pos[RJ_REV3] = (double)rev3_pos;

  js_vel[RJ_CONT] = (double)cont_vel;
  js_vel[RJ_REV1] = (double)rev1_vel;
  js_vel[RJ_REV2] = (double)rev2_vel;
  js_vel[RJ_REV3] = (double)rev3_vel;

  js_msg.header.stamp.sec = (int32_t)(ms / 1000);
  js_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

  RCSOFTCHECK(rcl_publish(&js_pub, &js_msg, NULL));

  wrist_dbg_buf[0] = (double)wrist_present_tick;
  wrist_dbg_buf[1] = (double)wrist_home_tick;
  wrist_dbg_buf[2] = (double)wrist_last_tgt_tick;
  wrist_dbg_buf[3] = (double)wrist_last_dtick;
  wrist_dbg_buf[4] = (double)wrist_last_lo;
  wrist_dbg_buf[5] = (double)wrist_last_hi;
  RCSOFTCHECK(rcl_publish(&wrist_dbg_pub, &wrist_dbg_msg, NULL));

  bool streaming_now = ros_stream_active && ((uint32_t)(millis() - ros_last_rx_ms) <= STREAM_ACTIVE_MS);

  float tgt_phys_arm1 = rosTargetToPhysDeg(ARM1, tgt_ros_rad[ARM1]);
  float tgt_phys_base = rosTargetToPhysDeg(BASE, tgt_ros_rad[BASE]);
  float tgt_phys_arm2 = rosTargetToPhysDeg(ARM2, tgt_ros_rad[ARM2]);

  float err_arm1 = tgt_phys_arm1 - fb_joint_deg[ARM1];
  float err_base = tgt_phys_base - fb_joint_deg[BASE];
  float err_arm2 = tgt_phys_arm2 - fb_joint_deg[ARM2];

  status_buf[0]  = homed_this_boot ? 1.0 : 0.0;
  status_buf[1]  = have_cmd_since_home ? 1.0 : 0.0;
  status_buf[2]  = fault_latched ? 1.0 : 0.0;
  status_buf[3]  = (double)fault_code;
  status_buf[4]  = streaming_now ? 1.0 : 0.0;

  status_buf[5]  = (double)fb_joint_deg[ARM1];
  status_buf[6]  = (double)fb_joint_deg[BASE];
  status_buf[7]  = (double)fb_joint_deg[ARM2];
  status_buf[8]  = (double)base_motor_unwrapped_deg;

  status_buf[9]  = (double)tgt_ros_rad[ARM1];
  status_buf[10] = (double)tgt_ros_rad[BASE];
  status_buf[11] = (double)tgt_ros_rad[ARM2];

  status_buf[12] = (double)tgt_phys_arm1;
  status_buf[13] = (double)tgt_phys_base;
  status_buf[14] = (double)tgt_phys_arm2;

  status_buf[15] = (double)err_arm1;
  status_buf[16] = (double)err_base;
  status_buf[17] = (double)err_arm2;

  status_buf[18] = (double)v_cur_deg_s[ARM1];
  status_buf[19] = (double)v_cur_deg_s[BASE];
  status_buf[20] = (double)v_cur_deg_s[ARM2];

  status_buf[21] = at_target[ARM1] ? 1.0 : 0.0;
  status_buf[22] = at_target[BASE] ? 1.0 : 0.0;
  status_buf[23] = at_target[ARM2] ? 1.0 : 0.0;

  RCSOFTCHECK(rcl_publish(&status_pub, &status_msg, NULL));
}

// ---------------------- setup / loop ----------------------
void setup(){
  forcePinsSafeLow();

  set_microros_transports();

  analogReadResolution(12);
  analogReadAveraging(4);

  Serial1.begin(WRIST_BAUD);
  sc.pSerial = &Serial1;
  delay(30);
  (void)sc.Ping(WRIST_ID);
  delay(30);
  sc.EnableTorque(WRIST_ID, 1);
  delay(30);

  loadSettings();

  DIR_INVERT[BASE] = FORCE_BASE_DIR_INVERT;
  ENC_INVERT[BASE] = FORCE_BASE_ENC_INVERT;

  S.home_set = 0;
  homed_this_boot = false;
  have_cmd_since_home = false;
  fault_latched = false;
  fault_code = 0;

  ros_stream_active = false;
  ros_last_rx_ms = millis();

  tgt_ros_rad[ARM1] = tgt_ros_rad[BASE] = tgt_ros_rad[ARM2] = 0.0f;

  stopAllSpeeds();
  for(uint8_t j=0;j<NS;j++){
    v_cur_deg_s[j] = 0.0f;
    at_target[j] = true;
    enc_last_deg[j] = 0.0f;
    enc_last_change_ms[j] = millis();
    wd_arm_ms[j] = 0;
    fb_joint_deg_rawrel[j] = 0.0f;
    fb_joint_deg[j] = 0.0f;
  }

  base_track_init = false;
  base_motor_unwrapped_deg = 0.0f;
  base_prev_raw_deg = 0.0f;

  applyStepperConfig();
  enableAllDrivers(true);

  (void)wristPollPresent();
  wrist_last_poll_ms = millis();

  enc_last_update_ms = 0;

  // Attach gripper servo and start open
  gripper_servo.attach(GRIPPER_SERVO_PIN);
  gripper_servo.write((int)GRIPPER_OPEN_DEG);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "asrs_teensy", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &js_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/teensy/joint_states"
  ));

  RCCHECK(rclc_publisher_init_default(
    &wrist_dbg_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/teensy/wrist_debug"
  ));

  RCCHECK(rclc_publisher_init_default(
    &status_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/teensy/status"
  ));

  RCCHECK(rclc_subscription_init_default(
    &arm_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "/teensy_joint_targets"
  ));

  RCCHECK(rclc_subscription_init_default(
    &grip_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/teensy_gripper_angle_target"
  ));

  RCCHECK(rclc_subscription_init_default(
    &caphome_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/teensy_caphome"
  ));

  RCCHECK(rclc_subscription_init_default(
    &reset_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/teensy_factory_reset"
  ));

  // NEW: jaw servo topic
  RCCHECK(rclc_subscription_init_default(
    &jaws_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "/teensy_jaw_target"
  ));

  const uint32_t pub_period_ms = 20;
  RCCHECK(rclc_timer_init_default(
    &js_timer, &support,
    RCL_MS_TO_NS(pub_period_ms),
    js_timer_cb
  ));

  init_jointstate_message();
  init_arm_in_message();
  init_wrist_dbg_message();
  init_status_message();

  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &arm_sub, &arm_msg_in, &arm_sub_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &grip_sub, &grip_msg_in, &grip_sub_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &caphome_sub, &caphome_msg_in, &caphome_sub_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &reset_sub, &reset_msg_in, &reset_sub_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &jaws_sub, &jaws_msg_in, &jaws_sub_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &js_timer));

  last_any_motion_ms = millis();
}

void loop(){
  for(uint8_t j=0;j<NS;j++){
    stp[j].runSpeed();
  }

  updateFeedbackFromEncoders_Throttled();

  controlTickClosedLoop();

  rosStreamWatchdog();

  if(ALLOW_IDLE_DRIVER_DISABLE){
    uint32_t now_ms = millis();
    bool streaming_now = ros_stream_active && ((uint32_t)(now_ms - ros_last_rx_ms) <= STREAM_ACTIVE_MS);
    bool all_at = at_target[ARM1] && at_target[BASE] && at_target[ARM2];
    if(all_at && !streaming_now && have_cmd_since_home){
      if((uint32_t)(now_ms - last_any_motion_ms) > IDLE_DISABLE_MS){
        enableAllDrivers(false);
      }
    }
  }

  static uint32_t last_spin_ms = 0;
  uint32_t ms = millis();
  if((uint32_t)(ms - last_spin_ms) >= 2){
    last_spin_ms = ms;
    (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
}
