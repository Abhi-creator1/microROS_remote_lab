/*
  ASRS Robot Teensy 4.1 micro-ROS firmware (new simplified version)

  Key improvements over v11.1:
  - Reconnection state machine (no more infinite blink loop)
  - Clean robot state machine (IDLE -> HOMED -> MOVING)
  - Consistent index ordering: BASE=0, SHOULDER=1, ELBOW=2 matching ROS
  - No EEPROM storage of hardware constants (only home positions)
  - No factory reset, no streaming mode, no double fault checks
  - Encoder glitch rejection preserved
  - Proportional velocity control preserved
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <math.h>
#include <SCServo.h>
#include <Servo.h>

// micro-ROS
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/bool.h>

// ===================== HELPERS =====================
static inline float deg2rad(float d) { return d * 0.01745329252f; }
static inline float rad2deg(float r) { return r * 57.295779513f; }
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static inline float clampSym(float x, float lim) { return clampf(x, -lim, +lim); }

static float wrap360(float x) {
  while (x >= 360.0f) x -= 360.0f;
  while (x < 0.0f)   x += 360.0f;
  return x;
}

static float angleDiffDeg(float target_deg, float current_deg) {
  float t = deg2rad(target_deg);
  float c = deg2rad(current_deg);
  return rad2deg(atan2f(sinf(t - c), cosf(t - c)));
}

static float meanAngleDeg(float a_deg, float b_deg) {
  float a = deg2rad(a_deg), b = deg2rad(b_deg);
  float ang = atan2f(sinf(a) + sinf(b), cosf(a) + cosf(b));
  if (ang < 0) ang += TWO_PI;
  return wrap360(rad2deg(ang));
}

static inline uint16_t adcRead12(int pin) {
  (void)analogRead(pin);
  uint16_t raw = (uint16_t)analogRead(pin);
  if (raw >= 4095) raw = 4094;
  return raw;
}

// ===================== JOINT INDICES =====================
// Consistent ordering matching ROS joint order
enum : uint8_t { BASE = 0, SHOULDER = 1, ELBOW = 2 };
static const uint8_t NS = 3;

// ===================== PINS =====================
static const int STEP_PIN[NS] = { 6, 3, 9 };   // BASE, SHOULDER, ELBOW
static const int DIR_PIN[NS]  = { 5, 2, 8 };
static const int ENC_PIN[NS]  = { A2, A0, A1 }; // BASE, SHOULDER, ELBOW

// ===================== HARDWARE CONSTANTS =====================
static const float BASE_GEAR_RATIO = 4.0f;  // 80:20 = 4:1
static const float SPR[NS] = { 1600.0f, 64000.0f, 8000.0f };  // steps per rev (motor)

static const bool DIR_INVERT[NS]  = { true, false, false };  // BASE dir inverted
static const bool FB_INVERT[NS]   = { false, false, true };  // ELBOW feedback inverted

// Joint limits (radians, ROS space)
static const float JOINT_MIN_RAD[NS] = { deg2rad(-170.0f), -0.50f,   -0.7853f };
static const float JOINT_MAX_RAD[NS] = { deg2rad(+170.0f),  0.20f,    1.0472f  };

// Stepper speed caps
static const float MAX_SPEED_SPS[NS] = { 1800, 5000, 1800 };  // steps/sec
static const float ACCEL_SPS2[NS]    = { 100,  600,  100 };

// Closed-loop control parameters
static const float KP_V[NS]       = { 3.0f, 2.5f, 2.5f };
static const float VMAX_DEG_S[NS] = { 5.0f, 8.0f, 4.0f };
static const float AMAX_DEG_S2[NS]= { 40.0f, 60.0f, 5.0f };
static const float STOP_TOL_DEG[NS]  = { 2.0f, 1.8f, 1.0f };
static const float START_TOL_DEG[NS] = { 3.0f, 2.6f, 1.6f };

// Encoder stability
static const float STABLE_JUMP_MAX_DEG[NS] = { 12.0f, 6.0f, 20.0f };
static const uint16_t FB_NAVG[NS] = { 3, 3, 3 };
static const float FB_ALPHA[NS]   = { 0.25f, 0.20f, 0.20f };

static const uint32_t CTL_PERIOD_US = 10000;  // 100 Hz
static const uint32_t ENC_UPDATE_MS = 25;
static const uint16_t MIN_PULSE_US = 5;

// ===================== WRIST SERVO (SCS15) =====================
SCSCL sc;
static const uint8_t WRIST_ID = 1;
static const long WRIST_BAUD = 1000000;
static const int WRIST_POS_MAX = 1023;
static const float WRIST_DEG_MAX = 220.0f;
static const float REV3_MIN_RAD = -2.600f;
static const float REV3_MAX_RAD = 0.000f;
static const uint16_t WRIST_SPEED = 400;
static const uint8_t WRIST_ACCEL = 4;

static uint16_t wrist_home_tick = 750;
static float wrist_present_rad = 0.0f;

// ===================== JAW SERVO (PWM) =====================
static const int JAW_PIN = 10;
static Servo jaw_servo;
static const float JAW_CLOSE_DEG = 135.0f;
static const float JAW_OPEN_DEG = 65.0f;

static void setJaw(float v) {
  float alpha = clampf(v, 0.0f, 1.0f);
  float deg = JAW_CLOSE_DEG + (JAW_OPEN_DEG - JAW_CLOSE_DEG) * alpha;
  jaw_servo.write((int)deg);
}

// ===================== STEPPERS =====================
AccelStepper stp[NS] = {
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[0], DIR_PIN[0]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[1], DIR_PIN[1]),
  AccelStepper(AccelStepper::DRIVER, STEP_PIN[2], DIR_PIN[2])
};

static void stopAllSpeeds() {
  for (uint8_t j = 0; j < NS; j++) stp[j].setSpeed(0.0f);
}

static inline float stepsPerDeg(uint8_t j) {
  if (j == BASE) return (SPR[BASE] * BASE_GEAR_RATIO) / 360.0f;
  return SPR[j] / 360.0f;
}

// ===================== EEPROM (home positions only) =====================
struct HomeData {
  uint32_t magic;
  uint8_t valid;
  float home_abs_deg[NS];
  uint16_t wrist_home_tick;
};
static HomeData home_data;
static const uint32_t MAGIC = 0x41535297u;

static void saveHome() {
  home_data.magic = MAGIC;
  EEPROM.put(0, home_data);
}

static void loadHome() {
  EEPROM.get(0, home_data);
  if (home_data.magic != MAGIC) {
    memset(&home_data, 0, sizeof(home_data));
    home_data.valid = 0;
  } else if (home_data.valid && home_data.wrist_home_tick <= WRIST_POS_MAX) {
    wrist_home_tick = home_data.wrist_home_tick;
  }
}

// ===================== ENCODER FEEDBACK =====================
static float fb_joint_deg[NS] = {0, 0, 0};

// Base unwrapping
static bool base_track_init = false;
static float base_prev_raw_deg = 0.0f;
static float base_unwrapped_motor_deg = 0.0f;

static float readAbsDeg(uint8_t j, uint16_t n) {
  float sx = 0, sy = 0;
  for (uint16_t i = 0; i < n; i++) {
    uint16_t raw = adcRead12(ENC_PIN[j]);
    float a = (raw / 4095.0f) * TWO_PI;
    sx += cosf(a);
    sy += sinf(a);
  }
  float ang = atan2f(sy, sx);
  if (ang < 0) ang += TWO_PI;
  return wrap360(rad2deg(ang));
}

static bool readAbsDegStable(uint8_t j, uint16_t navg, float max_jump, float &out) {
  float a = readAbsDeg(j, navg);
  delayMicroseconds(250);
  float b = readAbsDeg(j, navg);
  if (fabsf(angleDiffDeg(b, a)) > max_jump) {
    out = b;
    return false;
  }
  out = meanAngleDeg(a, b);
  return true;
}

static float caphomeReadStable(uint8_t j) {
  float raw = 0.0f;
  for (int k = 0; k < 10; k++) {
    if (readAbsDegStable(j, 20, STABLE_JUMP_MAX_DEG[j], raw)) return raw;
    delay(5);
  }
  return raw;
}

static float baseUpdateUnwrapped(float raw_deg) {
  if (!base_track_init) {
    base_prev_raw_deg = raw_deg;
    base_unwrapped_motor_deg = -angleDiffDeg(home_data.home_abs_deg[BASE], raw_deg);
    base_track_init = true;
    return base_unwrapped_motor_deg;
  }
  float d = angleDiffDeg(raw_deg, base_prev_raw_deg);
  if (fabsf(d) < 0.08f) d = 0.0f;
  base_unwrapped_motor_deg += d;
  base_prev_raw_deg = raw_deg;
  return base_unwrapped_motor_deg;
}

static uint32_t enc_last_update_ms = 0;

static void updateEncoders() {
  if (!home_data.valid) return;
  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - enc_last_update_ms) < ENC_UPDATE_MS) return;
  enc_last_update_ms = now_ms;

  // BASE (geared, unwrapped)
  {
    float raw = 0.0f;
    if (readAbsDegStable(BASE, FB_NAVG[BASE], STABLE_JUMP_MAX_DEG[BASE], raw)) {
      float motor_deg = baseUpdateUnwrapped(raw);
      float joint_deg = motor_deg / BASE_GEAR_RATIO;
      float prev = fb_joint_deg[BASE];
      fb_joint_deg[BASE] = prev + FB_ALPHA[BASE] * (joint_deg - prev);
    }
  }

  // SHOULDER and ELBOW (direct)
  for (uint8_t j = SHOULDER; j <= ELBOW; j++) {
    float raw = 0.0f;
    if (readAbsDegStable(j, FB_NAVG[j], STABLE_JUMP_MAX_DEG[j], raw)) {
      float rel_deg = -angleDiffDeg(home_data.home_abs_deg[j], raw);
      float prev = fb_joint_deg[j];
      float d = angleDiffDeg(rel_deg, prev);
      float filt = prev + FB_ALPHA[j] * d;
      while (filt > 180.0f) filt -= 360.0f;
      while (filt < -180.0f) filt += 360.0f;
      fb_joint_deg[j] = filt;
    }
  }
}

// ===================== WRIST HELPERS =====================
static int wristReadRetry(int tries = 5) {
  for (int i = 0; i < tries; i++) {
    int p = sc.ReadPos(WRIST_ID);
    if (p >= 0) return p;
    delay(4);
  }
  return -1;
}

static void wristPollPresent() {
  int p = wristReadRetry(3);
  if (p < 0) return;
  int dtick = p - (int)wrist_home_tick;
  float deg = (float)dtick * (WRIST_DEG_MAX / (float)WRIST_POS_MAX);
  wrist_present_rad = clampf(deg2rad(deg), REV3_MIN_RAD, REV3_MAX_RAD);
}

static void wristSendTarget(float ros_rad) {
  ros_rad = clampf(ros_rad, REV3_MIN_RAD, REV3_MAX_RAD);
  float deg = rad2deg(ros_rad);
  int dtick = (int)lroundf(deg * ((float)WRIST_POS_MAX / WRIST_DEG_MAX));
  int tgt = (int)wrist_home_tick + dtick;
  tgt = (tgt < 0) ? 0 : (tgt > WRIST_POS_MAX) ? WRIST_POS_MAX : tgt;
  sc.WritePos(WRIST_ID, tgt, WRIST_ACCEL, WRIST_SPEED);
}

// ===================== ROBOT STATE MACHINE =====================
enum RobotState : uint8_t { ROBOT_IDLE, ROBOT_HOMED, ROBOT_MOVING };
static volatile RobotState robot_state = ROBOT_IDLE;

// Control state
static float tgt_ros_rad[NS] = {0, 0, 0};
static float v_cur_deg_s[NS] = {0, 0, 0};
static bool at_target[NS] = {true, true, true};
static uint32_t ctl_last_us = 0;

static inline float rosToPhysDeg(uint8_t j, float ros_rad) {
  return rad2deg(ros_rad);  // no sign inversion in ROS command direction
}

static inline float physToRosRad(uint8_t j, float phys_deg) {
  float rad = deg2rad(phys_deg);
  return FB_INVERT[j] ? -rad : rad;
}

static void controlTick() {
  if (robot_state != ROBOT_HOMED && robot_state != ROBOT_MOVING) {
    stopAllSpeeds();
    return;
  }

  uint32_t now = micros();
  if ((uint32_t)(now - ctl_last_us) < CTL_PERIOD_US) return;
  float dt = (ctl_last_us == 0) ? (CTL_PERIOD_US * 1e-6f) : ((now - ctl_last_us) * 1e-6f);
  ctl_last_us = now;
  if (dt < 1e-4f) dt = 1e-4f;

  bool any_moving = false;

  for (uint8_t j = 0; j < NS; j++) {
    float tgt_deg = rosToPhysDeg(j, tgt_ros_rad[j]);

    // Clamp to joint limits in physical degrees
    float min_deg = rad2deg(JOINT_MIN_RAD[j]);
    float max_deg = rad2deg(JOINT_MAX_RAD[j]);
    tgt_deg = clampf(tgt_deg, min_deg, max_deg);

    float meas_deg = fb_joint_deg[j];
    float err_deg = tgt_deg - meas_deg;
    float ae = fabsf(err_deg);

    // Hysteresis
    if (at_target[j]) {
      if (ae >= START_TOL_DEG[j]) at_target[j] = false;
    } else {
      if (ae <= STOP_TOL_DEG[j]) at_target[j] = true;
    }

    if (at_target[j]) {
      v_cur_deg_s[j] = 0.0f;
      stp[j].setSpeed(0.0f);
      continue;
    }

    any_moving = true;

    // Proportional velocity control with acceleration limiting
    float v_des = clampSym(KP_V[j] * err_deg, VMAX_DEG_S[j]);
    float dv = v_des - v_cur_deg_s[j];
    float dv_max = AMAX_DEG_S2[j] * dt;
    dv = clampSym(dv, dv_max);
    v_cur_deg_s[j] += dv;

    float steps_s = v_cur_deg_s[j] * stepsPerDeg(j);
    float steps_s_max = VMAX_DEG_S[j] * stepsPerDeg(j);
    steps_s = clampf(steps_s, -steps_s_max, +steps_s_max);
    stp[j].setSpeed(steps_s);
  }

  // Update robot state
  if (robot_state == ROBOT_MOVING && !any_moving) {
    robot_state = ROBOT_HOMED;
  }
}

// ===================== CAPHOME =====================
static void doCaphome() {
  stopAllSpeeds();
  for (uint8_t j = 0; j < NS; j++) {
    v_cur_deg_s[j] = 0.0f;
    at_target[j] = true;
  }

  home_data.home_abs_deg[BASE]     = caphomeReadStable(BASE);
  home_data.home_abs_deg[SHOULDER] = caphomeReadStable(SHOULDER);
  home_data.home_abs_deg[ELBOW]    = caphomeReadStable(ELBOW);
  home_data.valid = 1;

  int wp = wristReadRetry(10);
  if (wp >= 0) wrist_home_tick = (uint16_t)((wp < 0) ? 0 : (wp > WRIST_POS_MAX) ? WRIST_POS_MAX : wp);
  home_data.wrist_home_tick = wrist_home_tick;
  saveHome();

  for (uint8_t j = 0; j < NS; j++) {
    fb_joint_deg[j] = 0.0f;
    tgt_ros_rad[j] = 0.0f;
  }

  base_track_init = true;
  base_prev_raw_deg = home_data.home_abs_deg[BASE];
  base_unwrapped_motor_deg = 0.0f;

  enc_last_update_ms = 0;
  ctl_last_us = 0;

  wristPollPresent();

  robot_state = ROBOT_HOMED;
}

// ===================== COMMAND APPLICATION =====================
static void applyArmTargets(float cont, float rev1, float rev2) {
  if (robot_state != ROBOT_HOMED && robot_state != ROBOT_MOVING) return;

  tgt_ros_rad[BASE]     = clampf(cont, JOINT_MIN_RAD[BASE], JOINT_MAX_RAD[BASE]);
  tgt_ros_rad[SHOULDER] = clampf(rev1, JOINT_MIN_RAD[SHOULDER], JOINT_MAX_RAD[SHOULDER]);
  tgt_ros_rad[ELBOW]    = clampf(rev2, JOINT_MIN_RAD[ELBOW], JOINT_MAX_RAD[ELBOW]);

  for (uint8_t j = 0; j < NS; j++) at_target[j] = false;
  robot_state = ROBOT_MOVING;
}

static void applyWristTarget(float rev3) {
  if (robot_state != ROBOT_HOMED && robot_state != ROBOT_MOVING) return;
  wristSendTarget(rev3);
}

// ===================== MICRO-ROS AGENT STATE MACHINE =====================
enum AgentState : uint8_t {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};
static AgentState agent_state = WAITING_AGENT;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;

static rcl_publisher_t js_pub;
static rcl_publisher_t status_pub;

static rcl_subscription_t arm_sub;
static rcl_subscription_t grip_sub;
static rcl_subscription_t caphome_sub;
static rcl_subscription_t jaws_sub;

static rcl_timer_t js_timer;
static rclc_executor_t executor;

// Message objects
static sensor_msgs__msg__JointState js_msg;
static std_msgs__msg__Float64MultiArray arm_msg_in;
static std_msgs__msg__Float64 grip_msg_in;
static std_msgs__msg__Bool caphome_msg_in;
static std_msgs__msg__Float64 jaws_msg_in;
static std_msgs__msg__Float64MultiArray status_msg;

// Message backing storage
static double js_pos[4], js_vel[4];
static rosidl_runtime_c__String js_names[4];
static char n0[] = "continuous";
static char n1[] = "revolute1";
static char n2[] = "revolute2";
static char n3[] = "revolute3";
static char frame_id[] = "";
static double arm_in_buf[4];
static double status_buf[8];

static void init_messages() {
  // Joint state message
  js_msg.header.frame_id.data = frame_id;
  js_msg.header.frame_id.size = 0;
  js_msg.header.frame_id.capacity = sizeof(frame_id);

  js_msg.name.data = js_names;
  js_msg.name.size = 4;
  js_msg.name.capacity = 4;
  js_names[0] = {n0, strlen(n0), sizeof(n0)};
  js_names[1] = {n1, strlen(n1), sizeof(n1)};
  js_names[2] = {n2, strlen(n2), sizeof(n2)};
  js_names[3] = {n3, strlen(n3), sizeof(n3)};

  js_msg.position.data = js_pos;
  js_msg.position.size = 4;
  js_msg.position.capacity = 4;
  js_msg.velocity.data = js_vel;
  js_msg.velocity.size = 4;
  js_msg.velocity.capacity = 4;
  js_msg.effort.data = NULL;
  js_msg.effort.size = 0;
  js_msg.effort.capacity = 0;

  // Arm input message
  arm_msg_in.layout.dim.data = NULL;
  arm_msg_in.layout.dim.size = 0;
  arm_msg_in.layout.dim.capacity = 0;
  arm_msg_in.layout.data_offset = 0;
  arm_msg_in.data.data = arm_in_buf;
  arm_msg_in.data.size = 0;
  arm_msg_in.data.capacity = 4;

  // Status message
  status_msg.layout.dim.data = NULL;
  status_msg.layout.dim.size = 0;
  status_msg.layout.dim.capacity = 0;
  status_msg.layout.data_offset = 0;
  status_msg.data.data = status_buf;
  status_msg.data.size = 8;
  status_msg.data.capacity = 8;
}

// ===================== ROS CALLBACKS =====================
static void arm_sub_cb(const void *msgin) {
  const std_msgs__msg__Float64MultiArray *m = (const std_msgs__msg__Float64MultiArray *)msgin;
  if (m->data.size < 3) return;
  applyArmTargets((float)m->data.data[0], (float)m->data.data[1], (float)m->data.data[2]);
  if (m->data.size >= 4) applyWristTarget((float)m->data.data[3]);
}

static void grip_sub_cb(const void *msgin) {
  const std_msgs__msg__Float64 *m = (const std_msgs__msg__Float64 *)msgin;
  applyWristTarget((float)m->data);
}

static void jaws_sub_cb(const void *msgin) {
  const std_msgs__msg__Float64 *m = (const std_msgs__msg__Float64 *)msgin;
  setJaw((float)m->data);
}

static void caphome_sub_cb(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  if (m->data) doCaphome();
}

static uint32_t wrist_last_poll_ms = 0;

static void js_timer_cb(rcl_timer_t *timer, int64_t) {
  if (timer == NULL) return;

  uint32_t ms = millis();

  // Poll wrist at 10 Hz
  if ((uint32_t)(ms - wrist_last_poll_ms) >= 100) {
    wrist_last_poll_ms = ms;
    wristPollPresent();
  }

  // Build joint state
  float cont_pos = 0.0f, rev1_pos = 0.0f, rev2_pos = 0.0f;
  float cont_vel = 0.0f, rev1_vel = 0.0f, rev2_vel = 0.0f;

  if (home_data.valid) {
    cont_pos = physToRosRad(BASE, clampf(fb_joint_deg[BASE],
                 rad2deg(JOINT_MIN_RAD[BASE]), rad2deg(JOINT_MAX_RAD[BASE])));
    rev1_pos = physToRosRad(SHOULDER, fb_joint_deg[SHOULDER]);
    rev2_pos = physToRosRad(ELBOW, fb_joint_deg[ELBOW]);

    cont_vel = physToRosRad(BASE, v_cur_deg_s[BASE]);
    rev1_vel = physToRosRad(SHOULDER, v_cur_deg_s[SHOULDER]);
    rev2_vel = physToRosRad(ELBOW, v_cur_deg_s[ELBOW]);
  }

  js_pos[0] = (double)cont_pos;
  js_pos[1] = (double)rev1_pos;
  js_pos[2] = (double)rev2_pos;
  js_pos[3] = (double)clampf(wrist_present_rad, REV3_MIN_RAD, REV3_MAX_RAD);

  js_vel[0] = (double)cont_vel;
  js_vel[1] = (double)rev1_vel;
  js_vel[2] = (double)rev2_vel;
  js_vel[3] = 0.0;

  js_msg.header.stamp.sec = (int32_t)(ms / 1000);
  js_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

  rcl_publish(&js_pub, &js_msg, NULL);

  // Status: [homed, state, base_deg, shoulder_deg, elbow_deg, tgt_base, tgt_shoulder, tgt_elbow]
  status_buf[0] = home_data.valid ? 1.0 : 0.0;
  status_buf[1] = (double)robot_state;
  status_buf[2] = (double)fb_joint_deg[BASE];
  status_buf[3] = (double)fb_joint_deg[SHOULDER];
  status_buf[4] = (double)fb_joint_deg[ELBOW];
  status_buf[5] = (double)tgt_ros_rad[BASE];
  status_buf[6] = (double)tgt_ros_rad[SHOULDER];
  status_buf[7] = (double)tgt_ros_rad[ELBOW];

  rcl_publish(&status_pub, &status_msg, NULL);
}

// ===================== ENTITY MANAGEMENT =====================
static bool create_entities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "asrs_teensy", "", &support) != RCL_RET_OK) return false;

  // Publishers
  if (rclc_publisher_init_default(&js_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/teensy/joint_states") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(&status_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/teensy/status") != RCL_RET_OK) return false;

  // Subscribers
  if (rclc_subscription_init_default(&arm_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
      "/teensy_joint_targets") != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&grip_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "/teensy_gripper_angle_target") != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&caphome_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/teensy_caphome") != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(&jaws_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "/teensy_jaw_target") != RCL_RET_OK) return false;

  // Timer (50 Hz joint state publishing)
  if (rclc_timer_init_default(&js_timer, &support,
      RCL_MS_TO_NS(20), js_timer_cb) != RCL_RET_OK) return false;

  // Executor: 4 subscriptions + 1 timer = 5 handles
  if (rclc_executor_init(&executor, &support.context, 5, &allocator) != RCL_RET_OK) return false;

  rclc_executor_add_subscription(&executor, &arm_sub, &arm_msg_in, &arm_sub_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &grip_sub, &grip_msg_in, &grip_sub_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &caphome_sub, &caphome_msg_in, &caphome_sub_cb, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &jaws_sub, &jaws_msg_in, &jaws_sub_cb, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &js_timer);

  return true;
}

static void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&js_pub, &node);
  rcl_publisher_fini(&status_pub, &node);
  rcl_subscription_fini(&arm_sub, &node);
  rcl_subscription_fini(&grip_sub, &node);
  rcl_subscription_fini(&caphome_sub, &node);
  rcl_subscription_fini(&jaws_sub, &node);
  rcl_timer_fini(&js_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ===================== SETUP =====================
void setup() {
  // Safe pin state
  for (uint8_t j = 0; j < NS; j++) {
    pinMode(STEP_PIN[j], OUTPUT);
    pinMode(DIR_PIN[j], OUTPUT);
    digitalWrite(STEP_PIN[j], LOW);
    digitalWrite(DIR_PIN[j], LOW);
  }

  set_microros_transports();

  analogReadResolution(12);
  analogReadAveraging(4);

  // Wrist servo init
  Serial1.begin(WRIST_BAUD);
  sc.pSerial = &Serial1;
  delay(30);
  sc.Ping(WRIST_ID);
  delay(30);
  sc.EnableTorque(WRIST_ID, 1);
  delay(30);

  loadHome();

  // Stepper config
  for (uint8_t j = 0; j < NS; j++) {
    stp[j].setMinPulseWidth(MIN_PULSE_US);
    stp[j].setPinsInverted(DIR_INVERT[j], false, false);
    stp[j].setMaxSpeed(MAX_SPEED_SPS[j]);
    stp[j].setAcceleration(ACCEL_SPS2[j]);
    stp[j].setSpeed(0.0f);
  }

  // Jaw servo
  jaw_servo.attach(JAW_PIN);
  jaw_servo.write((int)JAW_OPEN_DEG);

  // Reset state
  robot_state = ROBOT_IDLE;
  stopAllSpeeds();
  for (uint8_t j = 0; j < NS; j++) {
    v_cur_deg_s[j] = 0.0f;
    at_target[j] = true;
    fb_joint_deg[j] = 0.0f;
    tgt_ros_rad[j] = 0.0f;
  }
  base_track_init = false;

  wristPollPresent();

  init_messages();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  agent_state = WAITING_AGENT;
}

// ===================== MAIN LOOP =====================
static uint32_t last_ping_ms = 0;

void loop() {
  // Always run steppers
  for (uint8_t j = 0; j < NS; j++) stp[j].runSpeed();

  // Encoder update and control tick when homed
  if (home_data.valid) {
    updateEncoders();
    controlTick();
  }

  // Agent state machine
  uint32_t now_ms = millis();

  switch (agent_state) {
    case WAITING_AGENT:
      // Ping agent every 500ms
      if ((uint32_t)(now_ms - last_ping_ms) >= 500) {
        last_ping_ms = now_ms;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          agent_state = AGENT_AVAILABLE;
        }
      }
      digitalWrite(LED_BUILTIN, LOW);
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        agent_state = AGENT_CONNECTED;
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // Spin executor
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

      // Ping agent every 200ms to detect disconnect
      if ((uint32_t)(now_ms - last_ping_ms) >= 200) {
        last_ping_ms = now_ms;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          agent_state = AGENT_DISCONNECTED;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      // Stop motors on disconnect
      stopAllSpeeds();
      for (uint8_t j = 0; j < NS; j++) v_cur_deg_s[j] = 0.0f;

      destroy_entities();
      digitalWrite(LED_BUILTIN, LOW);
      agent_state = WAITING_AGENT;
      break;
  }
}
