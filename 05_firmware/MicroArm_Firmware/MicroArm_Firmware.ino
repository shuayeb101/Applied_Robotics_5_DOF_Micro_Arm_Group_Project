/*
  CT6056 Applied Robotics - 5-DOF Micro Arm
  Arduino Nano + PCA9685 firmware                                   v1.0.0

  Replaces the supplied "Arduino Code.cpp" baseline. Main changes:
    - Drives the PCA9685 directly over I2C (Wire only, no extra library).
    - Safe start: every output is OFF (servos limp) until HOME is sent.
    - Non-blocking motion: all joints move together at SPEED deg/s, and
      STOP / STATUS are still handled mid-move.
    - Per-joint calibration table: pulse range, direction, limits, HOME.
    - Line protocol with ACK / DONE / ERR / STATUS responses, optional
      sequence ids, and strict validation. A bad command never moves the arm.
    - XY command: two-link planar IK (shoulder + elbow) with reach, joint-limit,
      floor and path checks. It never swaps branch or clamps silently.
    - Fault handling: I2C loss and comm watchdog timeout -> FAULT,
      cleared only by RESET_FAULT (never resumes the old command).

  Wiring (Nano):  A4 -> SDA, A5 -> SCL, 5V -> VCC, GND -> GND
                  External 5-6 V servo supply -> PCA9685 V+ terminal
                  Servos on PCA9685 channels 0..4
  Serial: 115200 baud, commands end with newline (\n). Send HELP for a list.
*/

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define FW_VERSION "1.0.0"
const long BAUD = 115200;

// =====================================================================
//  CALIBRATION - EDIT THIS BLOCK DURING TASK 4 (one servo at a time)
// =====================================================================
//  Angles are "servo degrees" 0..180 (same scale as the baseline / GUI).
//  minUs / maxUs : pulse width (microseconds) at servo 0 deg and 180 deg.
//                  SG90-class servos are usually ~500..2400 us. Start narrower
//                  (e.g. 600..2300) and widen only if the joint is not binding.
//  dir           : +1 normal, -1 to reverse a joint whose horn is mounted mirrored.
//  offsetDeg     : trim added after dir (fix horn spline misalignment).
//  lo / hi       : soft limits in servo degrees (must be inside 0..180).
//  home          : HOME pose (must be inside lo..hi).
struct Joint {
  const char *name;
  uint8_t ch;
  int16_t minUs, maxUs;
  int8_t dir;
  float offsetDeg;
  float lo, hi, home;
};

const uint8_t NJ = 5;
Joint J[NJ] = {
  //  name    ch  minUs maxUs dir offset  lo    hi    home
  { "BASE",   0,  500,  2400, +1, 0.0,    0.0,  180.0, 90.0 },
  { "SWING",  1,  500,  2400, +1, 0.0,   10.0,  170.0, 90.0 },
  { "ARM1",   2,  500,  2400, +1, 0.0,    0.0,  180.0, 90.0 },
  { "ARM2",   3,  500,  2400, +1, 0.0,    0.0,  180.0, 90.0 },
  { "GRIP",   4,  500,  2400, +1, 0.0,   30.0,  150.0, 90.0 },
};

// Order joints are energised during the first HOME (outer joints first,
// so the heavy shoulder is not driven while the forearm is flopping around).
const uint8_t HOME_ORDER[NJ] = { 3, 2, 1, 0, 4 };
const unsigned long HOME_STEP_MS = 700;   // pause between energising joints

// ---- Kinematic model (planar, shoulder pivot = origin, x forward, y up) ----
// Link lengths measured from the supplied STLs (pivot-to-pivot), mm.
const float L1 = 49.6;        // shoulder (SWING, ch1) -> elbow (ARM1, ch2)
const float L2 = 49.6;        // elbow -> wrist (ARM2, ch3)
const float L3 = 73.0;        // wrist pivot -> gripper jaw tips (approx.)
const float SHOULDER_H = 44.0; // shoulder axis height above the table (approx.)
// Servo angle at which each link is straight/horizontal, and the sign that
// maps +servo degrees to +geometric angle (anticlockwise = lifting).
// VERIFY THESE ON THE REAL ARM - they depend on how the horns were fitted.
const float KIN_ZERO[3] = { 90.0, 90.0, 90.0 };  // for ch1, ch2, ch3
const int8_t KIN_SIGN[3] = { +1, +1, +1 };
// Workspace / safety envelope
const float R_MIN = 15.0;        // inner reach boundary for XY, mm
const float FLOOR_CLEAR = 8.0;   // min height above table for elbow/wrist/tip, mm
const float BASE_KEEP_X = 20.0;  // tip may not come closer than this...
const float BASE_KEEP_Y = 0.0;   // ...while below the shoulder axis (base body)

// Motion
const float SPEED_MIN = 5.0, SPEED_MAX = 180.0;  // deg/s
float speedDps = 60.0;
const unsigned long MOTION_TICK_MS = 20;

// Communication watchdog (0 = off). The GUI enables it automatically.
unsigned long watchdogMs = 0;
const unsigned long WD_MIN = 300, WD_MAX = 10000;

// PCA9685
const uint8_t PCA_ADDR = 0x40;
const float PCA_OSC_HZ = 27000000.0; // many PCA9685 boards run ~25-27 MHz; trim with a scope
const float PWM_FREQ = 50.0;
// =====================================================================

enum State : uint8_t { ST_INIT, ST_IDLE, ST_MOVING, ST_HOMING, ST_HOLD, ST_FAULT };
const char *STATE_NAME[] = { "INIT", "IDLE", "MOVING", "HOMING", "HOLD", "FAULT" };
State state = ST_INIT;

float pos[NJ], tgt[NJ];
bool energised[NJ];
float actualFreq = 50.0;

char lastErr[24] = "NONE";
char faultCode[20] = "";
char activeSeq[10] = "";   // seq id of the command that started the current motion
char activeCmd[6] = "";
char curSeq[10] = "";      // seq id of the line being processed

uint8_t homeIdx = 0;
unsigned long homeT = 0;
unsigned long lastRx = 0, lastTick = 0, lastI2cCheck = 0;
unsigned long rxCount = 0, errCount = 0;

// ---------------------------------------------------------------------
//  PCA9685 low-level
// ---------------------------------------------------------------------
bool pcaWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool pcaPresent() {
  Wire.beginTransmission(PCA_ADDR);
  return Wire.endTransmission() == 0;
}
bool pcaSetCounts(uint8_t ch, uint16_t on, uint16_t off) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(0x06 + 4 * ch);
  Wire.write(on & 0xFF); Wire.write(on >> 8);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  return Wire.endTransmission() == 0;
}
bool pcaChannelOff(uint8_t ch) { return pcaSetCounts(ch, 0, 0x1000); } // full-OFF bit
bool pcaAllOff() { return pcaWrite8(0xFC, 0) && pcaWrite8(0xFD, 0x10); }  // ALL_LED full-OFF

bool pcaInit() {
  if (!pcaPresent()) return false;
  float pre = PCA_OSC_HZ / (4096.0 * PWM_FREQ) - 1.0;
  uint8_t prescale = (uint8_t)constrain((int)(pre + 0.5), 3, 255);
  actualFreq = PCA_OSC_HZ / (4096.0 * (prescale + 1));
  bool ok = pcaWrite8(0x00, 0x10);        // MODE1: sleep (needed to set prescale)
  ok &= pcaWrite8(0xFE, prescale);        // PRE_SCALE
  ok &= pcaWrite8(0x01, 0x04);            // MODE2: totem-pole outputs
  ok &= pcaAllOff();                      // safe output state before waking
  ok &= pcaWrite8(0x00, 0x20);            // MODE1: wake, auto-increment
  delay(2);
  ok &= pcaWrite8(0x00, 0xA0);            // restart PWM
  return ok;
}

// servo degrees (0..180, already limit-checked) -> PCA9685 counts
uint16_t angleToCounts(uint8_t j, float a) {
  float s = (J[j].dir > 0 ? a : 180.0 - a) + J[j].offsetDeg;
  s = constrain(s, 0.0, 180.0);                       // hardware protection only
  float us = J[j].minUs + (J[j].maxUs - J[j].minUs) * s / 180.0;
  return (uint16_t)(us * actualFreq * 4096.0 / 1000000.0 + 0.5);
}

bool writeJoint(uint8_t j) {
  return pcaSetCounts(J[j].ch, 0, angleToCounts(j, pos[j]));
}

// ---------------------------------------------------------------------
//  Output helpers
// ---------------------------------------------------------------------
void printSeq(const char *s) {
  if (s[0]) { Serial.print(' '); Serial.print('#'); Serial.print(s); }
}
void reply(const __FlashStringHelper *kind, const char *cmd) {
  Serial.print(kind); printSeq(curSeq); Serial.print(' '); Serial.print(cmd);
}
void err(const char *code, const __FlashStringHelper *msg) {
  Serial.print(F("ERR")); printSeq(curSeq); Serial.print(' ');
  Serial.print(code); Serial.print(' '); Serial.println(msg);
  strncpy(lastErr, code, sizeof(lastErr) - 1);
  errCount++;
}
void printAngles(const float *a) {
  for (uint8_t j = 0; j < NJ; j++) {
    if (j) Serial.print(',');
    Serial.print(a[j], 1);
  }
}
void printStatus() {
  Serial.print(F("STATUS")); printSeq(curSeq);
  Serial.print(F(" state=")); Serial.print(STATE_NAME[state]);
  Serial.print(F(" pos=")); printAngles(pos);
  Serial.print(F(" tgt=")); printAngles(tgt);
  Serial.print(F(" en="));
  for (uint8_t j = 0; j < NJ; j++) Serial.print(energised[j] ? '1' : '0');
  Serial.print(F(" spd=")); Serial.print(speedDps, 0);
  Serial.print(F(" wd=")); Serial.print(watchdogMs);
  Serial.print(F(" err=")); Serial.print(lastErr);
  Serial.print(F(" fault=")); Serial.print(faultCode[0] ? faultCode : "NONE");
  Serial.print(F(" rx=")); Serial.print(rxCount);
  Serial.print(F(" t=")); Serial.println(millis());
}
void event(const char *what) {
  Serial.print(F("EVENT ")); Serial.print(what);
  Serial.print(F(" state=")); Serial.println(STATE_NAME[state]);
}

// ---------------------------------------------------------------------
//  Kinematics (planar, joints ch1..ch3)
// ---------------------------------------------------------------------
float geomRad(uint8_t k, float servoDeg) {           // k: 0=SWING,1=ARM1,2=ARM2
  return KIN_SIGN[k] * (servoDeg - KIN_ZERO[k]) * (float)DEG_TO_RAD;
}
float servoDeg(uint8_t k, float rad) {
  return KIN_ZERO[k] + KIN_SIGN[k] * rad * (float)RAD_TO_DEG;
}
// Checks elbow, wrist and gripper tip against the floor and base keep-out.
// Returns NULL if safe, otherwise a short reason.
const char *poseUnsafe(const float *a) {
  float t1 = geomRad(0, a[1]);
  float t12 = t1 + geomRad(1, a[2]);
  float t123 = t12 + geomRad(2, a[3]);
  float ex = L1 * cos(t1),          ey = L1 * sin(t1);
  float wx = ex + L2 * cos(t12),    wy = ey + L2 * sin(t12);
  float px = wx + L3 * cos(t123),   py = wy + L3 * sin(t123);
  float floorY = -SHOULDER_H + FLOOR_CLEAR;
  if (ey < floorY || wy < floorY || py < floorY) return "FLOOR";
  if (px < BASE_KEEP_X && py < BASE_KEEP_Y) return "BASE_ZONE";
  return NULL;
}
// Simulates the real motion profile (each joint at speedDps, finishing at
// different times) from pos[] to newTgt[] and checks intermediate poses.
const char *pathUnsafe(const float *newTgt) {
  float maxD = 0;
  for (uint8_t j = 0; j < NJ; j++) maxD = max(maxD, fabs(newTgt[j] - pos[j]));
  const uint8_t N = 16;
  float a[NJ];
  for (uint8_t i = 1; i <= N; i++) {
    float travelled = maxD * i / N;
    for (uint8_t j = 0; j < NJ; j++) {
      float d = newTgt[j] - pos[j];
      a[j] = pos[j] + (fabs(d) <= travelled ? d : (d > 0 ? travelled : -travelled));
    }
    const char *r = poseUnsafe(a);
    if (r) return r;
  }
  return NULL;
}

// ---------------------------------------------------------------------
//  Parsing helpers
// ---------------------------------------------------------------------
bool parseNum(const char *tok, float &out) {
  if (!tok || !*tok) return false;
  char *end;
  double v = strtod(tok, &end);
  if (*end != '\0' || isnan(v) || isinf(v)) return false;
  out = (float)v;
  return true;
}
int parseJoint(const char *tok) {
  if (!tok) return -1;
  if (tok[0] >= '0' && tok[0] <= '9' && tok[1] == '\0') {
    int j = tok[0] - '0';
    return j < NJ ? j : -1;
  }
  for (uint8_t j = 0; j < NJ; j++) if (strcmp(tok, J[j].name) == 0) return j;
  return -1;
}
bool allEnergised() {
  for (uint8_t j = 0; j < NJ; j++) if (!energised[j]) return false;
  return true;
}
bool canCommandMotion() {
  if (state == ST_FAULT) { err("IN_FAULT", F("send RESET_FAULT after fixing the cause")); return false; }
  if (state == ST_INIT)  { err("NOT_HOMED", F("send HOME first (outputs are off)")); return false; }
  if (state == ST_HOMING){ err("BUSY_HOMING", F("wait for DONE HOME or send STOP")); return false; }
  if (!allEnergised())   { err("NOT_HOMED", F("some joints are still off - send HOME")); return false; }
  return true;
}

// Accept a new full target set (already validated) and start moving.
void startMove(const float *newTgt, const char *cmd) {
  if (state == ST_MOVING && activeCmd[0]) {        // latest command wins - say so
    Serial.print(F("EVENT SUPERSEDED")); printSeq(activeSeq);
    Serial.print(' '); Serial.println(activeCmd);
  }
  for (uint8_t j = 0; j < NJ; j++) tgt[j] = newTgt[j];
  strncpy(activeSeq, curSeq, sizeof(activeSeq) - 1); activeSeq[sizeof(activeSeq) - 1] = 0;
  strncpy(activeCmd, cmd, sizeof(activeCmd) - 1);  activeCmd[sizeof(activeCmd) - 1] = 0;
  state = ST_MOVING;
}
float etaSeconds(const float *newTgt) {
  float m = 0;
  for (uint8_t j = 0; j < NJ; j++) m = max(m, fabs(newTgt[j] - pos[j]));
  return m / speedDps;
}

// ---------------------------------------------------------------------
//  Command handlers
// ---------------------------------------------------------------------
void cmdS(char *a1, char *a2, char *extra) {
  int j = parseJoint(a1);
  if (j < 0) { err("BAD_JOINT", F("joint must be 0-4 or BASE/SWING/ARM1/ARM2/GRIP")); return; }
  float a;
  if (!a2) { err("MISSING_FIELD", F("usage: S <joint> <angle_deg>")); return; }
  if (!parseNum(a2, a)) { err("BAD_NUMBER", F("angle is not a number")); return; }
  if (extra) { err("EXTRA_FIELD", F("usage: S <joint> <angle_deg>")); return; }
  if (a < J[j].lo || a > J[j].hi) {
    Serial.print(F("ERR")); printSeq(curSeq); Serial.print(F(" OUT_OF_RANGE joint="));
    Serial.print(J[j].name); Serial.print(F(" angle=")); Serial.print(a, 1);
    Serial.print(F(" limits=")); Serial.print(J[j].lo, 0); Serial.print(F(".."));
    Serial.println(J[j].hi, 0);
    strcpy(lastErr, "OUT_OF_RANGE"); errCount++;
    return;
  }
  if (!canCommandMotion()) return;
  float nt[NJ];
  for (uint8_t k = 0; k < NJ; k++) nt[k] = tgt[k];
  nt[j] = a;
  const char *why = pathUnsafe(nt);
  if (why) {
    Serial.print(F("ERR")); printSeq(curSeq); Serial.print(F(" UNSAFE_PATH reason="));
    Serial.println(why);
    strcpy(lastErr, "UNSAFE_PATH"); errCount++;
    return;
  }
  reply(F("ACK"), "S");
  Serial.print(F(" joint=")); Serial.print(J[j].name);
  Serial.print(F(" target=")); Serial.print(a, 1);
  Serial.print(F(" eta=")); Serial.print(etaSeconds(nt), 1); Serial.println('s');
  startMove(nt, "S");
}

void cmdXY(char *a1, char *a2, char *a3, char *extra) {
  float x, y;
  if (!a1 || !a2) { err("MISSING_FIELD", F("usage: XY <x_mm> <y_mm> [UP|DOWN]")); return; }
  if (!parseNum(a1, x)) { err("BAD_NUMBER", F("x is not a number")); return; }
  if (!parseNum(a2, y)) { err("BAD_NUMBER", F("y is not a number")); return; }
  bool up = true;
  if (a3) {
    if (strcmp(a3, "UP") == 0) up = true;
    else if (strcmp(a3, "DOWN") == 0) up = false;
    else { err("BAD_BRANCH", F("branch must be UP or DOWN")); return; }
  }
  if (extra) { err("EXTRA_FIELD", F("usage: XY <x_mm> <y_mm> [UP|DOWN]")); return; }

  float r = sqrt(x * x + y * y);
  if (r > L1 + L2 || r < R_MIN) {
    Serial.print(F("ERR")); printSeq(curSeq); Serial.print(F(" UNREACHABLE r="));
    Serial.print(r, 1); Serial.print(F("mm reach=")); Serial.print(R_MIN, 0);
    Serial.print(F("..")); Serial.println(L1 + L2, 1);
    strcpy(lastErr, "UNREACHABLE"); errCount++;
    return;
  }
  float c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  c2 = constrain(c2, -1.0, 1.0);          // only rounding error can reach here
  float t2 = acos(c2);
  if (up) t2 = -t2;                       // UP = elbow above the shoulder-wrist line
  float t1 = atan2(y, x) - atan2(L2 * sin(t2), L1 + L2 * cos(t2));
  float aSw = servoDeg(0, t1), a2s = servoDeg(1, t2);

  // forward-kinematics residual (evidence for the KIN table)
  float fx = L1 * cos(t1) + L2 * cos(t1 + t2), fy = L1 * sin(t1) + L2 * sin(t1 + t2);
  float res = sqrt((fx - x) * (fx - x) + (fy - y) * (fy - y));

  if (aSw < J[1].lo || aSw > J[1].hi || a2s < J[2].lo || a2s > J[2].hi) {
    Serial.print(F("ERR")); printSeq(curSeq); Serial.print(F(" JOINT_LIMIT branch="));
    Serial.print(up ? "UP" : "DOWN"); Serial.print(F(" SWING=")); Serial.print(aSw, 1);
    Serial.print(F(" ARM1=")); Serial.print(a2s, 1);
    Serial.println(F(" (no automatic branch change - try the other branch explicitly)"));
    strcpy(lastErr, "JOINT_LIMIT"); errCount++;
    return;
  }
  if (!canCommandMotion()) return;
  float nt[NJ];
  for (uint8_t k = 0; k < NJ; k++) nt[k] = tgt[k];
  nt[1] = aSw; nt[2] = a2s;
  const char *why = pathUnsafe(nt);
  if (why) {
    Serial.print(F("ERR")); printSeq(curSeq); Serial.print(F(" UNSAFE_PATH reason="));
    Serial.println(why);
    strcpy(lastErr, "UNSAFE_PATH"); errCount++;
    return;
  }
  reply(F("ACK"), "XY");
  Serial.print(F(" x=")); Serial.print(x, 1); Serial.print(F(" y=")); Serial.print(y, 1);
  Serial.print(F(" branch=")); Serial.print(up ? "UP" : "DOWN");
  Serial.print(F(" SWING=")); Serial.print(aSw, 1);
  Serial.print(F(" ARM1=")); Serial.print(a2s, 1);
  Serial.print(F(" residual=")); Serial.print(res, 3); Serial.print(F("mm eta="));
  Serial.print(etaSeconds(nt), 1); Serial.println('s');
  startMove(nt, "XY");
}

void cmdHome() {
  if (state == ST_FAULT) { err("IN_FAULT", F("send RESET_FAULT first")); return; }
  if (state == ST_HOMING) { err("BUSY_HOMING", F("already homing")); return; }
  for (uint8_t j = 0; j < NJ; j++) tgt[j] = J[j].home;
  homeIdx = 0; homeT = 0;
  strncpy(activeSeq, curSeq, sizeof(activeSeq) - 1); activeSeq[sizeof(activeSeq) - 1] = 0;
  strcpy(activeCmd, "HOME");
  state = ST_HOMING;
  reply(F("ACK"), "HOME");
  Serial.println(F(" (unpowered joints are energised one at a time at HOME)"));
}

void cmdStop() {
  // Highest priority: freeze every target at the current position.
  for (uint8_t j = 0; j < NJ; j++) tgt[j] = pos[j];
  if (state == ST_MOVING || state == ST_HOMING || state == ST_IDLE || state == ST_HOLD) {
    bool wasHoming = state == ST_HOMING;
    if (wasHoming) homeIdx = NJ; // abandon remaining energise steps
    state = allEnergised() ? ST_HOLD : ST_INIT;
  }
  activeSeq[0] = 0; activeCmd[0] = 0;
  reply(F("ACK"), "STOP");
  Serial.print(F(" state=")); Serial.println(STATE_NAME[state]);
}

void enterFault(const char *code) {
  for (uint8_t j = 0; j < NJ; j++) tgt[j] = pos[j];
  strncpy(faultCode, code, sizeof(faultCode) - 1);
  strncpy(lastErr, code, sizeof(lastErr) - 1);
  state = ST_FAULT;
  activeSeq[0] = 0; activeCmd[0] = 0;
  Serial.print(F("EVENT FAULT code=")); Serial.print(code);
  Serial.println(F(" state=FAULT (motion frozen; send RESET_FAULT)"));
}

void cmdResetFault() {
  if (state != ST_FAULT) { err("NO_FAULT", F("not in FAULT")); return; }
  if (!pcaPresent()) { err("I2C_STILL_DOWN", F("PCA9685 not answering at 0x40 - check SDA/SCL/VCC")); return; }
  if (strcmp(faultCode, "I2C_LOST") == 0) {
    // Driver may have reset: re-initialise with outputs OFF and require HOME.
    if (!pcaInit()) { err("I2C_INIT_FAIL", F("PCA9685 init failed")); return; }
    for (uint8_t j = 0; j < NJ; j++) energised[j] = false;
    state = ST_INIT;
  } else {
    state = allEnergised() ? ST_HOLD : ST_INIT;  // outputs hold the frozen position
  }
  faultCode[0] = 0;
  strcpy(lastErr, "NONE");
  reply(F("ACK"), "RESET_FAULT");
  Serial.print(F(" state=")); Serial.print(STATE_NAME[state]);
  Serial.println(F(" (previous command NOT resumed)"));
}

void cmdSpeed(char *a1, char *extra) {
  float v;
  if (!a1) { err("MISSING_FIELD", F("usage: SPEED <deg_per_s 5..180>")); return; }
  if (!parseNum(a1, v)) { err("BAD_NUMBER", F("speed is not a number")); return; }
  if (extra) { err("EXTRA_FIELD", F("usage: SPEED <deg_per_s>")); return; }
  if (v < SPEED_MIN || v > SPEED_MAX) { err("OUT_OF_RANGE", F("speed must be 5..180 deg/s")); return; }
  speedDps = v;
  reply(F("ACK"), "SPEED");
  Serial.print(F(" speed=")); Serial.print(speedDps, 0); Serial.println(F("deg/s"));
}

void cmdWatchdog(char *a1, char *extra) {
  float v;
  if (!a1 || !parseNum(a1, v) || extra) { err("BAD_NUMBER", F("usage: WATCHDOG <ms 300..10000 | 0=off>")); return; }
  if (v != 0 && (v < WD_MIN || v > WD_MAX)) { err("OUT_OF_RANGE", F("watchdog must be 0 or 300..10000 ms")); return; }
  watchdogMs = (unsigned long)v;
  reply(F("ACK"), "WATCHDOG");
  Serial.print(F(" ms=")); Serial.println(watchdogMs);
}

void cmdConfig() {
  for (uint8_t j = 0; j < NJ; j++) {
    Serial.print(F("CONFIG joint=")); Serial.print(j);
    Serial.print(F(" name=")); Serial.print(J[j].name);
    Serial.print(F(" ch=")); Serial.print(J[j].ch);
    Serial.print(F(" lo=")); Serial.print(J[j].lo, 1);
    Serial.print(F(" hi=")); Serial.print(J[j].hi, 1);
    Serial.print(F(" home=")); Serial.print(J[j].home, 1);
    Serial.print(F(" us=")); Serial.print(J[j].minUs); Serial.print(F(".."));
    Serial.print(J[j].maxUs);
    Serial.print(F(" dir=")); Serial.println(J[j].dir);
  }
  Serial.print(F("CONFIG kin L1=")); Serial.print(L1, 1);
  Serial.print(F(" L2=")); Serial.print(L2, 1);
  Serial.print(F(" rmin=")); Serial.print(R_MIN, 1);
  Serial.print(F(" pwm_hz=")); Serial.print(actualFreq, 2);
  Serial.print(F(" fw=")); Serial.println(F(FW_VERSION));
  Serial.println(F("CONFIG END"));
}

void cmdHelp() {
  Serial.println(F("HELP S <joint> <deg> | XY <x> <y> [UP|DOWN] | SPEED <deg/s> | HOME | STOP"));
  Serial.println(F("HELP STATUS | RESET_FAULT | CONFIG | WATCHDOG <ms|0> | PING | HELP"));
  Serial.println(F("HELP joints: 0 BASE, 1 SWING, 2 ARM1, 3 ARM2, 4 GRIP. Optional prefix #<id> is echoed."));
}

// ---------------------------------------------------------------------
//  Line handling
// ---------------------------------------------------------------------
const uint8_t LINE_MAX = 48;
char line[LINE_MAX + 1];
uint8_t lineLen = 0;
bool lineOverflow = false;

void handleLine(char *s) {
  rxCount++;
  lastRx = millis();
  curSeq[0] = 0;
  for (char *p = s; *p; p++) *p = toupper(*p);

  char *tok = strtok(s, " \t,");
  if (!tok) return;                              // blank line
  if (tok[0] == '#') {                           // optional sequence id
    strncpy(curSeq, tok + 1, sizeof(curSeq) - 1); curSeq[sizeof(curSeq) - 1] = 0;
    tok = strtok(NULL, " \t,");
    if (!tok) { err("MISSING_CMD", F("sequence id without a command")); return; }
  }
  char *a1 = strtok(NULL, " \t,");
  char *a2 = strtok(NULL, " \t,");
  char *a3 = strtok(NULL, " \t,");
  char *a4 = strtok(NULL, " \t,");

  // Baseline compatibility: "S1 90" (joint glued to the S)
  if (tok[0] == 'S' && isdigit(tok[1]) && tok[2] == '\0') {
    static char jbuf[2]; jbuf[0] = tok[1]; jbuf[1] = 0;
    cmdS(jbuf, a1, a2); return;
  }
  if      (!strcmp(tok, "S"))           cmdS(a1, a2, a3);
  else if (!strcmp(tok, "XY"))          cmdXY(a1, a2, a3, a4);
  else if (!strcmp(tok, "STOP") || !strcmp(tok, "X")) cmdStop();
  else if (!strcmp(tok, "SPEED"))       cmdSpeed(a1, a2);
  else if (!strcmp(tok, "WATCHDOG"))    cmdWatchdog(a1, a2);
  else if (a1)                          err("EXTRA_FIELD", F("this command takes no arguments"));
  else if (!strcmp(tok, "STATUS") || !strcmp(tok, "?")) printStatus();
  else if (!strcmp(tok, "HOME"))        cmdHome();
  else if (!strcmp(tok, "RESET_FAULT")) cmdResetFault();
  else if (!strcmp(tok, "CONFIG"))      cmdConfig();
  else if (!strcmp(tok, "HELP"))        cmdHelp();
  else if (!strcmp(tok, "PING"))        { reply(F("ACK"), "PING"); Serial.println(); }
  else err("UNKNOWN_CMD", F("send HELP for the command list"));
}
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineOverflow) {
        curSeq[0] = 0;
        err("LINE_TOO_LONG", F("max 48 characters; line discarded"));
        lastRx = millis();
      } else if (lineLen > 0) {
        line[lineLen] = 0;
        handleLine(line);
      }
      lineLen = 0; lineOverflow = false;
    } else if (lineLen < LINE_MAX) {
      line[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

// ---------------------------------------------------------------------
//  Motion / state update (runs every MOTION_TICK_MS, never blocks)
// ---------------------------------------------------------------------
void finishMotion() {
  Serial.print(F("DONE")); printSeq(activeSeq); Serial.print(' ');
  Serial.print(activeCmd[0] ? activeCmd : "MOVE");
  Serial.print(F(" pos=")); printAngles(pos); Serial.println();
  activeSeq[0] = 0; activeCmd[0] = 0;
  state = ST_IDLE;
}

void updateMotion(float dt) {
  bool allThere = true;
  float step = speedDps * dt;
  for (uint8_t j = 0; j < NJ; j++) {
    if (!energised[j]) { if (state == ST_HOMING) allThere = false; continue; }
    float d = tgt[j] - pos[j];
    if (fabs(d) > 0.001) {
      pos[j] += (fabs(d) <= step) ? d : (d > 0 ? step : -step);
      if (!writeJoint(j)) { enterFault("I2C_LOST"); return; }
      if (fabs(tgt[j] - pos[j]) > 0.001) allThere = false;
    }
  }
  if (state == ST_HOMING) {
    // energise unpowered joints one at a time, directly at HOME
    if (homeIdx < NJ && millis() - homeT >= HOME_STEP_MS) {
      uint8_t j = HOME_ORDER[homeIdx++];
      if (!energised[j]) {
        pos[j] = J[j].home;
        if (!writeJoint(j)) { enterFault("I2C_LOST"); return; }
        energised[j] = true;
        Serial.print(F("EVENT HOME_ENERGISED joint=")); Serial.println(J[j].name);
        homeT = millis();
      }
    }
    if (homeIdx >= NJ && allThere) {
      bool allEn = true;
      for (uint8_t j = 0; j < NJ; j++) allEn &= energised[j];
      if (allEn) finishMotion();
    }
  } else if (state == ST_MOVING && allThere) {
    finishMotion();
  }
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);   // never hang forever on a stuck I2C bus
#endif
  for (uint8_t j = 0; j < NJ; j++) { pos[j] = J[j].home; tgt[j] = J[j].home; energised[j] = false; }
  delay(50);
  if (!pcaInit()) {
    state = ST_FAULT;
    strcpy(faultCode, "I2C_LOST");
    strcpy(lastErr, "I2C_LOST");
  }
  Serial.print(F("READY MicroArm fw=")); Serial.print(F(FW_VERSION));
  Serial.print(F(" state=")); Serial.print(STATE_NAME[state]);
  Serial.println(state == ST_FAULT ? F(" (PCA9685 not found at 0x40)") : F(" (outputs OFF - send HOME)"));
  lastRx = lastTick = lastI2cCheck = millis();
}

void loop() {
  readSerial();
  unsigned long now = millis();

  // communication watchdog: only matters while something is moving
  if (watchdogMs && (state == ST_MOVING || state == ST_HOMING) && now - lastRx > watchdogMs) {
    enterFault("COMM_TIMEOUT");
  }
  // periodic I2C health check (motion writes also detect failures)
  if (now - lastI2cCheck >= 500) {
    lastI2cCheck = now;
    if (state != ST_FAULT && !pcaPresent()) enterFault("I2C_LOST");
  }
  if (now - lastTick >= MOTION_TICK_MS) {
    float dt = (now - lastTick) / 1000.0;
    lastTick = now;
    if (state == ST_MOVING || state == ST_HOMING) updateMotion(dt);
  }
}
