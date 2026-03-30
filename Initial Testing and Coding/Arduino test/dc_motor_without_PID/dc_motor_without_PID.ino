// =====================================
// DC Motor Speed Control (NO PID)
// Setpoint = RPM (converted to PWM linearly)
// =====================================

#include <Arduino.h>

#define ENA   9
#define IN1   8
#define IN2   7
#define ENC_A 2
#define ENC_B 3

#define CPR_MOTOR_4X 64.0f
#define GEAR_RATIO   131.25f
#define RPM_AT_OUTPUT 1

#define SAMPLE_MS 50UL

const int PWM_MIN = 0;
const int PWM_MAX = 255;

const float PLOT_MIN = 0.0f;
const float PLOT_MAX = 250.0f;

// ===== USER SETPOINT (RPM) =====
float SETPOINT_RPM = 0.0f;

// ===== Calibration =====
// عدل هذا حسب موتورتك
// إذا 255 PWM ≈ 100 RPM
// kFF = 255 / 100 = 2.55
float kFF = 2.55f;  

bool forwardDir = true;

volatile long encoderCount = 0;
long lastCount = 0;

unsigned long lastSampleTime = 0;
unsigned long startTime = 0;

static float rpmFilt = 0.0f;
const float rpmAlpha = 0.25f;

// ============================================================
// Fast encoder
// ============================================================
static inline uint8_t readEncAB_bits() {
  return (PIND >> 2) & 0x03;
}

void isrA() {
  uint8_t ab = readEncAB_bits();
  bool A = ab & 0x01;
  bool B = ab & 0x02;
  encoderCount += (A == B) ? +1 : -1;
}

void isrB() {
  uint8_t ab = readEncAB_bits();
  bool A = ab & 0x01;
  bool B = ab & 0x02;
  encoderCount += (A != B) ? +1 : -1;
}

// ============================================================
// Motor
// ============================================================
void applyMotorPWM(int pwm) {
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);

  if (pwm == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    return;
  }

  if (forwardDir) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, pwm);
}

// ============================================================
// RPM
// ============================================================
float computeRPM(long delta, float dt) {
  float cpr = CPR_MOTOR_4X;
#if RPM_AT_OUTPUT
  cpr *= GEAR_RATIO;
#endif
  return ((float)delta * 60.0f) / (cpr * dt);
}

float filterRPM(float rpmRaw) {
  rpmFilt = rpmAlpha * rpmRaw + (1.0f - rpmAlpha) * rpmFilt;
  return rpmFilt;
}

// ============================================================
// Serial
// ============================================================
void handleCommand(String cmd) {

  cmd.trim();
  if (cmd.length() == 0) return;

  char c = toupper(cmd[0]);

  if (c == 'S') {
    SETPOINT_RPM = abs(cmd.substring(1).toFloat());
    Serial.print("SETPOINT_RPM = ");
    Serial.println(SETPOINT_RPM);
    return;
  }

  if (c == 'F') {
    forwardDir = cmd.substring(1).toInt() != 0;
    Serial.print("Direction = ");
    Serial.println(forwardDir ? "FORWARD" : "REVERSE");
    return;
  }

  if (c == 'X') {
    SETPOINT_RPM = 0;
    applyMotorPWM(0);
    Serial.println("STOP");
    return;
  }

  // If just number → treat as setpoint
  if (isDigit(cmd[0]) || cmd[0] == '-') {
    SETPOINT_RPM = abs(cmd.toFloat());
    Serial.print("SETPOINT_RPM = ");
    Serial.println(SETPOINT_RPM);
  }
}

// ============================================================
// Setup
// ============================================================
void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  startTime = millis();
  lastSampleTime = startTime;

  applyMotorPWM(0);

  Serial.println("Commands:");
  Serial.println("S20 -> 20 RPM");
  Serial.println("F1/F0 -> Direction");
  Serial.println("X -> Stop");
}

// ============================================================
// Loop
// ============================================================
void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }

  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_MS) {

    long count;
    noInterrupts();
    count = encoderCount;
    interrupts();

    long delta = count - lastCount;
    float dt = SAMPLE_MS / 1000.0f;

    float rpmRaw = computeRPM(delta, dt);
    float rpmF = filterRPM(rpmRaw);

    // ===== Feedforward only (NO PID) =====
    int pwmCmd = (int)(kFF * SETPOINT_RPM);
    pwmCmd = constrain(pwmCmd, PWM_MIN, PWM_MAX);

    applyMotorPWM(pwmCmd);

    float t = (now - startTime) / 1000.0f;

    Serial.print(t,3); Serial.print(",");
    Serial.print(rpmF,2); Serial.print(",");
    Serial.println(pwmCmd);

    lastCount = count;
    lastSampleTime = now;
  }
}