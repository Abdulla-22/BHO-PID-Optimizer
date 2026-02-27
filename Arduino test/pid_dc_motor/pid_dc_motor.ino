// =====================================
// DC Motor Speed PID Control (RPM) - CONTINUOUS RUN (DIR-FIXED)
// Board: Arduino Uno/Nano
// Encoder: D2 (A), D3 (B)
// Driver: ENA D9 (PWM), IN1 D8, IN2 D7
//
// Serial commands:
//   S20     -> setpoint RPM (always positive)
//   P15     -> Kp
//   I10     -> Ki
//   D0.5    -> Kd
//   Z       -> reset PID
//   F1/F0   -> direction (forward/reverse)  [resets PID to prevent PWM slam]
//   X       -> stop
//
// Serial Plotter:
//   t,rpm_raw,rpm_filt,pwm,low,high,setpoint
// Notes:
// - Encoder RPM can be negative depending on direction.
// - We flip measurement sign when in reverse so control always sees +RPM.
// =====================================

#include <Arduino.h>

#define ENA   9
#define IN1   8
#define IN2   7
#define ENC_A 2
#define ENC_B 3

#define CPR_MOTOR_4X 64.0f
#define GEAR_RATIO   131.25f

// 1 = gearbox output RPM, 0 = motor shaft RPM
#define RPM_AT_OUTPUT 1

#define SAMPLE_MS 50UL

const int PWM_MIN = 0;
const int PWM_MAX = 255;

// Plot anchors (Serial Plotter scale)
const float PLOT_MIN = 0.0f;
const float PLOT_MAX = 250.0f;

// ===== Setpoint (ALWAYS POSITIVE here) =====
float SETPOINT_RPM = 20.0f;

// ===== PID gains =====
float Kp = 15.0f;
float Ki = 10.0f;
float Kd = 0.5f;

// Direction
bool forwardDir = true;

// ===== PID state =====
static float integral  = 0.0f;
static float prevError = 0.0f;
static float dFiltered = 0.0f;

// Derivative filter
const float dAlpha = 0.15f;

// RPM filter (EMA)
static float rpmFilt = 0.0f;
const float rpmAlpha = 0.25f;

// Integral clamp
const float I_LIMIT = 200.0f;

// ===== Encoder count =====
volatile long encoderCount = 0;
static long lastCount = 0;

// ===== Time =====
static unsigned long startTime = 0;
static unsigned long lastSampleTime = 0;

// ===== Serial buffer =====
static char lineBuf[32];
static uint8_t lineLen = 0;

// ============================================================
// Fast encoder read (direct port read)
// D2=PD2, D3=PD3 on Uno/Nano
// ============================================================
static inline uint8_t readEncAB_bits() {
  return (PIND >> 2) & 0x03; // bit0=A(D2), bit1=B(D3)
}

// 4x quadrature decode (CHANGE on both pins)
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
void applyMotorPWM(int pwm, bool forward) {
  pwm = constrain(pwm, PWM_MIN, PWM_MAX);

  if (pwm == 0) {
    // coast
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    return;
  }

  if (forward) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, pwm);
}

void resetPID() {
  integral  = 0.0f;
  prevError = 0.0f;
  dFiltered = 0.0f;
  // rpmFilt keep as-is to avoid a large jump (optional)
}

// ============================================================
// RPM compute + filter
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
// PID -> PWM
// ============================================================
int computePWM(float rpmMeasForControl, float setpoint, float dt) {
  if (setpoint < 0.5f) {
    resetPID();
    return 0;
  }

  float error = setpoint - rpmMeasForControl;

  integral += error * dt;
  integral = constrain(integral, -I_LIMIT, I_LIMIT);

  float d = (error - prevError) / dt;
  dFiltered = dAlpha * d + (1.0f - dAlpha) * dFiltered;

  float u = (Kp * error) + (Ki * integral) + (Kd * dFiltered);
  int pwm = (int)lround(u);

  // saturation + anti-windup
  if (pwm > PWM_MAX) {
    pwm = PWM_MAX;
    if (error > 0) integral -= error * dt;
  } else if (pwm < PWM_MIN) {
    pwm = PWM_MIN;
    if (error < 0) integral -= error * dt;
  }

  prevError = error;
  return pwm;
}

// ============================================================
// Serial commands
// ============================================================
void handleCommand(const char* cmd) {
  if (!cmd || !cmd[0]) return;

  char c = toupper(cmd[0]);

  if (c == 'Z') {
    resetPID();
    Serial.println("PID reset");
    return;
  }

  if (c == 'X') {
    SETPOINT_RPM = 0.0f;
    resetPID();
    applyMotorPWM(0, forwardDir);
    Serial.println("STOP");
    return;
  }

  if (c == 'F') {
    int v = atoi(cmd + 1);
    forwardDir = (v != 0);

    // IMPORTANT: prevent PWM slam on direction change
    resetPID();
    applyMotorPWM(0, forwardDir);
    delay(50);

    Serial.print("Direction = ");
    Serial.println(forwardDir ? "FORWARD" : "REVERSE");
    return;
  }

  // commands with value
  float v = atof(cmd + 1);

  switch (c) {
    case 'S':
      // keep setpoint positive; direction is controlled by F
      if (v < 0) v = -v;
      SETPOINT_RPM = v;
      resetPID();
      Serial.print("SETPOINT_RPM = ");
      Serial.println(SETPOINT_RPM, 2);
      break;

    case 'P':
      Kp = v;
      Serial.print("Kp = ");
      Serial.println(Kp, 4);
      break;

    case 'I':
      Ki = v;
      Serial.print("Ki = ");
      Serial.println(Ki, 4);
      break;

    case 'D':
      Kd = v;
      Serial.print("Kd = ");
      Serial.println(Kd, 4);
      break;

    default:
      // if just a number, treat as setpoint
      if ((cmd[0] >= '0' && cmd[0] <= '9') || cmd[0] == '.' || cmd[0] == '-') {
        float sp = atof(cmd);
        if (sp < 0) sp = -sp;
        SETPOINT_RPM = sp;
        resetPID();
        Serial.print("SETPOINT_RPM = ");
        Serial.println(SETPOINT_RPM, 2);
      } else {
        Serial.print("Unknown cmd: ");
        Serial.println(cmd);
      }
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();

    if (ch == '\r' || ch == '\n') {
      if (lineLen > 0) {
        lineBuf[lineLen] = '\0';
        handleCommand(lineBuf);
        lineLen = 0;
      }
    } else {
      if (lineLen < sizeof(lineBuf) - 1) {
        lineBuf[lineLen++] = ch;
      }
    }
  }
}

// ============================================================
// Telemetry
// ============================================================
void printCSV(float t, float rpmRaw, float rpmF, int pwm) {
  Serial.print(t, 3); Serial.print(",");
  Serial.print(rpmRaw, 2); Serial.print(",");
  Serial.print(rpmF, 2); Serial.print(",");
  Serial.print(pwm); Serial.print(",");
  Serial.print(PLOT_MIN, 2); Serial.print(",");
  Serial.print(PLOT_MAX, 2); Serial.print(",");
  Serial.println(SETPOINT_RPM, 2);
}

// ============================================================
// Setup / Loop
// ============================================================
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  Serial.begin(115200);

  Serial.println("Commands: Sxx Pxx Ixx Dxx Z F1/F0 X");
  Serial.println("Plot: t,rpm_raw,rpm_filt,pwm,low,high,setpoint");

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  startTime = millis();
  lastSampleTime = startTime;

  applyMotorPWM(0, forwardDir);
}

void loop() {
  pollSerial();

  unsigned long now = millis();

  if (now - lastSampleTime >= SAMPLE_MS) {
    long count;
    noInterrupts();
    count = encoderCount;
    interrupts();

    long delta = count - lastCount;
    float dt = (float)SAMPLE_MS / 1000.0f;

    float rpmRaw = computeRPM(delta, dt);
    float rpmF   = filterRPM(rpmRaw);

    // KEY FIX: make measurement sign match commanded direction
    float rpmForControl = forwardDir ? rpmF : -rpmF;

    int pwmCmd = computePWM(rpmForControl, SETPOINT_RPM, dt);

    applyMotorPWM(pwmCmd, forwardDir);

    float t = (now - startTime) / 1000.0f;
    printCSV(t, rpmRaw, rpmF, pwmCmd);

    lastCount = count;
    lastSampleTime = now;
  }
}