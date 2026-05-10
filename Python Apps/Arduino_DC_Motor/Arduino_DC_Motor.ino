#include <Arduino.h>

const int R_PWM = 9;
const int L_PWM = 10;
const int R_EN  = 8;
const int L_EN  = 7;
const int ENC_A = 2;
const int ENC_B = 3;

volatile long encoderValue = 0;
unsigned long lastPing = 0;

// 4X Quadrature decoding ISRs
void isr_A() {
  bool a = PIND & (1 << PD2);
  bool b = PIND & (1 << PD3);
  if (a == b) encoderValue++; else encoderValue--;
}

void isr_B() {
  bool a = PIND & (1 << PD2);
  bool b = PIND & (1 << PD3);
  if (a != b) encoderValue++; else encoderValue--;
}

void setup() {
  Serial.begin(115200);

  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_EN,  OUTPUT);
  pinMode(L_EN,  OUTPUT);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);

  // Keep default 490 Hz PWM on Timer1 (pins 9,10)
  // This matches the deadzone calibration in Python (PWM_DEADZONE=35)

  attachInterrupt(digitalPinToInterrupt(ENC_A), isr_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isr_B, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "GET") {
      noInterrupts();
      long val = encoderValue;
      interrupts();
      Serial.println(val);
      lastPing = millis();
    }
    else if (command.startsWith("PWM:")) {
      int separator = command.indexOf(',');
      if (separator != -1) {
        int dir = command.substring(4, separator).toInt();
        int val = command.substring(separator + 1).toInt();
        val = constrain(val, 0, 255);

        if (dir == 1) {          // Forward
          analogWrite(L_PWM, 0);
          analogWrite(R_PWM, val);
        } else {                 // Reverse
          analogWrite(R_PWM, 0);
          analogWrite(L_PWM, val);
        }
      }
      lastPing = millis();
    }
    else if (command == "RESET") {
      noInterrupts();
      encoderValue = 0;
      interrupts();
      analogWrite(R_PWM, 0);
      analogWrite(L_PWM, 0);
      Serial.println("OK");
      lastPing = millis();
    }
    else if (command == "PING") {
      Serial.println("PONG");
      lastPing = millis();
    }
  }

  // Safety watchdog — stop motor if no command for 1 second
  if (millis() - lastPing > 1000) {
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
  }
}
