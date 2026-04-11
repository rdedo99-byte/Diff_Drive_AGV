#include <Arduino.h>

const int PIN_ENA = 18; const int PIN_IN1 = 19; const int PIN_IN2 = 21;
const int PIN_ENB = 23; const int PIN_IN3 = 22; const int PIN_IN4 = 5;
const int PIN_L_ENC_A = 33; const int PIN_L_ENC_B = 32;
const int PIN_R_ENC_A = 34; const int PIN_R_ENC_B = 35;

volatile long left_ticks = 0;
volatile long right_ticks = 0;
String inputBuffer = "";

//setting up interrupts for encoders, using RISING edge 
void IRAM_ATTR readEncoderSX() { (digitalRead(PIN_L_ENC_B) == LOW) ? left_ticks++ : left_ticks--; }
void IRAM_ATTR readEncoderDX() { (digitalRead(PIN_R_ENC_B) == LOW) ? right_ticks-- : right_ticks++; }

// Function to set motor speed and direction based on PWM values
void setMotorSpeed(int pwm_l, int pwm_r) {
  digitalWrite(PIN_IN1, (pwm_l > 0));
  digitalWrite(PIN_IN2, (pwm_l < 0));
  analogWrite(PIN_ENA, constrain(abs(pwm_l), 0, 255));
  digitalWrite(PIN_IN3, (pwm_r > 0));
  digitalWrite(PIN_IN4, (pwm_r < 0));
  analogWrite(PIN_ENB, constrain(abs(pwm_r), 0, 255));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_L_ENC_A, INPUT_PULLUP); pinMode(PIN_L_ENC_B, INPUT_PULLUP);
  pinMode(PIN_R_ENC_A, INPUT_PULLUP); pinMode(PIN_R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_L_ENC_A), readEncoderSX, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_R_ENC_A), readEncoderDX, RISING);
  setMotorSpeed(0, 0);
}

void loop() {
  // Read serial input and parse motor commands
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      int l_idx = inputBuffer.indexOf('L');
      int r_idx = inputBuffer.indexOf('R');
      
      if (l_idx != -1 && r_idx != -1) {
        int pwm_l = inputBuffer.substring(l_idx + 1, r_idx).toInt();
        int pwm_r = inputBuffer.substring(r_idx + 1).toInt();
        setMotorSpeed(pwm_l, pwm_r);
      }
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
  // Send encoder ticks back to the serial monitor every 30ms
  static unsigned long last_send = 0;
  if (millis() - last_send >= 30) {
    Serial.printf("ENC %ld %ld\n", left_ticks, right_ticks);
    last_send = millis();
  }
}