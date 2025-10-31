// TCS3200/TCS230 + ESP32 + MG90S servo control + IR sensor (obstacle detection)
// Modified: Now IR sensor activates the color reading

#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>   // Library: ESP32Servo by Kevin Harrington (v3.0.9)

//// -------- Pins --------
const int PIN_S0  = 14;
const int PIN_S1  = 27;
const int PIN_S2  = 26;
const int PIN_S3  = 25;
const int PIN_OUT = 34;    // input-only, for TCS3200 OUT

const int SERVO_PIN = 18;  // MG90S signal pin (use external 5V; share GND)

// IR sensor pin
const int IR_SENSOR_PIN = 19; // IR sensor OUT connected to GPIO 19

//// -------- Clear channel scaling for brightness 0..1 --------
#define CLEAR_MAX_HZ 25000.0f   // tweak if your Clear freq is very different

//// -------- Servo setup --------
Servo gate;
const int SERVO_MIN_US = 500;    // typical 500–2500 µs
const int SERVO_MAX_US = 2400;

//// -------- Measurement timing --------
const uint32_t MEASURE_PERIOD_MS = 3000;  // every 3 seconds
uint32_t lastMeasureMs = 0;

//// -------- Low-level color read helpers --------
static uint32_t readFrequency() {
  uint32_t tHigh = pulseIn(PIN_OUT, HIGH, 250000); // 250 ms timeout
  uint32_t tLow  = pulseIn(PIN_OUT, LOW,  250000);
  if (tHigh == 0 || tLow == 0) return 0;
  float period_us = float(tHigh + tLow);
  if (period_us <= 0.0f) return 0;
  return (uint32_t)(1000000.0f / period_us); // Hz
}

static uint32_t readColorFreq(bool s2, bool s3, int samples = 12) {
  digitalWrite(PIN_S2, s2 ? HIGH : LOW);
  digitalWrite(PIN_S3, s3 ? HIGH : LOW);
  delayMicroseconds(300); // settle
  uint64_t sum = 0;
  int good = 0;
  for (int i = 0; i < samples; ++i) {
    uint32_t f = readFrequency();
    if (f > 0) { sum += f; ++good; }
  }
  return good ? (uint32_t)(sum / good) : 0;
}

//// -------- Color classifier (uses normalized r,g,b and brightness) --------
String classify(float r, float g, float b, float clearN) {
  float brightness = constrain(clearN, 0.0f, 1.0f);
  if (brightness < 0.08f) return "black"; // very dark

  // We already normalized by Clear; use raw r,g,b ratios
  float maxv = max(r, max(g, b));
  float minv = min(r, min(g, b));
  float midv = (r + g + b) - maxv - minv;

  String maxC, midC;
  if (maxv == r) {
    maxC = "red";
    midC = (g > b ? "green" : "blue");
    midv = (g > b ? g : b);
  } else if (maxv == g) {
    maxC = "green";
    midC = (r > b ? "red" : "blue");
    midv = (r > b ? r : b);
  } else { // maxv == b
    maxC = "blue";
    midC = (r > g ? "red" : "green");
    midv = (r > g ? r : g);
  }

  // near-neutral?
  if (fabs(r - g) < 0.06f && fabs(r - b) < 0.06f) {
    return (brightness > 0.6f) ? "white" : "gray";
  }

  const float DOM = 1.12f;            // 12% dominance threshold
  if (maxv > DOM * midv) return maxC; // pure red/green/blue

  if ((maxC == "red"   && midC == "green") ||
      (maxC == "green" && midC == "red"))   return "yellow";
  if ((maxC == "red"   && midC == "blue")  ||
      (maxC == "blue"  && midC == "red"))   return "magenta";
  if ((maxC == "green" && midC == "blue")  ||
      (maxC == "blue"  && midC == "green")) return "cyan";

  return "unknown";
}

//// -------- Servo helpers --------
void servoGo(int angle) {
  angle = constrain(angle, 0, 180);
  gate.write(angle);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // TCS3200 pins
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_OUT, INPUT);

  // IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);  // Set IR sensor pin as INPUT

  // 20% frequency scaling: S0=HIGH, S1=LOW
  digitalWrite(PIN_S0, HIGH);
  digitalWrite(PIN_S1, LOW);

  // Servo init at 50 Hz on GPIO18
  gate.setPeriodHertz(50);                 // <-- correct API name in v3.0.9
  gate.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoGo(0);                              // start at 0°

  Serial.println("\nTCS3200 + ESP32 + MG90S + IR Sensor");
  Serial.println("Pins: S0=14 S1=27 S2=26 S3=25 OUT=34 | SERVO=18 | IR_SENSOR=19");
  lastMeasureMs = millis() - MEASURE_PERIOD_MS; // force immediate first read
}

void loop() {
  // Read IR sensor: HIGH means object detected
  bool objectDetected = digitalRead(IR_SENSOR_PIN) == HIGH;

  if (!objectDetected) {
    Serial.println("Object detected!");

    if (millis() - lastMeasureMs >= MEASURE_PERIOD_MS) {
      lastMeasureMs = millis();

      // Read frequencies (S2/S3 mapping)
      uint32_t fRed   = readColorFreq(false, false); // R: L,L
      uint32_t fBlue  = readColorFreq(false, true);  // B: L,H
      uint32_t fClear = readColorFreq(true,  false); // C: H,L
      uint32_t fGreen = readColorFreq(true,  true);  // G: H,H

      // Normalize by Clear
      float rN = (fClear > 0) ? float(fRed)   / float(fClear) : 0.0f;
      float gN = (fClear > 0) ? float(fGreen) / float(fClear) : 0.0f;
      float bN = (fClear > 0) ? float(fBlue)  / float(fClear) : 0.0f;

      float clearN = (CLEAR_MAX_HZ > 1.0f)
                     ? min(1.0f, float(fClear) / CLEAR_MAX_HZ) : 0.0f;

      String color = classify(rN, gN, bN, clearN);

      Serial.printf("Freqs Hz  R:%lu  G:%lu  B:%lu  Clear:%lu\n",
                    (unsigned long)fRed, (unsigned long)fGreen,
                    (unsigned long)fBlue, (unsigned long)fClear);
      Serial.printf("Normalized R:%.2f G:%.2f B:%.2f  Bright:%.2f  -> %s\n",
                    rN, gN, bN, clearN, color.c_str());

      // Actions every 4 seconds based on detected color
      if (color == "red") {
        Serial.println("Action: RED → Servo 90° then 0°");
        servoGo(90);
        delay(700);
        servoGo(0);
      } else if (color == "blue") {
        Serial.println("Action: BLUE → Servo 180° then 0°");
        servoGo(180);
        delay(900);
        servoGo(0);
      } else {
        Serial.println("Action: none");
      }

      Serial.println();
    }
  } else {
    // No object detected
    Serial.println("No object detected.");
  }

  delay(500); // light idle
}
