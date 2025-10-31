#include <Arduino.h>
#include <ESP32Servo.h>

// ----- wiring -----
// MG90S signal -> GPIO 18
// Power the servo from 5V (prefer external 5V >= 1A) and tie grounds together.

const int SERVO_PIN = 18;

// Tune these if your MG90S hits end-stops or doesn’t reach full travel
const int MIN_US = 500;   // try 600 if it buzzes at 0°
const int MAX_US = 2400;  // try 2300 if it hits the end at 180°

Servo servo;

// Sequence: 0° -> 90° -> 180° -> 90° -> repeat
const int positions[] = {0, 90, 180, 90};
const int numPositions = sizeof(positions) / sizeof(positions[0]);

// Timing
const uint32_t STEP_PERIOD_MS = 2000;   // total time per 90° step (move + dwell)
const uint32_t MOVE_TIME_MS   = 600;    // how long to ramp between angles (smoothness)
const uint32_t STEP_INTERVAL  = 10;     // update servo every 10 ms during motion

// State
int currentDeg = 90;
int targetIdx  = 0;
int startDeg   = 90;
uint32_t moveStartMs = 0;
uint32_t lastUpdateMs = 0;
bool moving = false;

// Simple ease-in-out (cosine) for smoother motion than linear
// t in [0..1] -> eased t
float easeInOut(float t) {
  return 0.5f - 0.5f * cosf(t * PI);
}

void startMoveTo(int nextDeg) {
  startDeg = currentDeg;
  moving = true;
  moveStartMs = millis();
  lastUpdateMs = moveStartMs;
}

void setup() {
  // Reserve PWM timers (avoids conflicts with other PWM uses)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);                 // standard servo freq
  servo.attach(SERVO_PIN, MIN_US, MAX_US);  // attach with pulse limits
  currentDeg = 90;
  servo.write(currentDeg);

  // Begin first move
  startMoveTo(positions[targetIdx]);
}

void loop() {
  uint32_t now = millis();

  if (moving) {
    // Update every STEP_INTERVAL for a smooth path
    if (now - lastUpdateMs >= STEP_INTERVAL) {
      lastUpdateMs = now;
      uint32_t elapsed = now - moveStartMs;
      float t = (float)elapsed / (float)MOVE_TIME_MS;
      if (t >= 1.0f) {
        // Reached target
        currentDeg = positions[targetIdx];
        servo.write(currentDeg);
        moving = false;
        // Schedule when to start the next move so total step period is 2000 ms
        moveStartMs = now; // reuse as "finished at"
      } else {
        // Smoothly interpolate
        float e = easeInOut(t);
        int targetDeg = positions[targetIdx];
        int interp = startDeg + (int)roundf((targetDeg - startDeg) * e);
        currentDeg = interp;
        servo.write(currentDeg);
      }
    }
  } else {
    // Dwell until STEP_PERIOD_MS elapsed since the start of this step
    // (MOVE_TIME_MS was used for motion; the rest is idle)
    uint32_t finishedAt = moveStartMs; // set above when motion ended
    if (now - finishedAt >= (STEP_PERIOD_MS - MOVE_TIME_MS)) {
      // Advance to next waypoint and start the next smooth move
      targetIdx = (targetIdx + 1) % numPositions;
      startMoveTo(positions[targetIdx]);
    }
  }
}
