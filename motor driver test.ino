// Pins (change if you wired differently)
const int ENA_PIN = 5;   // ESP32 GPIO5 -> L298N ENA
const int IN1_PIN = 18;  // ESP32 GPIO18 -> L298N IN1
const int IN2_PIN = 19;  // ESP32 GPIO19 -> L298N IN2

void setup() {
  // Set control pins as outputs
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  // Set fixed direction: IN1 HIGH, IN2 LOW
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  // Start with motor off
  digitalWrite(ENA_PIN, LOW);
}

void loop() {
  // Motor ON for 2 seconds
  digitalWrite(ENA_PIN, HIGH);
  delay(2000);  // 2000 ms = 2 seconds

  // Motor OFF for 2 seconds
  digitalWrite(ENA_PIN, LOW);
  delay(2000);
}
