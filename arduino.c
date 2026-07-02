#include <Servo.h>
#include <DHT.h>

#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define TRIG 3
#define ECHO 4
#define IR_PIN 5

Servo camServo;
#define SERVO_PIN 6

#define LED_FRONT A0
#define LED_REAR  A1
#define LED_LEFT  A2
#define LED_RIGHT A3

// Motor driver (L298N)
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10
#define ENA 11
#define ENB 12

// Debug LED (built‑in pin 13)
#define DEBUG_LED 13

unsigned long lastSensorSend = 0;

void setup() {
  Serial.begin(115200);          // communication with ESP32
  dht.begin();

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(LED_FRONT, OUTPUT);
  pinMode(LED_REAR, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  digitalWrite(LED_FRONT, HIGH);
  digitalWrite(LED_REAR, HIGH);
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_RIGHT, HIGH);

  camServo.attach(SERVO_PIN);
  camServo.write(90);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  stopMotors();

  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, LOW);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Blink debug LED on every command
    digitalWrite(DEBUG_LED, HIGH);
    delay(50);
    digitalWrite(DEBUG_LED, LOW);

    if (cmd.startsWith("M ")) {
      int sp1 = cmd.indexOf(' ', 2);
      int leftSpeed = cmd.substring(2, sp1).toInt();
      int rightSpeed = cmd.substring(sp1+1).toInt();
      setMotorA(leftSpeed);
      setMotorB(rightSpeed);
    } else if (cmd.startsWith("S ")) {
      int angle = cmd.substring(2).toInt();
      camServo.write(constrain(angle, 0, 180));
    } else if (cmd.startsWith("L1 ")) {
      digitalWrite(LED_FRONT, cmd.substring(3).toInt() ? HIGH : LOW);
    } else if (cmd.startsWith("L2 ")) {
      digitalWrite(LED_REAR, cmd.substring(3).toInt() ? HIGH : LOW);
    } else if (cmd.startsWith("L3 ")) {
      digitalWrite(LED_LEFT, cmd.substring(3).toInt() ? HIGH : LOW);
    } else if (cmd.startsWith("L4 ")) {
      digitalWrite(LED_RIGHT, cmd.substring(3).toInt() ? HIGH : LOW);
    }
  }

  if (millis() - lastSensorSend > 500) {
    sendSensorData();
    lastSensorSend = millis();
  }
}

void sendSensorData() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) { t = 0; h = 0; }

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 30000);
  float dist = (duration == 0) ? -1 : duration * 0.0343 / 2.0;

  int ir = digitalRead(IR_PIN);

  Serial.print("T:"); Serial.print(t, 1);
  Serial.print(" H:"); Serial.print(h, 1);
  Serial.print(" D:"); Serial.print(dist, 1);
  Serial.print(" I:"); Serial.println(ir);
}

void setMotorA(int speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void setMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
  } else if (speed < 0) {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENB, -speed);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void stopMotors() {
  setMotorA(0); setMotorB(0);
}f
