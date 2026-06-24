/*
 * PROFESSIONAL LINE FOLLOWER ROBOT - 8-ARRAY SENSOR
 * D2 = Calibration | D3 = Start | Both = Emergency Stop
 * RATIO-BASED SPEED CONFIGURATION - Change ONLY lfSpeed
 * WITH 3-STAGE RECOVERY
 * SENSOR ORIENTATION: Sensor 7 (A7) = Left Most | Sensor 0 (A0) = Right Most
 */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//--------Pin definitions for the TB6612FNG Motor Driver----
#define AIN1 9    // Motor A input 1 (Left motor)
#define AIN2 8    // Motor A input 2 (Left motor)
#define PWMA 10   // Motor A speed (Left motor)
#define BIN1 7    // Motor B input 1 (Right motor)
#define BIN2 6    // Motor B input 2 (Right motor)
#define PWMB 5    // Motor B speed (Right motor)
#define STBY 11   // Standby pin

// ===== FUNCTION DECLARATIONS =====
void readLine();
void calibrate();
void linefollow();
void recoverySequence();
void motor1run(int motorSpeed);
void motor2run(int motorSpeed);
//------------------------------------------------------------

//--------Sensor Pin Definitions (8-Array)--------
// IMPORTANT: Sensor 7 (A7) = LEFT MOST | Sensor 0 (A0) = RIGHT MOST
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// Sensor arrangement (from RIGHT to LEFT):
// Sensor 0 (A0) - Right most
// Sensor 1 (A1) - Right
// Sensor 2 (A2) - Right center
// Sensor 3 (A3) - Center right
// Sensor 4 (A4) - Center left
// Sensor 5 (A5) - Left center
// Sensor 6 (A6) - Left
// Sensor 7 (A7) - Left most
//------------------------------------------------

//--------Enter Line Details here---------
bool isBlackLine = 1;             // 1 for black line, 0 for white line
unsigned int lineThickness = 25;  // Line thickness in mm
unsigned int numSensors = 8;      // Number of sensors
//-----------------------------------------

// ============ MASTER SPEED CONTROL - CHANGE ONLY THIS ============
int lfSpeed = 105;  // Maximum straight line speed (60-255)
// ALL OTHER SPEEDS WILL AUTO-ADJUST BASED ON THIS VALUE
// ================================================================

// ===== DYNAMIC TIMINGS (Calculated automatically) =====
int TIME_45_DEG;
int TIME_180_DEG;
int TIME_360_DEG;

// Auto-calculated speed variables (DO NOT CHANGE MANUALLY)
int minTurnSpeed;           // 67% of lfSpeed (for curves)
int calSpeed;               // 65% of lfSpeed (for calibration)
int recoverySpeed;          // 85% of lfSpeed (for recovery)
int recoveryTurnSpeed;      // 60% of lfSpeed (spin speed)
int accelerationStep;       // Auto-calculated (1-3)
int recoveryLeftTurn;       // Auto-calculated based on lfSpeed
int recoveryRightTurn;      // Auto-calculated based on lfSpeed
int loopDelay;              // Auto-adjusted based on speed
int calibrationSeconds;     // Fixed at 4 seconds

// PID Variables - OPTIMIZED FOR 8 SENSORS
int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int currentSpeed = 50;
// 8-sensor weights (REVERSED because sensor 7 is left most)
// Negative weights = left side, Positive weights = right side
int sensorWeight[8] = { 8, 4, 2, 1, -1, -2, -4, -8 };
int activeSensors;
float Kp = 0.08;   // Adjusted for 8 sensors
float Kd = 0.18;   // Adjusted for 8 sensors
float Ki = 0.0;    // Minimal integral

// Sensor Variables
int onLine = 0;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];

// Button pins
const int CAL_BTN = 2;    // Calibration button (D2)
const int START_BTN = 3;  // Start button (D3)

// State variables
bool isRunning = false;
bool calibrated = false;
bool emergencyStop = false;

// ============ 3-STAGE RECOVERY VARIABLES ============
int lastLineSide = 0;     // -1 = left, +1 = right
bool inRecovery = false;  // Flag to track if we're in recovery mode

// Debounce variables
const int debounceDelay = 50;

void setup() {
  // Calculate all speeds based on lfSpeed
  calculateSpeeds();
  
  // Speed up ADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Initialize motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  // Initialize button pins (with pull-up)
  pinMode(CAL_BTN, INPUT_PULLUP);
  pinMode(START_BTN, INPUT_PULLUP);

  // Initialize sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  // Configure line thickness
  lineThickness = constrain(lineThickness, 10, 35);
  
  // Brief delay for stabilization
  delay(500);
}

// ============ AUTO SPEED CALCULATION FUNCTION (REPLACED) ============
void calculateSpeeds() {
  lfSpeed = constrain(lfSpeed, 60, 255);

  // ===== SPEED SCALING =====
  minTurnSpeed       = lfSpeed * 0.67;
  calSpeed           = lfSpeed * 0.65;
  recoverySpeed      = lfSpeed * 0.85;
  recoveryTurnSpeed  = lfSpeed * 0.60;

  recoveryLeftTurn   = -lfSpeed * 0.45;
  recoveryRightTurn  =  lfSpeed * 0.85;

  minTurnSpeed = constrain(minTurnSpeed, 40, 180);
  calSpeed = constrain(calSpeed, 50, 150);
  recoverySpeed = constrain(recoverySpeed, 60, 200);
  recoveryTurnSpeed = constrain(recoveryTurnSpeed, 40, 150);

  accelerationStep = max(1, lfSpeed / 80);
  loopDelay = constrain(20 - (lfSpeed / 15), 5, 20);

  calibrationSeconds = 4;

  // ===== ROTATION SCALING =====
  float baseSpeed = 80.0;
  float scale = baseSpeed / lfSpeed;
  scale = constrain(scale, 0.4, 2.5);

  TIME_45_DEG  = 300  * scale;
  TIME_180_DEG = 780  * scale;
  TIME_360_DEG = 1560 * scale;
}

// ============ 3-STAGE RECOVERY FUNCTION (REPLACED) ============
void recoverySequence() {
  inRecovery = true;

  int turnSpeed = recoveryTurnSpeed * 1.2;
  turnSpeed = constrain(turnSpeed, 60, 180);

  // ===== STAGE 1: 45° =====
  unsigned long startTime = millis();
  while (millis() - startTime < TIME_45_DEG) {
    readLine();
    if (onLine) return;

    if (lastLineSide == -1) {
      motor1run(-turnSpeed);
      motor2run(turnSpeed);
    } else {
      motor1run(turnSpeed);
      motor2run(-turnSpeed);
    }
  }

  // ===== STAGE 2: 180° =====
  startTime = millis();
  while (millis() - startTime < TIME_180_DEG) {
    readLine();
    if (onLine) return;

    if (lastLineSide == -1) {
      motor1run(turnSpeed);
      motor2run(-turnSpeed);
    } else {
      motor1run(-turnSpeed);
      motor2run(turnSpeed);
    }
  }

  // ===== STAGE 3: 360° =====
  startTime = millis();
  while (millis() - startTime < TIME_360_DEG) {
    readLine();
    if (onLine) return;

    if (lastLineSide == -1) {
      motor1run(-turnSpeed);
      motor2run(turnSpeed);
    } else {
      motor1run(turnSpeed);
      motor2run(-turnSpeed);
    }
  }

  inRecovery = false;
}

void loop() {
  // Read button states (LOW when pressed due to pull-up)
  int calState = digitalRead(CAL_BTN);
  int startState = digitalRead(START_BTN);
  
  // ============ EMERGENCY STOP ============
  if (calState == LOW && startState == LOW) {
    delay(debounceDelay);
    
    if (digitalRead(CAL_BTN) == LOW && digitalRead(START_BTN) == LOW) {
      emergencyStop = true;
      isRunning = false;
      inRecovery = false;
      motor1run(0);
      motor2run(0);
      
      while (digitalRead(CAL_BTN) == LOW || digitalRead(START_BTN) == LOW) {
        delay(10);
      }
      
      delay(100);
      emergencyStop = false;
    }
  }
  
  // ============ CALIBRATION BUTTON (D2 alone) ============
  else if (calState == LOW && startState == HIGH && !emergencyStop) {
    delay(debounceDelay);
    
    if (digitalRead(CAL_BTN) == LOW && digitalRead(START_BTN) == HIGH) {
      calibrate();
      calibrated = true;
      
      while (digitalRead(CAL_BTN) == LOW) {
        delay(10);
      }
      delay(100);
    }
  }
  
  // ============ START BUTTON (D3 alone) ============
  else if (startState == LOW && calState == HIGH && calibrated && !emergencyStop) {
    delay(debounceDelay);
    
    if (digitalRead(START_BTN) == LOW && digitalRead(CAL_BTN) == HIGH) {
      isRunning = !isRunning;
      
      if (!isRunning) {
        motor1run(0);
        motor2run(0);
        currentSpeed = 30;
        inRecovery = false;
        lastLineSide = 0;
      } else {
        currentSpeed = minTurnSpeed;
        inRecovery = false;
        lastLineSide = 0;
      }
      
      while (digitalRead(START_BTN) == LOW) {
        delay(10);
      }
      delay(100);
    }
  }
  
  // ============ LINE FOLLOWING LOOP (REPLACED) ============
  if (isRunning && calibrated && !emergencyStop) {
    readLine();
    
    // ===== FIXED LAST LINE SIDE DETECTION FOR 8 ARRAY =====
    // LEFT side (A5, A6, A7) - indices 5,6,7
    if (sensorArray[5] || sensorArray[6] || sensorArray[7]) {
      lastLineSide = -1;
    }
    // RIGHT side (A0, A1, A2) - indices 0,1,2
    if (sensorArray[0] || sensorArray[1] || sensorArray[2]) {
      lastLineSide = +1;
    }
    
    // Gradually increase speed
    if (currentSpeed < lfSpeed) {
      currentSpeed += accelerationStep;
    }
    
    // ===== REPLACED LINE LOST LOGIC =====
    if (onLine == 1) {
      inRecovery = false;
      linefollow();
    } else {
      inRecovery = true;
      recoverySequence();
    }
    
    delay(loopDelay);
  }
}

void linefollow() {
  long weightedSum = 0;
  long sum = 0;

  // ===== CENTER BIAS WEIGHTS (VERY IMPORTANT) =====
  int improvedWeight[8] = { 6, 3, 1, 0, 0, -1, -3, -6 };

  for (int i = 0; i < 8; i++) {
    int value = sensorValue[i];

    // EDGE SENSOR SUPPRESSION
    if (i == 0 || i == 7) {
      value *= 0.4;  // reduce extreme sensor influence
    }

    weightedSum += (long)value * improvedWeight[i];
    sum += value;
  }

  if (sum != 0) {
    error = (float)weightedSum / sum;
  }

  // ===== PID CONTROL =====
  P = error;

  I += error;
  I = constrain(I, -50, 50);   // tighter control

  D = error - previousError;

  // FINAL TUNED PID (VERY STABLE)
  PIDvalue = (0.08 * P) + (0.00 * I) + (0.18 * D);

  previousError = error;

  // ===== DEAD BAND (ANTI-JITTER) =====
  if (abs(PIDvalue) < 8) PIDvalue = 0;

  // ===== MOTOR SPEED CONTROL =====
  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  // ===== SMOOTH TURN LIMITING =====
  if (abs(PIDvalue) > 60) {
    lsp *= 0.85;
    rsp *= 0.85;
  }

  lsp = constrain(lsp, 0, 255);
  rsp = constrain(rsp, 0, 255);

  motor1run(lsp);
  motor2run(rsp);
}

void calibrate() {
  // Reset calibration values
  for (int i = 0; i < 8; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }

  // Rotate clockwise during calibration
  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)(calibrationSeconds * 1000)) {
    motor1run(calSpeed);
    motor2run(-calSpeed);

    // Read all 8 sensors
    for (int i = 0; i < 8; i++) {
      int value = analogRead(sensorPins[i]);
      
      if (value < minValues[i]) {
        minValues[i] = value;
      }
      if (value > maxValues[i]) {
        maxValues[i] = value;
      }
    }
    delay(10);
  }

  // Stop motors
  motor1run(0);
  motor2run(0);
  delay(100);

  // Calculate thresholds
  for (int i = 0; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }
}

void readLine() {
  onLine = 0;
  
  // Read all 8 sensors
  for (int i = 0; i < 8; i++) {
    int rawValue = analogRead(sensorPins[i]);
    
    // Map to 0-1000 based on calibration
    if (isBlackLine) {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 0, 1000);
    } else {
      sensorValue[i] = map(rawValue, minValues[i], maxValues[i], 1000, 0);
    }
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    
    // Determine if sensor sees line (threshold 500)
    sensorArray[i] = (sensorValue[i] > 500);
    
    // Check if any sensor sees the line
    if (sensorArray[i]) {
      onLine = 1;
    }
  }
}

//--------Function to run Motor 1 (Left motor)-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2 (Right motor)-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}
