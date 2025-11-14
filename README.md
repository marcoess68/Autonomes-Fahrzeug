# Autonomes-Fahrzeug
Das autonome Fahrzeug ist ein mobiles Robotersystem, das mithilfe eines Ultraschallsensors Hindernisse erkennt und selbstständig ausweicht. Die Steuerung erfolgt über einen Arduino Uno R4 WiFi, der die Sensoren, Motoren, LEDs und das Display koordiniert. 

´´´
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ------------------ Pins ------------------
#define SERVO_PIN 9
#define TRIG_PIN 10
#define ECHO_PIN 2 

// Motor pins (DFRobot Quad Motor Shield V1.0)
int motorPWM[4] = {3, 5, 6, 11};
int motorDIR[4] = {4, 7, 8, 12};

// ------------------ LED pins (moved to avoid conflict with motor DIR pins) ------------------
#define BACK_LED_LEFT   A0   // was 4 (conflicted with motorDIR[0])
#define BACK_LED_RIGHT  A1   // was 7 (conflicted with motorDIR[1])
#define FRONT_LED_LEFT  A2   // was 12 (motorDIR[3] used 12 earlier; keep front on free pin)
#define FRONT_LED_RIGHT A3   // was 13

// ------------------ Objects ------------------
Servo radarServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ------------------ Configurable constants ------------------
const int THRESHOLD        = 40;    // cm distance to trigger obstacle
const int SWEEP_DELAY      = 2;     // ms between servo steps
const int STEP_SIZE        = 6;    // degrees per update (larger = faster sweep)
const int MAX_SPEED        = 20;    // % motor speed
const int REVERSE_TIME     = 2500;  // ms to reverse
const int TURN_TIME        = 2500;  // ms to turn
const int SERVO_SETTLE     = 1;    // ms wait after servo write
const int MEASURE_SAMPLES  = 3;     // number of readings to average
const int DEBOUNCE_COUNT   = 2;     // consecutive readings below threshold
const unsigned long AVOID_COOLDOWN = 100; // ms cooldown after avoidance
const unsigned long REAR_BLINK_INTERVAL = 300; // ms for rear blinking

// ------------------ Radar sweep control ------------------
int servoPos = 90;
int servoDir = 1;
bool radarPaused = false;

// ------------------ Avoidance state machine ------------------
bool avoidInProgress = false;
unsigned long avoidStartTime = 0;
int avoidStep = 0;
int lastObstacleAngle = 90;

// ------------------ Timing / detection ------------------
unsigned long lastSweepTime = 0;
int consecutiveBelow = 0;
unsigned long lastAvoidEndTime = 0;

// ------------------ LED control ------------------
bool rearLightState = true;
unsigned long lastBlinkTime = 0;

// ------------------ Setup ------------------
bool startAllowed = true;

void setup() {
  Serial.begin(115200);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Autonomous Car");
  lcd.setCursor(0, 1);
  lcd.print("System Ready!");
  delay(1500);
  lcd.clear();

  // Servo
  radarServo.attach(SERVO_PIN);
  radarServo.write(servoPos);

  // Motors
  for (int i = 0; i < 4; i++) {
    pinMode(motorPWM[i], OUTPUT);
    pinMode(motorDIR[i], OUTPUT);
    analogWrite(motorPWM[i], 0);
    digitalWrite(motorDIR[i], LOW);
  }

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LEDs
  pinMode(FRONT_LED_LEFT, OUTPUT);
  pinMode(FRONT_LED_RIGHT, OUTPUT);
  pinMode(BACK_LED_LEFT, OUTPUT);
  pinMode(BACK_LED_RIGHT, OUTPUT);

  // --- Startup animation (blink all LEDs 3×) ---
  for (int i = 0; i < 3; i++) {
    setAllLights(HIGH);
    delay(250);
    setAllLights(LOW);
    delay(250);
  }

  // Keep lights ON
  setAllLights(HIGH);

  Serial.println("Autonomous car ready!");
  updateLCD(0, servoPos, "READY");
}

// ------------------ Main loop ------------------
void loop() {
  if (!startAllowed) {
    stopMotors();
    updateLCD(0, servoPos, "STOPPED");
    return;
  }

  updateLights();

  if (!avoidInProgress) {
    bool moved = sweepRadar();

    if (moved) {
      int dist = measureDistance();
      Serial.print("SCAN ");
      Serial.print(servoPos);
      Serial.print("° -> ");
      Serial.print(dist);
      Serial.println(" cm");

      if (dist > 0 && dist < THRESHOLD) consecutiveBelow++;
      else if (dist > THRESHOLD) consecutiveBelow = 0;

      if (consecutiveBelow >= DEBOUNCE_COUNT && (millis() - lastAvoidEndTime) > AVOID_COOLDOWN) {
        Serial.println("OBSTACLE confirmed -> START AVOID");
        stopMotors();
        radarPaused = true;
        avoidInProgress = true;
        avoidStep = 0;
        avoidStartTime = millis();
        lastObstacleAngle = servoPos;
        consecutiveBelow = 0;
        updateLCD(dist, servoPos, "OBSTACLE!");
      } else {
        updateLCD(dist, servoPos, "FORWARD");
      }
    }

    if (!avoidInProgress) driveForward();
  } else {
    handleAvoidance();
  }
}

// ------------------ Radar sweeping ------------------
bool sweepRadar() {
  if (radarPaused) return false;

  if (millis() - lastSweepTime > SWEEP_DELAY) {
    servoPos += STEP_SIZE * servoDir;
    if (servoPos >= 180) { servoPos = 180; servoDir = -1; }
    if (servoPos <= 0)   { servoPos = 0;   servoDir = 1;  }

    radarServo.write(servoPos);
    delay(SERVO_SETTLE);
    lastSweepTime = millis();
    return true;
  }
  return false;
}

// ------------------ Distance ------------------
int measureDistance() {
  long total = 0;
  int count = 0;

  for (int i = 0; i < MEASURE_SAMPLES; i++) {
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    if (duration > 0) {
      int d = duration * 0.034 / 2;
      if (d >= 2 && d <= 300) { total += d; count++; }
    }
    delay(5);
  }

  if (count == 0) return -1;
  return total / count;
}

// ------------------ Motors ------------------
void setMotor(int m, int dir, int speedPct) {
  speedPct = constrain(speedPct, 0, 100);
  int pwmVal = map(speedPct, 0, 100, 0, 255);

  if (m == 1 || m == 3) dir = -dir;

  if (dir == 0) analogWrite(motorPWM[m], 0);
  else {
    digitalWrite(motorDIR[m], (dir > 0) ? LOW : HIGH);
    analogWrite(motorPWM[m], pwmVal);
  }
}

void stopMotors()       { for (int i = 0; i < 4; i++) setMotor(i, 0, 0); }
void driveForward()     { for (int i = 0; i < 4; i++) setMotor(i, 1, MAX_SPEED); }
void driveBackward()    { for (int i = 0; i < 4; i++) setMotor(i, -1, MAX_SPEED); }

void turnLeft() {
  setMotor(0,  1, MAX_SPEED);
  setMotor(2,  1, MAX_SPEED);
  setMotor(1, -1, MAX_SPEED);
  setMotor(3, -1, MAX_SPEED);
}

void turnRight() {
  setMotor(1,  1, MAX_SPEED);
  setMotor(3,  1, MAX_SPEED);
  setMotor(0, -1, MAX_SPEED);
  setMotor(2, -1, MAX_SPEED);
}

// ------------------ Obstacle avoidance ------------------
void handleAvoidance() {
  unsigned long now = millis();

  switch (avoidStep) {
    case 0:
      driveBackward();
      updateLCD(0, servoPos, "REVERSING");
      if (now - avoidStartTime > REVERSE_TIME) {
        avoidStep = 1;
        avoidStartTime = now;
        stopMotors();
        Serial.println("REV complete -> TURN");
      }
      break;

    case 1:
      if (lastObstacleAngle < 75) {
        turnRight();
        updateLCD(0, servoPos, "TURN RIGHT");
      } else if (lastObstacleAngle > 105) {
        turnLeft();
        updateLCD(0, servoPos, "TURN LEFT");
      } else {
        turnRight();
        updateLCD(0, servoPos, "TURN RIGHT");
      }

      if (now - avoidStartTime > TURN_TIME) {
        avoidStep = 2;
        avoidStartTime = now;
        stopMotors();
        Serial.println("TURN complete -> RESUME");
      }
      break;

    case 2:
      driveForward();
      radarPaused = false;
      avoidInProgress = false;
      lastAvoidEndTime = millis();
      updateLCD(0, servoPos, "FORWARD");
      break;
  }
}

// ------------------ LED control ------------------
void updateLights() {
  if (avoidInProgress && avoidStep == 0) {  // reversing
    if (millis() - lastBlinkTime > REAR_BLINK_INTERVAL) {
      lastBlinkTime = millis();
      rearLightState = !rearLightState;
      digitalWrite(BACK_LED_LEFT, rearLightState);
      digitalWrite(BACK_LED_RIGHT, rearLightState);
    }
  } else {
    // steady ON during normal or turning
    digitalWrite(BACK_LED_LEFT, HIGH);
    digitalWrite(BACK_LED_RIGHT, HIGH);
  }

  // front lights always ON
  digitalWrite(FRONT_LED_LEFT, HIGH);
  digitalWrite(FRONT_LED_RIGHT, HIGH);
}

void setAllLights(bool state) {
  digitalWrite(FRONT_LED_LEFT, state);
  digitalWrite(FRONT_LED_RIGHT, state);
  digitalWrite(BACK_LED_LEFT, state);
  digitalWrite(BACK_LED_RIGHT, state);
}

// ------------------ LCD helper ------------------
void updateLCD(int dist, int angle, String state) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("D:");
  lcd.print(dist);
  lcd.print("cm A:");
  lcd.print(angle);
  lcd.setCursor(0, 1);
  lcd.print(state);
}
´´´
