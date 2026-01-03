#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor pins
#define ENA 5
#define ENB 6
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11

// PID values (SAFE START)
float Kp = 27.5;
float Ki = 350;
float Kd = 0.7;

// Setpoint from your measurement
float setPoint = 2.12;

// MPU variables
int16_t ax, ay, az, gx, gy, gz;
float accAngle, gyroRate;
float angle = 0;
float filteredAngle;

float error, lastError = 0;
float integral = 0;
float derivative;
float pidOutput;

unsigned long lastTime;
float dt;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1);
  }

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accAngle = atan2(ay, az) * 57.2958;
  gyroRate = gx / 131.0;

  angle = 0.98 * (angle + gyroRate * dt) + 0.02 * accAngle;
  filteredAngle = angle;

  // SAFETY STOP
  if (abs(filteredAngle - setPoint) > 20) {
    stopMotors();
    return;
  }

  // PID
  error = setPoint - filteredAngle;
  integral += error * dt;
  derivative = (error - lastError) / dt;
  pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  pidOutput = constrain(pidOutput, -255, 255);

  moveMotors(pidOutput);

  Serial.print("Angle: ");
  Serial.print(filteredAngle);
  Serial.print(" PID: ");
  Serial.println(pidOutput);
}

void moveMotors(float speed) {
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    speed = -speed;
  }

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}