#include <AStar32U4Motors.h>
#include <Encoder.h>
#include <pid-autotune.h>

#define PI 3.141592653589

// ================= MOTOR/ENCODER CONFIG =================
AStar32U4Motors m;

const int encoderRightPinA = 16;
const int encoderRightPinB = 15;
const int encoderLeftPinA = 8;
const int encoderLeftPinB = 11;

Encoder encoderRight(encoderRightPinA, encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA, encoderLeftPinB);

const int encoderResolution = 1440;  // counts per revolution
const double wheelDiameter = 2.7559055; // inches
const double wheelRadius = wheelDiameter / 2.0;
const double interval = 5; // control loop every 5 ms (200 Hz)

// ================= PID VARIABLES =================
double Kp = 0.0;
double Ki = 0.0;
double Kd = 0.0;

double desVelL = 20.0; // desired left linear velocity (in/s)
double desVelR = 20.0; // desired right linear velocity (in/s)

double velLeft = 0, velRight = 0;
double newVelLeft = 0, newVelRight = 0;

//// ================= AUTOTUNE CONFIG =================
PID pid = PID();
pid_tuner tuner = pid_tuner(pid, 10, 10000, pid_tuner::CLASSIC_PID);

bool tuning = true;  // start with autotuning enabled

// ================= TIMING =================
unsigned long prevMillis = 0;
int posLeftCountLast = 0;
int posRightCountLast = 0;

// ================= MOTOR LIMITS =================
const double leftMotorMax = 26.45;
const double rightMotorMax = 28.86;

// ================== OUTPUT FUNCTION ==================
void outputFunc(double pwm) {
  // limit to 0–255 range
  pwm = constrain(pwm, 0, 255);
  analogWrite(11, pwm);  // or appropriate motor PWM pin
}

// ================== ENCODER VELOCITY ==================
double computeLinearVelocity(Encoder& enc, int& lastCount) {
  static unsigned long lastTime = 0;
  unsigned long now = millis();
  double dt = (now - lastTime);

  if (dt == 0) return 0;
  int count = enc.read();
  int diff = count - lastCount;
  lastCount = count;
  lastTime = now;

  // Angular velocity (rad/s)
  double omega = ((2 * PI) / encoderResolution) * (diff / (dt / 1000.0));
  // Linear velocity (in/s)
  return wheelRadius * omega;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  
  m.setM1Speed(0);
  m.setM2Speed(0);

  // Configure autotune
 tuner.setConstrains(0, 400);
 tuner.setTargetValue(20); // target velocity (in/s)
 delay(2000);
 Serial.println("Starting PID auto-tuning...xxxxxx");
 // Serial.println("Starting PID auto-tuning...");
}

// ================== LOOP ==================
void loop() {
  Serial.println("Starting PID auto-tuning...xxxyyxxx");
  unsigned long currentMillis = millis();

  if (tuning) {
    // Run autotune once on one wheel (left)
    tuner.tune([](int pin){ return computeLinearVelocity(encoderLeft, posLeftCountLast); }, A0, outputFunc);

    // Once tuning is done, print new gains
    Kp = tuner.getKp();
    Ki = tuner.getKi();
    Kd = tuner.getKd();

    Serial.println("Tuning complete!");
    Serial.print("Kp: "); Serial.println(Kp);
    Serial.print("Ki: "); Serial.println(Ki);
    Serial.print("Kd: "); Serial.println(Kd);

    tuning = false; // switch to PID control
  }
  else if (currentMillis - prevMillis >= interval) {
    prevMillis = currentMillis;
  }
    // === Compute encoder velocities ===
    velLeft = computeLinearVelocity(encoderLeft, posLeftCountLast);
    velRight = computeLinearVelocity(encoderRight, posRightCountLast);

    // === Apply tuned PID control ===
    newVelLeft = drivePID(velLeft, desVelL, Kp, Ki, Kd);
    newVelRight = drivePID(velRight, desVelR, Kp, Ki, Kd);

    // === Map to motor commands ===
    int leftMotorCmd = motorVelToSpeedCommand(newVelLeft, leftMotorMax);
    int rightMotorCmd = motorVelToSpeedCommand(newVelRight, rightMotorMax);

    m.setM1Speed(leftMotorCmd);
    m.setM2Speed(rightMotorCmd);

    Serial.print(velLeft);
    Serial.print(',');
    Serial.println(velRight);
}

// ================== PID FUNCTION ==================
double drivePID(double curr, double setpoint, double kp, double ki, double kd) {
  static double cumError = 0, lastError = 0;
  static unsigned long priorTime = 0;

  unsigned long currentTime = millis();
  double elapsed = (double)(currentTime - priorTime);

  double error = setpoint - curr;
  cumError += error * elapsed;
  double rateError = (error - lastError) / elapsed;

  double out = kp * error + ki * cumError + kd * rateError;

  // anti-windup
  cumError = constrain(cumError, -20, 20);

  lastError = error;
  priorTime = currentTime;

  return out;
}

// ================== MOTOR MAPPING ==================
int motorVelToSpeedCommand(double Vel, double maxVel) {
  Vel = constrain(Vel, -1 * maxVel, maxVel);
  return map(Vel, -1 * maxVel, maxVel, -400, 400);
}
