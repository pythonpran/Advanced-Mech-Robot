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
const double d = 2.7559055; // inches
const double wheelRadius = wheelDiameter / 2.0;
const double interval = 5; // control loop every 5 ms (200 Hz)

// ================= PID VARIABLES =================
float Kp_l = 0.1;
float Ki_l = 0.3;
float Kd_l = 0.2;

float Kp_r = 0.1;
float Ki_r = 0.3;
float Kd_r = 0.2;

double desVelL = 10.0; // desired left linear velocity (in/s)
double desVelR = 10.0; // desired right linear velocity (in/s)

double velLeft = 0, velRight = 0;
double newVelLeft = 0, newVelRight = 0;

float diffRight=0;
float diffLeft=0;
double posLeftRad = 0.0; // this will need to be converted to rad/sec
double posRightRad = 0.0; // this will need to be converted to rad/sec
int posLeftCount = 0;
int posRightCount = 0;
unsigned long currentMillis;
double dtlastL = 0;double dtL = 0;
double dtlastR = 0;double dtR = 0;
double interval2L;
double interval2R;
unsigned long priorTimeL;
double lastSpeedErrorL;
double cumErrorL;
unsigned long priorTimeR;
double lastSpeedErrorR;
double cumErrorR;
double maxErr = 20;

// ================= AUTOTUNE CONFIG =================
PID pid = PID();
pid_tuner tuner = pid_tuner(pid, 1000, 900, pid_tuner::CLASSIC_PID);
bool tuning = true;  // start with autotuning enabled

// ================= TIMING =================
unsigned long prevMillis = 0;
int posLeftCountLast = 0;
int posRightCountLast = 0;

// ================= MOTOR LIMITS =================
const double leftMotorMax = 26.45;
const double rightMotorMax = 28.86;

// ================== OUTPUT FUNCTION ==================
void outputFuncLeft(double pwm) {
    pwm = constrain(pwm, -400, 400);
    m.setM1Speed(pwm);
}
void outputFuncRight(double pwm) {
    pwm = constrain(pwm, -400, 400);
    m.setM2Speed(pwm);
}

//// ================== ENCODER VELOCITY ==================
//double computeLinearVelocity(Encoder& enc, int& lastCount) {
//  static unsigned long lastTime = 0;
//  unsigned long now = millis();
//  double dt = (now - lastTime);
//
//  if (dt == 0) return 0;
//  int count = enc.read();
//  int diff = count - lastCount;
//  lastCount = count;
//  lastTime = now;
//  // Angular velocity (rad/s)
//  double omega = ((2 * PI) / encoderResolution) * (diff / dt) * 1000.0;
//  // Linear velocity (in/s)
//  return wheelRadius * omega;
//}

//// ================== ENCODER VELOCITY LEFT ==================
//double computeLinearVelocityLeft() {
//
//     diffLeft = posLeftCount - posLeftCountLast;
//     posLeftRad = ((2*PI)/encoderResolution)*(diffLeft/interval)*1000; 
//
//     velLeft = (d/2)*posLeftRad; // Same - Inches/sec
//    
//  return velLeft;
//}

 //================== ENCODER VELOCITY RIGHT ==================
double computeLinearVelocityRight() {
     dtR=millis();
     interval2R = dtR-dtlastR;
     yield();
     posRightCount = encoderRight.read();
     diffRight = posRightCount - posRightCountLast;
     posRightRad = ((2*PI)/encoderResolution)*(diffRight/interval2R)*1000; 
     velRight = (d/2)*posRightRad; // Now convert to get inches/sec (tangential velocity)
     posRightCountLast = posRightCount;
     dtlastR=dtR;
  return velRight;
}

 //================== ENCODER VELOCITY LEFT ==================
double computeLinearVelocityLeft() {
     dtL=millis();
     interval2L = dtL-dtlastL;
     yield();
     posLeftCount = encoderLeft.read();
     diffLeft = posLeftCount - posLeftCountLast;
     posLeftRad = ((2*PI)/encoderResolution)*(diffLeft/interval2L)*1000; 
     velLeft = (d/2)*posLeftRad; // Now convert to get inches/sec (tangential velocity)
     posLeftCountLast = posLeftCount;
     dtlastL=dtL;
  return velLeft;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(9600);
  delay(2000);
  m.setM1Speed(0);
  m.setM2Speed(0);
  tuner.setConstrains(-400, 400);
  tuner.setTargetValue(10); // target velocity (in/s)
  Serial.println("Starting PID auto-tuning...");
}

// ================== LOOP ==================
void loop() {
  unsigned long currentMillis = millis();
  posRightCount = encoderRight.read(); 
  posLeftCount = encoderLeft.read();
  if (tuning) {
    
    // Run autotune once on one wheel (left)

    tuner.tune([](int pin){ return computeLinearVelocityLeft(); }, 0, outputFuncLeft,[](){ encoderLeft.read(); });
    
    Kp_l = tuner.getKp();
    Ki_l = tuner.getKi();
    Kd_l = tuner.getKd();

    Serial.println("Tuning Left complete!");
    Serial.print("KpLeft: "); Serial.println(Kp_l);
    Serial.print("KiLeft: "); Serial.println(Ki_l);
    Serial.print("KdLeft: "); Serial.println(Kd_l);
    
    // Run autotune once on one wheel (Right)
    tuner.tune([](int pin){ return computeLinearVelocityRight(); }, 0, outputFuncRight,[](){ encoderRight.read(); });
    
    Kp_r = tuner.getKp();
    Ki_r = tuner.getKi();
    Kd_r = tuner.getKd();

    Serial.println("Tuning Right complete!");
    Serial.print("KpRight: "); Serial.println(Kp_r);
    Serial.print("KiRight: "); Serial.println(Ki_r);
    Serial.print("KdRight: "); Serial.println(Kd_r);

    tuning = false; // switch to PID control
  }
  else if (currentMillis - prevMillis >= 3*interval) {
    //delay(10000);
    prevMillis = currentMillis;
    velLeft = computeLinearVelocityLeft();
    velRight = computeLinearVelocityRight();
    newVelLeft = drivePIDL(velLeft, desVelL, 0.018*Kp_l, 0.018*Ki_l, 0.018*Kd_l);
    newVelRight = drivePIDR(velRight, desVelR, 0.022*Kp_r, 0.022*Ki_r, 0.022*Kd_r);
    int leftMotorCmd = motorVelToSpeedCommand(newVelLeft, leftMotorMax);
    int rightMotorCmd = motorVelToSpeedCommand(newVelRight, rightMotorMax);
    m.setM1Speed(leftMotorCmd);
    m.setM2Speed(rightMotorCmd);
    Serial.print(velLeft);
    Serial.print(',');
    Serial.println(velRight);
    Serial.print(leftMotorCmd);
    Serial.print(',');
    Serial.println(rightMotorCmd);
  }

}

// ================== PID FUNCTION ==================
double drivePIDL(double curr, double setpoint, double kp, double ki, double kd) {

    double rateErrorL;
    double errorL;
    unsigned long currentTimeL;
    unsigned long elapsedTimeL;
    cumErrorL = 0;
    
    currentTimeL = millis();                               //get current time
    elapsedTimeL = (double)(currentTimeL - priorTimeL);      // compute elasped time for this control period

    errorL = setpoint - curr;                                   // Error
    cumErrorL += errorL*elapsedTimeL;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorL>maxErr)
    cumErrorL = maxErr;
    else if (cumErrorL<-1*maxErr)
      cumErrorL = -1*maxErr;

    rateErrorL = (errorL-lastSpeedErrorL)/elapsedTimeL;      // Derivative Error

    double outL = kp*errorL+ki*cumErrorL+kd*rateErrorL;   // PID output
    //Serial.println(outL);
    lastSpeedErrorL = errorL;                              // remember current error
    priorTimeL = currentTimeL;                             // remember current time
    return outL;                                           // return the needed motor speed. 
}
double drivePIDR(double curr, double setpoint, double kp, double ki, double kd) {

    double rateErrorR;
    double errorR;
    unsigned long currentTimeR;
    unsigned long elapsedTimeR;
    cumErrorR = 0;
    
    currentTimeR = millis();                               //get current time
    elapsedTimeR = (double)(currentTimeR - priorTimeR);      // compute elasped time for this control period

    errorR = setpoint - curr;                                   // Error
    cumErrorR += errorR*elapsedTimeR;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorR>maxErr)
    cumErrorR = maxErr;
    else if (cumErrorR<-1*maxErr)
      cumErrorR = -1*maxErr;

    rateErrorR = (errorR-lastSpeedErrorR)/elapsedTimeR;      // Derivative Error

    double outR = kp*errorR+ki*cumErrorR+kd*rateErrorR;   // PID output
    //Serial.println(outR);
    lastSpeedErrorR = errorR;                              // remember current error
    priorTimeR = currentTimeR;                             // remember current time
    return outR;                                           // return the needed motor speed. 
}

// ================== MOTOR MAPPING ==================
int motorVelToSpeedCommand(double Vel, double maxVel) {
  Vel = constrain(Vel, -1 * maxVel, maxVel);
  return map(Vel, -1 * maxVel, maxVel, -400, 400);

}
