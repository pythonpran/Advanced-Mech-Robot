#include <PID_v2.h>
#include <pid-autotune.h>
#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them
PID pid = PID();
pid_tuner tuner = pid_tuner(pid, 10, 1000000, pid_tuner::CLASSIC_PID);
#define PI 3.141592653589

double input, output, setpoint;
double Kp=2, Ki=5, Kd=1;

const int encoderLeftPinA = 8; 
const int encoderLeftPinB = 11;
 
const int encoderRightPinA = 16;
const int encoderRightPinB = 15;

Encoder encoderRight(encoderRightPinA,encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA,encoderLeftPinB);

int encoderResolution = 1440; // counts per rev
double d = 2.7559055; //wheel diameter in inches

int diffRight=0;
int diffLeft=0;
int posLeftCount = 0;
int posRightCount = 0;
int posLeftCountLast = 0;
int posRightCountLast = 0;
double posLeftRad = 0.0; // this will need to be converted to rad/sec
double posRightRad = 0.0; // this will need to be converted to rad/sec
double velLeft = 0; // this will be omegaLeft*d/2;
double velRight = 0; // this will be omegaRight*d/2 will be in inches per sec;
double newVelLeft = 0; // this will be omegaLeft*d/2;
double newVelRight = 0; // this will be omegaRight*d/2 will be in inches per sec;

// MOTOR LOOP CONSTANTS
double interval = 5; // 5 ms means 200Hz loop
unsigned long previousMillis = 0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
//PID_ATune tuner(&input, &output);

// Relay autotune settings
double step = 50;       // output step size
double noiseBand = 1;   // ignore small oscillations
int lookBack = 20;      // lookback time in seconds

void setup() {
  Serial.begin(115200);
  myPID.SetMode(MANUAL);       // turn off PID while tuning
 // tuner.SetNoiseBand(noiseBand);
  //tuner.SetOutputStep(step);
  //tuner.SetLookbackSec(lookBack);
  setpoint = 20;              // target speed (from encoder)
  tuner.setConstraints(0,400);
  tuner.setTargetValue(20);
  tuner.tune(velleft);
}

void loop() {
   // replace with your encoder function
  
   unsigned long currentMillis = millis();

      posRightCount = encoderRight.read(); 
      posLeftCount = encoderLeft.read();

   if (currentMillis - previousMillis >= interval){
      previousMillis = currentMillis;

     diffRight = posRightCount - posRightCountLast;
     posRightRad = ((2*PI)/encoderResolution)*(diffRight/interval)*1000; 

     diffLeft = posLeftCount - posLeftCountLast;
     posLeftRad = ((2*PI)/encoderResolution)*(diffLeft/interval)*1000; 
     
    
     velRight = (d/2)*posRightRad; // Now convert to get inches/sec (tangential velocity)
     velLeft = (d/2)*posLeftRad; // Same - Inches/sec

   }

//     input = velLeft; 
//  if (tuner.Runtime()) {       // tuning complete?
//    Kp = tuner.GetKp();
//    Ki = tuner.GetKi();
//    Kd = tuner.GetKd();
//    myPID.SetTunings(Kp, Ki, Kd);
//    myPID.SetMode(AUTOMATIC);
//    Serial.println("Tuning complete!");
//    Serial.print("Kp="); Serial.println(Kp);
//    Serial.print("Ki="); Serial.println(Ki);
//    Serial.print("Kd="); Serial.println(Kd);
//  }
//
//  myPID.Compute();
//  analogWrite(9, constrain(output, 0, 400));
}
