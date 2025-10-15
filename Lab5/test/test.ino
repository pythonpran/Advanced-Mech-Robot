#include "pid-autotune.h"

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

void outputFunc(double x) {
  analogWrite(11, x);
}

void setup() {
    Serial.begin(115200);

    tuner.setConstrains(0, 255);
    tuner.setTargetValue(100);

    tuner.tune(analogRead, A0, outputFunc);

    Serial.println(tuner.getKp());
    Serial.println(tuner.getKi());
    Serial.println(tuner.getKd());
}

void loop() {
    // Your loop code here
}
