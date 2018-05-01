#include "LineStepper.h"
#include "GY25.h"
#include <PID_v1.h>
#include "BucketStepCounter.h"
#include "WifiTelnetServer.h"
WifiTelnetServer* WifiTelnetServer::instance = NULL;

const int MOTOR_DIR = 14;
const int MOTOR_STEP = 27;
const int MOTOR_SLEEP = 26;

const int MOTOR2_SLEEP = 32;
const int MOTOR2_STEP = 33;
const int MOTOR2_DIR = 25;

const uint16_t MOTOR_MAX_RPM = 90;
const uint16_t MOTOR_RESOLUTION = 800; // steps per rotation; this assumes a sub-step sampling (drv8834) of 4

WifiTelnetServer server(80);
#define Serial server

bool dirIsHigh = false;
uint32_t systemStartTime = 0;
uint32_t lastOutputTime = 0;
double steppedFrequency = 0;

BucketStepCounter stepCounter(20, 50);
int32_t stepCount = 0;

LineStepper stepper;
LineStepper stepper2;

GY25 gy25;

double pidSetpoint = 90, pidInput = 90, pidOutput;
double Kp=0.38, Ki=0.0, Kd=0.000;
PID myPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

bool motorsActive = false;

void outputPin(int num)
{
  digitalWrite(num, LOW);
  pinMode(num, OUTPUT);
}

// TODO this works?
#define SIGN(x) x >= 0 ? 1 : -1

static inline double sign(double x) {
  return x >= 0 ? 1 : -1;
}

static double inline clip(double z, double max)
{
  if (abs(z) <= max)
    return z;

  return max * sign(z);
}
  
void setup() {
  Serial.begin(115200);
  Serial.println("Balance start");

  outputPin(MOTOR_DIR);
  outputPin(MOTOR_SLEEP);

  outputPin(MOTOR2_DIR);
  outputPin(MOTOR2_SLEEP);

  stepper.setup(MOTOR_STEP);
  stepper2.setup(MOTOR2_STEP);
  
  gy25.init();

  server.startServer();
  server.start("Telnet");

  myPID.SetOutputLimits(-1, 1);
  myPID.SetSampleTime(10);

  systemStartTime = millis();

  myPID.SetMode(AUTOMATIC);

  Serial.println("Setup done");
}

void loop()
{
  bool hasNewOri = gy25.drive();

  if (hasNewOri) {
    float ori = gy25.getRoll() + 5.5f;

    if (ori > 45 && ori < 135) {
      if (!motorsActive) {
        digitalWrite(MOTOR_SLEEP, HIGH);
        digitalWrite(MOTOR2_SLEEP, HIGH);
        delay(1);
  
        motorsActive = true;
      }

      int32_t stepsNow = stepper.getCurrentSteps(true) * sign(steppedFrequency);
      int32_t bucketedSteps = stepCounter.addSteps(stepsNow);
      stepCount += stepsNow;
      double off = bucketedSteps / (double)MOTOR_RESOLUTION;
      off = clip(off, 1);
      pidInput = ori;// - off;
    
      bool hasNewPid = myPID.Compute();

      uint32_t now = millis();
      if (now - lastOutputTime > 300) {
      //if (abs(pidOutput) > 0 && abs(pidOutput) < 0.0001) {
        //Serial.println("ori "+String(ori)+" off "+String(off)+" (steps "+String(stepCount)+" 1s "+String(bucketedSteps)+") > input "+String(pidInput)+" > output "+String(pidOutput));
        lastOutputTime = now;
      }

      if (hasNewPid) {
        double frequency = pidOutput * MOTOR_MAX_RPM / 60.0 * MOTOR_RESOLUTION;

        if (frequency != steppedFrequency) {
          digitalWrite(MOTOR_DIR, frequency >= 0 ? LOW : HIGH);
          digitalWrite(MOTOR2_DIR, frequency < 0 ? LOW : HIGH); // inverted

          delayMicroseconds(1);
  
          stepper.setFrequency(frequency);
          stepper2.setFrequency(frequency);

          steppedFrequency = frequency;
        }
      }
    } else {
      stepper.setFrequency(0);
      stepper2.setFrequency(0);
      steppedFrequency = 0;

      if (motorsActive) {
        digitalWrite(MOTOR_SLEEP, LOW);
        digitalWrite(MOTOR2_SLEEP, LOW);
  
        motorsActive = false;
      }
    }
  }

  stepper.drive(true);
  stepper2.drive();
  
  delayMicroseconds(10);
}
