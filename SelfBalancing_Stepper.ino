/* PID Self Balancing Robot
 * 
 * Author : Kwon Jin
 * Date : 2023.02.11
 * 
 */

#define DIR_L 10
#define STP_L 9

#define DIR_R 7
#define STP_R 8

#define LED 4
#define BT 6

#include <AccelStepper.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Wire.h"

#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;

//double Kp = 43;
//double Ki = 63.1;
//double Kd = 30.1;

double Kp = 16.9;
double Ki = 3.0;
double Kd = 600;

double error;
double previousError;

double aimAngle = 0.0;
double currentAngle;

double P_control, I_control, D_control;
double PID_control;
double t = 0.0001;

int offsetMotor1 = 30;
int offsetMotor2 = 0;
int angleOffset = 0.0;

MPU6050 mpu;

AccelStepper stepperL(AccelStepper::DRIVER, STP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STP_R, DIR_R);

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BT, INPUT);
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(STP_L, OUTPUT);
  pinMode(STP_R, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  stepperL.setCurrentPosition(0);
  stepperR.setCurrentPosition(0);

  stepperL.setMaxSpeed(1000.0);
  stepperL.setAcceleration(100.0);
  stepperL.setSpeed(50);
  stepperR.setMaxSpeed(1000.0);
  stepperR.setAcceleration(100.0);
  stepperR.setSpeed(50);

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  Serial.println("Initialzing ... ");

//  calibration();
}

void loop() {
  // put your main code here, to run repeatedly:
//   selfBalancing(PID());
//
//   //Serial.println(millis());
//  if(digitalRead(BT))
//  {
//    Stop();
//    delay(500);
//    Kp -= 0.1;
//    calibration();
//  }
  forward(10);
}

void calibration()
{
  digitalWrite(LED, HIGH);
  mpu.initialize();

  while(!digitalRead(BT)){}

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688); 
  
  if(!mpu.dmpInitialize())
  {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);

    digitalPinToInterrupt(INTERRUPT_PIN);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  digitalWrite(LED, LOW);
}

int PID()
{
  currentAngle = getPitch();
  error = errOffset(aimAngle - currentAngle);
  P_control = Kp * error;
  I_control += Ki * error * t;
  D_control = Kd * (error - previousError) / t;

  PID_control = P_control + I_control + D_control;
//  Serial.println(PID_control);
  //PID_control = constrain(PID_control, -500, 500);

  previousError = error;

  return PID_control;
}

double getPitch()
{
  if(mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.println(ypr[1] * 180/M_PI);
    return ypr[1] * 180/M_PI;
  }
}

void dmpDataReady()
{
  mpuInterrupt = true;
}

double errOffset(double err)
{
  if((err < angleOffset && err >= 0.0) || (err > -angleOffset && err <= 0.0))
    return err = 0.0;
  else 
    return err;
}

void selfBalancing(int val)
{
  if(val > 0)
    backward(val);
  else if (val < 0)
    forward(-val);

  else
    Stop();
}

void forward(int spd)
{
  stepperL.setSpeed(-spd);
  stepperR.setSpeed(spd);
  stepperL.runSpeed();
  stepperR.runSpeed();
}

void backward(int spd)
{

  stepperL.setSpeed(spd);
  stepperR.setSpeed(-spd);
  stepperL.runSpeed();
  stepperR.runSpeed();
}

void Stop()
{
  stepperL.stop();
  stepperR.stop();
}