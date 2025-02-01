#include "RBPID.h"
#include <Arduino.h>
#include <ESP32Servo.h>
#include "MPU6050_tockn_RB.h"
#include <math.h>

void getVoltageCompensation(voltMeter_t &meter)
{
  meter.batteryVoltage = double(analogReadMilliVolts(voltMeterPin)) / 1000.0 * 11.0;
  //meter.analogIn = analogRead(voltMeterPin);
  //meter.TEMP = map(meter.analogIn, 0, 4095, 0, 8);
  //meter.TEMP = map(meter.analogIn, 0, 3300, 0, 8);
  //meter.batteryVoltage = meter.TEMP * 21.0 / 10;
}

void setPID(PID_t &pid, const char opt)
{
  switch(toupper(opt))
  {
    case 'R':
      for (int i = 0; i < 3; ++i)
      {
        pid.prevErrorRate[i] = 0;
        pid.prevITermRate[i] = 0;
      }
      for (int i = 0; i < 2; ++i)
      {
        pid.prevErrorAngle[i] = 0;
        pid.prevITermAngle[i] = 0;
      }
      break;
    case 'S':
      pid.PRateGain[0] = 1.80;
      pid.PRateGain[1] = pid.PRateGain[0];
      pid.PRateGain[2] = 1.80;

      pid.IRateGain[0] = 0.005;
      pid.IRateGain[1] = pid.IRateGain[0];
      pid.IRateGain[2] = 0.005;

      pid.DRateGain[0] = 0.008;
      pid.DRateGain[1] = pid.DRateGain[0];
      pid.DRateGain[2] = 0.008;

      pid.PAngleGain[0] = 3.0;
      pid.PAngleGain[1] = pid.PAngleGain[0];

      pid.IAngleGain[0] = 2.0;
      pid.IAngleGain[1] = pid.IAngleGain[0];

      pid.DAngleGain[0] = 0.0;
      pid.DAngleGain[1] = pid.DAngleGain[0];
      break;
    default:
      break;
  }
}

void getIMU(MPU6050 mpu6050, gyro_t &gyro, accel_t &accel)
{
  mpu6050.update();
  gyro.rotationRate[0] = double(mpu6050.getGyroX());
  gyro.rotationRate[1] = double(mpu6050.getGyroY());
  gyro.rotationRate[2] = double(mpu6050.getGyroZ());

  accel.acc[0] = double(mpu6050.getAccX());
  accel.acc[1] = double(mpu6050.getAccY());
  accel.acc[2] = double(mpu6050.getAccZ());

  accel.ang[0] = double(mpu6050.getAccAngleX());
  accel.ang[1] = double(mpu6050.getAccAngleY());
}

void kalman_1d(kalman_t &kalman, double KalmanState, double KalmanUncertainty, double KalmanInput, double KalmanMeasurement)
{
  double KalmanGain;
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState+KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  kalman.kalmanOutput[0] = KalmanState; 
  kalman.kalmanOutput[1] = KalmanUncertainty;
}

void calculate_pid(PID_t &myPID, kalman_t myKalman, gyro_t myGyro, accel_t myAccel)
{
  for (int i = 0; i < 2; ++i)
  {
    myPID.errorAngle[i] = myPID.desiredAngle[i] - myKalman.kalmanAngle[i];
    pid_equation(myPID, myPID.errorAngle[i], myPID.PAngleGain[i], myPID.IAngleGain[i], myPID.DAngleGain[i], myPID.prevErrorAngle[i], myPID.prevITermAngle[i]);
    myPID.desiredRate[i] = myPID.PIDReturn[0];
    myPID.prevErrorAngle[i] = myPID.PIDReturn[1];
    myPID.prevITermAngle[i] = myPID.PIDReturn[2];
  }

  for (int i = 0; i < 3; ++i)
  {
    myPID.errorRate[i] = myPID.desiredRate[i] - myGyro.rotationRate[i];
    pid_equation(myPID, myPID.errorRate[i], myPID.PRateGain[i], myPID.IRateGain[i], myPID.DRateGain[i], myPID.prevErrorRate[i], myPID.prevITermRate[i]);
    myPID.final[i] = myPID.PIDReturn[0];
    myPID.prevErrorRate[i] = myPID.PIDReturn[1];
    myPID.prevITermRate[i] = myPID.PIDReturn[2];
  }
}

void pid_equation(PID_t &pid, double Error, double P , double I, double D, double PrevError, double PrevIterm)
{
  double Dterm, PIDOutput, Pterm, Iterm;
  Pterm = P * Error;
  Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
  if (Iterm > MAX_PID)
  {
    Iterm = MAX_PID;
  }
  else if (Iterm < -MAX_PID)
  {
    Iterm = -MAX_PID;
  }
  Dterm = D * (Error - PrevError);// / 0.004;
  PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > MAX_PID)
  {
    PIDOutput = MAX_PID;
  }
  else if (PIDOutput < -MAX_PID)
  {
    PIDOutput = -MAX_PID;
  }
  pid.PIDReturn[0] = PIDOutput;
  pid.PIDReturn[1] = Error;
  pid.PIDReturn[2] = Iterm;
}

void correctInput(int receiver_input[], int corrected_input[])
{
  for (int i = 1; i <= 3; ++i)
  {
    if (receiver_input[i] > 1492 && receiver_input[i] < 1508)
    {
      corrected_input[i] = 0;
    }
    else
    {
      //corrected_input[i] = (0.15) * (receiver_input[i] - 1500);
      corrected_input[i] = (45 / 50.0) * (receiver_input[i] - 1500) / 10.0;
    }
    switch (i)
    {
      case 1:
      case 2:
      case 3:
        corrected_input[i] = -corrected_input[i];
        break;
      default:
        break;
    }
  }
}

void printReport(voltMeter_t meter, PID_t pid, kalman_t kalman, int ESCtimer[], accel_t accel, gyro_t gyro, int corrected_input[], int loopTime)
{
  /*
  Serial.printf("%0.2fl, %0.2fl, %0.2fl, ", gyro.rotationRate[0], gyro.rotationRate[1], gyro.rotationRate[2]);
  Serial.printf("%0.2f, %0.2f, %0.2f, ", pid.desiredRate[0], pid.desiredRate[1], pid.desiredRate[2]);
  Serial.printf("%0.2f, %0.2f, ", accel.ang[0], accel.ang[1]);
  Serial.printf("%0.2f, %0.2f, %0.2f, ", pid.final[0], pid.final[1], pid.final[2]);
  Serial.printf("%0.2f, %0.2f, ", kalman.kalmanAngle[0], kalman.kalmanAngle[1]);
  Serial.printf("%0.2f, %0.2f, %0.2f, ", pid.desiredAngle[0], pid.desiredAngle[1], pid.desiredRate[2]);
  Serial.printf("%d, %d, %d, %d, ", ESCtimer[0]
                                  , ESCtimer[1]
                                  , ESCtimer[2]
                                  , ESCtimer[3]);
  Serial.printf("%d\n", loopTime);
  */
  Serial.printf("%0.2f, ", meter.batteryVoltage);
  Serial.printf("\n");
}
