#ifndef RB_PID_H_
#define RB_PID_H_

#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "MPU6050_tockn_RB.h"
#include <math.h>

#define chan1Pin 35///Roll
#define chan2Pin 34///Pitch
#define chan3Pin 39///Throttle
#define chan4Pin 36///Yaw
#define motorFRPin 14
#define motorBRPin 27
#define motorBLPin 26
#define motorFLPin 25
#define voltMeterPin 13

#define MAX_THROTTLE 1800
#define MAX_PID 200
#define MAX_YAW 200

///Data Structs
struct accel_t
{
  double acc[3];
  double ang[2];
};

struct gyro_t
{
  double rotationRate[3];
  double rateCalibration[3];
};

struct kalman_t
{
  double kalmanAngle[2];
  double kalmanUncertaintyAngle[2];
  double kalmanOutput[2];
};

struct PID_t
{
  double PIDReturn[3];
  double final[3];
  double desiredRate[3];
  double desiredAngle[3];
  double errorRate[3];
  double prevErrorRate[3];
  double prevITermRate[3];
  double PRateGain[3];
  double IRateGain[3];
  double DRateGain[3];
  double errorAngle[3];
  double prevErrorAngle[3];
  double prevITermAngle[3];
  double PAngleGain[3];
  double IAngleGain[3];
  double DAngleGain[3];
};

struct voltMeter_t
{
  uint32_t analogIn;
  double batteryVoltage;
  int ESCcompensation;
  int TEMP;
};

void getVoltageCompensation(voltMeter_t &meter);
void setPID(PID_t &pid, const char opt);
void calculate_pid(PID_t &myPID, kalman_t myKalman, gyro_t myGyro, accel_t myAccel);
void getIMU(MPU6050 IMU, gyro_t &gyro, accel_t &accel);
void kalman_1d(kalman_t &kalman, double KalmanState, double KalmanUncertainty, double KalmanInput, double KalmanMeasurement);
void pid_equation(PID_t &pid, double Error, double P , double I, double D, double PrevError, double PrevIterm);
void correctInput(int receiver_input[], int corrected_input[]);
void printReport(voltMeter_t meter, PID_t pid, kalman_t kalman, int ESCtimer[], accel_t accel, gyro_t gyro, int corrected_input[], int loopTime);


#endif