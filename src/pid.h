#ifndef _RB_PID_H_
#define _RB_PID_H_

#pragma once
//#include <ESP32Servo.h>
#include "mpu6050.h"
#include <math.h>

#define chan1Pin 35///Roll
#define chan2Pin 34///Pitch
#define chan3Pin 39///Throttle
#define chan4Pin 36///Yaw
#define motorFRPin 14
#define motorBRPin 27
#define motorBLPin 26
#define motorFLPin 25

#define MAX_ROLL 45
#define MAX_YAW 45

///Data Structs
struct accel_t
{
  double vector;
  double acc_roll;
  double acc_pitch;
};

struct gyro_t
{
  double roll;
  double pitch;
  double yaw;
  double currNose;
  double refNose;
  double currGyro[4];
  double prevGyro[4];
};

struct PID_t
{
  double setpoint[4];
  double max[4];
  double out[4];
  double pOut[4];
  double iOut[4];
  double dOut[4];
  double pGain[4];
  double iGain[4];
  double dGain[4];
  double pError[4];
  double iError[4];
  double dError[4];
  double pMem[4];
  double iMem[4];
  double dMem[4];
  double pid_error_temp;
};

void initPID(PID_t pid);
void calculate_pid(PID_t pid, double gyro_input[4], gyro_t gyro, int corrected_input[5]);
void correctInput(int receiver_input[5], int corrected_input[5]);
void printReport(PID_t pid_calc, int ESCtimer[], gyro_t gyro, int corrected_input[]);

#endif
