#include "rbpid.h"
#include "mpu6050.h"
#include <math.h>

void initPID(PID_t pid)
{
  /*
  pid->pGain[1] = 1.3;
  pid->iGain[1] = 0.04;
  pid->dGain[1] = 18.0;
  pid->max[1] = 400;

  pid->pGain[2] = pid->pGain[1];
  pid->iGain[2] = pid->iGain[1];
  pid->dGain[2] = pid->dGain[1];
  pid->max[2] = pid->max[1];

  pid->pGain[3] = 4.0;
  pid->iGain[3] = 0.02;
  pid->dGain[3] = 0.0;
  pid->max[3] = 400;

  memset(pid->iMem, 0, 4);

  pid.pGain[1] = 1.3;
  pid.iGain[1] = 0.04;
  pid.dGain[1] = 18.0;
  pid.max[1] = 45;

  pid.pGain[2] = pid.pGain[1];
  pid.iGain[2] = pid.iGain[1];
  pid.dGain[2] = pid.dGain[1];
  pid.max[2] = pid.max[1];

  pid.pGain[3] = 4.0;
  pid.iGain[3] = 0.02;
  pid.dGain[3] = 0.0;
  pid.max[3] = 45;

  memset(pid.pMem, 0, 4);
  memset(pid.pError, 0, 4);
  memset(pid.iMem, 0, 4);
  memset(pid.iError, 0, 4);
  memset(pid.dMem, 0, 4);
  memset(pid.dError, 0, 4);
  */
}

void calculate_pid(PID_t pid, double gyro_input[4], gyro_t gyro, int corrected_input[5])
{
  ////////FOR LOOP!
  /*
  for (int i = 1; i <=3; ++i)///1 = roll, 2 = pitch, 3 = yaw
  {
    pid->pid_error_temp = gyro_input[i] - corrected_input[i];
    pid->iMem[i] += pid->iGain[i] * pid->pid_error_temp;
    if(pid->iMem[i] > pid->max[i])pid->iMem[i] = pid->max[i];
    else if(pid->iMem[i] < pid->max[i] * -1)pid->iMem[i] = pid->max[i] * -1;

    pid->out[i] = pid->pGain[i] * pid->pid_error_temp + pid->iMem[i] + 
									pid->dGain[i] * (pid->pid_error_temp - pid->dError[i]);
    if(pid->out[i] > pid->max[i])
			pid->out[i] = pid->max[i];
    else if(pid->out[i] < pid->max[i] * -1)
			pid->out[i] = pid->max[i] * -1;

    pid->dError[i] = pid->pid_error_temp;
  }
//*/

/*
  for (int i = 1; i <=3; ++i)///1 = roll, 2 = pitch, 3 = yaw
  {
    pid.pid_error_temp = gyro_input[i] - corrected_input[i];

    pid.iMem[i] += pid.iGain[i] * pid.pid_error_temp;
    if(pid.iMem[i] > pid.max[i])
			pid.iMem[i] = pid.max[i];
    else if(pid.iMem[i] < pid.max[2] * -1)
			pid.iMem[i] = pid.max[i] * -1;

    pid.out[i] = pid.pGain[i] * pid.pid_error_temp + pid.iMem[i] + 
								 pid.dGain[i] * (pid.pid_error_temp - pid.dError[i]);
    if(pid.out[i] > pid.max[i])
			pid.out[i] = pid.max[i];
    else if(pid.out[i] < pid.max[i] * -1)
			pid.out[i] = pid.max[i] * -1;

    pid.dError[i] = pid.pid_error_temp;
  }
//*/
}

void correctInput(int receiver_input[5], int corrected_input[5])///corrected_input = -45 to 45
{
  for (int i = 1; i <= 4; ++i)
  {
    if (receiver_input[i] > 1492 && receiver_input[i] < 1508)
    {
      corrected_input[i] = 0;
    }
    else
    {
      corrected_input[i] = (45 / 50.0) * (receiver_input[i] - 1500) / 10.0;
      //corrected_input[i] = receiver_input[i] - 1500;
    }
    switch (i)
    {
      case 1:
      case 2:
        corrected_input[i] = 0 - corrected_input[i];
        break;
      default:
        break;
    }
  }
}

void printReport(PID_t pid_calc, int ESCtimer[], gyro_t gyro, int corrected_input[])
{
  //Serial.printf("%0.2fl, %0.2fl, %0.2fl, ", gyro.currGyro[1], gyro.currGyro[2], gyro.currGyro[3]);
  //Serial.printf("%0.2fl, %0.2fl, %0.2fl, ", pid_calc.out[1], pid_calc.out[2], pid_calc.out[3]);
  /*
  Serial.printf("%0.1fl, %0.1fl, %0.1fl, %0.1fl, ", 0 - pid_calc.out[2] - pid_calc.out[1]
                                                  , 0 + pid_calc.out[2] - pid_calc.out[1]
                                                  , 0 + pid_calc.out[2] + pid_calc.out[1]
                                                  , 0 - pid_calc.out[2] + pid_calc.out[1]);
- pid_calc.out[3]
+ pid_calc.out[3]
- pid_calc.out[3]
+ pid_calc.out[3]
    for (int i = 1; i <= 4; ++i)
    {
      switch(i)
      {
        case 1:
          Serial.printf("FR: %d, ", ESCtimer[i]);
          break;
        case 2:
          Serial.printf("BR: %d, ", ESCtimer[i]);
          break;
        case 3:
          Serial.printf("BL: %d, ", ESCtimer[i]);
          break;
        case 4:
          Serial.printf("FL: %d, ", ESCtimer[i]);
          break;
        default:
          break;
      }
    }
  */
 //   Serial.println("");
}

/*
///Level Adjust
  gyro.pitch += gyro.currGyro[2] * 0.0000611;
  gyro.roll += gyro.currGyro[1] * 0.0000611;

  gyro.pitch -= gyro.roll * sin(gyro.yaw * 0.000001066);
  gyro.roll += gyro.pitch * sin(gyro.yaw * 0.000001066);

  gyro.roll = gyro.roll * 0.9996 + accel.acc_roll * 0.0004;
  gyro.pitch = gyro.pitch * 0.9996 + accel.acc_pitch * 0.0004;

  level_adj[1] = gyro.roll;
  level_adj[2] = gyro.pitch;

*/

/*
  if(!autoLevel){                                                          //If the quadcopter is not in auto-level mode
    level_adj[1] = 0;                                                 //Set the pitch angle correction to zero.
    level_adj[2] = 0;                                                  //Set the roll angle correcion to zero.
  }
*/

/*
  if (corrected_input[1] == 0 && corrected_input[2] == 0)
  {
    autoLevel = 0;
  }
  else
  {
    autoLevel = 1;
  }
*/
