#ifndef _RBFC_H_
#define _RBFC_H_

#include "mpu6050.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include "flysky_fs16x.h"

struct gyro_t {
	double angleX;
	double angleY;
	double angleZ;
	double refNose;
};

struct accel_t {
	double acc_roll;
	double acc_pitch;
	double acc_yaw;
};

struct pidcontrol_t {
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

class RbosDrone 
{
	public:
		RbosDrone();
		~RbosDrone();

		void init();
		void printImuData(void);
		void do_things();
	private:
		int receiver_input[5];
		int corrected_input[5];
		FlySky flysky;

		MPU6050 mpu6050;
		
		gyro_t gyro;
		accel_t accel;
		pidcontrol_t pidcontrol;
};

#endif
