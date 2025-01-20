#ifndef _RBFC_H_
#define _RBFC_H_

#include <stdio.h>
#include "mpu6050.h"
#include <zephyr/drivers/gnss.h>
#include "flysky_fs16x.h"

#define FLYSKY_SAMPLE_TIME_MS		4
#define DRONE_SAMPLE_TIME_MS		50
#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

//LOG_MODULE_REGISTER(gnss_sample, CONFIG_GNSS_LOG_LEVEL);

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
		
		static void flysky_work_handler(k_work *);
		static void flysky_timer_handler(k_timer *);
		
		static void drone_work_handler(k_work *);
		static void drone_timer_handler(k_timer *);
		navigation_data nav_data;
		gnss_satellite satellites;
	private:
		static RbosDrone *pThis;
		static FlySky flysky;
		static flysky_data_t flysky_data;

		k_mutex gps_mutex;

		MPU6050 mpu6050;
		
		gyro_t gyro;
		accel_t accel;
		pidcontrol_t pidcontrol;
		
		void imuInit(void);
		void gnssInit(void);
		void timerInit(void);
};

#endif
