#ifndef _RBFC_H_
#define _RBFC_H_

#include <stdio.h>
//#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
//#include <zephyr/logging/log.h>

#include "mpu6050.h"
#include "flysky_fs16x.h"
//#include "zsdcard.h"

#define FLYSKY_SAMPLE_TIME_MS		4
#define BMP390_SAMPLE_TIME_MS		5
#define DRONE_SAMPLE_TIME_MS		50
//#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))

//LOG_MODULE_REGISTER(gnss_sample, CONFIG_GNSS_LOG_LEVEL);

/*
#define chan1Pin 35///Roll
#define chan2Pin 34///Pitch
#define chan3Pin 39///Throttle
#define chan4Pin 36///Yaw
#define motorFRPin 14
#define motorBRPin 27
#define motorBLPin 26
#define motorFLPin 25
//*/

enum {roll, pitch, yaw};

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
  double setpoint[3];     
  double max[3];            
  double out[3];          
  double pOut[3];           
  double iOut[3];         
  double dOut[3];           
  double pGain[3];
  double iGain[3];
  double dGain[3];
  double pError[3];                                                                     
  double iError[3];
  double dError[3];
  double pMem[3];
  double iMem[3];                                            
  double dMem[3];
  double pid_error_temp;                                      
}; 

struct motor_t
{
	const pwm_dt_spec servo;// = PWM_DT_SPEC_GET(DT_ALIAS(servo0));
	const uint32_t min_pulse;// = DT_PROP(DT_ALIAS(servo0), min_pulse);
	const uint32_t max_pulse;// = DT_PROP(DT_ALIAS(servo0), max_pulse);
};

class RbosDrone 
{
	public:
		RbosDrone();
		~RbosDrone();

		void init();
		void printImuData(void);
		//navigation_data *getNavigationData(void);
		static void flysky_work_handler(k_work *);
		static void flysky_timer_handler(k_timer *);
		
		static void drone_work_handler(k_work *);
		static void drone_timer_handler(k_timer *);
		
	//*	
		static void bmp390_work_handler(k_work *);
		static void bmp390_timer_handler(k_timer *);
	//*/	
		//static void gnss_data_cb(const device *, const gnss_data *);
		//static void gnss_satellites_cb(const device *, const gnss_satellite *, uint16_t);
		const motor_t *getFrontRight(void);
		const motor_t *getBackRight(void);
		const motor_t *getBackLeft(void);
		const motor_t *getFrontLeft(void);
		int doSDCardThings(void);
	private:
//		SDCard sdcard;

		static const motor_t front_right;	
		static const motor_t back_right;
		static const motor_t back_left;
		static const motor_t front_left;
	
		static const device *bmp390;
		static sensor_value pressure;
		void capturePressure(sensor_value *);

		int startTriggeredBmp390(void);
		static void handle_bmp390_drdy(const struct device *, const struct sensor_trigger *);
		static int process_bmp390(const struct device *);
		static RbosDrone *pThis;

		static FlySky flysky;
		static flysky_data_t flysky_data;
		
		//static navigation_data nav_data;
		//static gnss_satellite satellites;

		//k_mutex gnss_mutex;
		k_mutex pressure_mutex;

		MPU6050 mpu6050;
		
		gyro_t gyro;
		accel_t accel;
		pidcontrol_t pidcontrol;
		
		void imuInit(void);
		void startWorkers(void);
		void gnssInit(void);
		
		void updateMotors(void);
		void doImuThings(void);
		void doGnssThings(void);
		void doFlySkyThings(void);
		void calculatePID(void);
		void doDroneThings();
		
		//void gnssCapture(navigation_data *);
		//void gnssPrint(navigation_data *);
};

#endif
