#ifndef _RBFC_H_
#define _RBFC_H_

#include "mpu6050.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
//#include <ztest.h>
//#include "pid.h"

#define THROTTLE_NODE	DT_ALIAS(receiver0) // throttle
#define ROLL_NODE			DT_ALIAS(receiver1) // roll
#define PITCH_NODE		DT_ALIAS(receiver2) // pitch
#define YAW_NODE			DT_ALIAS(receiver3) // yaw

//enum {throttle, roll, pitch, yaw};

#define PWM_LOOPBACK_OUT_IDX 0
#define PWM_LOOPBACK_IN_IDX  1

#define PWM_LOOPBACK_NODE DT_INST(0, test_pwm_loopback)

#define PWM_LOOPBACK_IN_CTLR \
	DT_PWMS_CTLR_BY_IDX(PWM_LOOPBACK_NODE, PWM_LOOPBACK_IN_IDX)
#define PWM_LOOPBACK_IN_CHANNEL \
	DT_PWMS_CHANNEL_BY_IDX(PWM_LOOPBACK_NODE, PWM_LOOPBACK_IN_IDX)
#define PWM_LOOPBACK_IN_FLAGS \
	DT_PWMS_FLAGS_BY_IDX(PWM_LOOPBACK_NODE, PWM_LOOPBACK_IN_IDX)

struct pwm_t {
	const struct device *dev;
	uint32_t pwm;
	pwm_flags_t flags;
};

struct pwm_callback_data_t {
	uint32_t *buffer;
	size_t buffer_len;
	size_t count;
	int status;
	struct k_sem sem;
	bool pulse_capture;
};

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
		void printReceiver(void);
	private:
		int receiver_input[5];
		int corrected_input[5];
		
		//typedef void (*receiver_cb[4])(const struct device *, struct gpio_callback *, uint32_t);
		//receiver_cb cb[4];
		//static struct gpio_callback receiver_data[4];
		static struct gpio_callback throttle_data;
		static struct gpio_callback roll_data;
		static struct gpio_callback pitch_data;
		static struct gpio_callback yaw_data;
	
		static uint64_t throttle_time_ns; 
		static uint64_t roll_time_ns;
		static uint64_t pitch_time_ns;
		static uint64_t yaw_time_ns;
		
		int setupReceiver(const gpio_dt_spec *, gpio_callback *eceiver_dat, gpio_callback_handler_t);
		static void throttle_cb(const struct device *, struct gpio_callback *, uint32_t);
		static void roll_cb(const struct device *, struct gpio_callback *, uint32_t);
		static void pitch_cb(const struct device *, struct gpio_callback *, uint32_t);
		static void yaw_cb(const struct device *, struct gpio_callback *, uint32_t);

		static const struct gpio_dt_spec throttle;
		static const struct gpio_dt_spec roll;
		static const struct gpio_dt_spec pitch;
		static const struct gpio_dt_spec yaw;

		///These need to be changed to: receiver_input = {1 = roll, 2 = pitch, 3 = yaw, 4 = throttle}

		MPU6050 mpu6050;
		//static const struct gpio_dt_spec receiver[4];
/*
		static const struct gpio_dt_spec roll;
		static const struct gpio_dt_spec pitch; 
		static const struct gpio_dt_spec yaw;	
		static const struct gpio_dt_spec throttle;
//*/

		//static struct gpio_callback button_cb_data;
		gyro_t gyro;
		accel_t accel;
		pidcontrol_t pidcontrol;
};

#endif
