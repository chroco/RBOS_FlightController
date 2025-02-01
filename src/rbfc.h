#ifndef _RBFC_H_
#define _RBFC_H_

#include <stdio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#include "RBPID.h"

#include "mpu6050.h"
#include "flysky_fs16x.h"

#define FLYSKY_SAMPLE_TIME_MS		4
#define BMP390_SAMPLE_TIME_MS		5
#define DRONE_SAMPLE_TIME_MS		4
#define DRONE_PRINT_TIME_MS		  20

enum {roll, pitch, yaw};

struct motor_t
{
	const pwm_dt_spec servo;
	const uint32_t min_pulse;
	const uint32_t max_pulse;
};

class RbosDrone 
{
	public:
		RbosDrone();
		~RbosDrone();

		void init();
		void printImuData(void);
		static void flysky_work_handler(k_work *);
		static void flysky_timer_handler(k_timer *);
		
		static void drone_work_handler(k_work *);
		static void drone_timer_handler(k_timer *);
		
    static void bmp390_work_handler(k_work *);
		static void bmp390_timer_handler(k_timer *);
		
    static void print_work_handler(k_work *);
		static void print_timer_handler(k_timer *);

    void printExecutionTimeInUs(void);
    
    const motor_t *getFrontRight(void);
		const motor_t *getBackRight(void);
		const motor_t *getBackLeft(void);
		const motor_t *getFrontLeft(void);
	private:
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
		
		k_mutex pressure_mutex;
		k_mutex esc_mutex;

		MPU6050 mpu6050;
		gyro_t gyro;
		accel_t accel;
    PID_t pid;
    kalman_t kalman;

    bool motor_start;
		
    static uint64_t motor_input_us[4];
		
		static timing_t start_time;  
    static timing_t stop_time;  
    static uint64_t total_us;
    
    void imuInit(void);
		void startWorkers(void);
		void gnssInit(void);
		
		void updateMotors(void);
		void updateImu(void);
		void updateFlySky(void);
		void calculatePID(void);
    void updateKalman(void);
    void updatePressure(void);
		void doDroneThings(void);
};

#endif
