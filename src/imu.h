#ifndef _IMU_H_
#define _IMU_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/timing/timing.h>

#define SENSOR_ARRAY_SIZE 3

typedef struct sensor_value sensor_value_t;

class Imu
{
	public:
		Imu();
		~Imu();

		const char *now_str(void);
		int printCycleTime(void);
		int printImuData(sensor_value_t  *, sensor_value_t *, sensor_value_t *, int);
		int printImuData(void);
		int getImuData(sensor_value_t  *, sensor_value_t *, sensor_value_t *, int);
		//int getTemperature(struct sensor_value &);

	private:
		static sensor_value_t temperature;
		static sensor_value_t accel[SENSOR_ARRAY_SIZE];
		static sensor_value_t gyro[SENSOR_ARRAY_SIZE];

		const struct device *const mpu6050;

		int startTriggeredImu(void);
		static void handle_mpu6050_drdy(const struct device *, const struct sensor_trigger *);
		static int process_mpu6050(const struct device *);
};

#endif
