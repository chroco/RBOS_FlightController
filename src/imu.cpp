#include "imu.h"

/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

timing_t start_time;
timing_t end_time;
uint64_t total_cycles = 0;
uint64_t total_ns = 0;
uint64_t max_ns = 0;

K_MUTEX_DEFINE(max_ns_mutex);
K_MUTEX_DEFINE(imu_data_mutex);

sensor_value_t Imu::temperature;
sensor_value_t Imu::accel[3] = {0};
sensor_value_t Imu::gyro[3] = {0};

Imu::Imu() :
	mpu6050(DEVICE_DT_GET_ONE(invensense_mpu6050))
{
	startTriggeredImu();
}

Imu::~Imu()
{

}

int Imu::getImuData(
				sensor_value_t *ptemperature, sensor_value_t *paccel, sensor_value_t *pgyro, int size) {
	
	__ASSERT(size==SENSOR_ARRAY_SIZE, "Invalid size, got %d\n", size);

	k_mutex_lock(&imu_data_mutex, K_FOREVER);
	memcpy(ptemperature, &temperature, sizeof(sensor_value_t));	
	memcpy(paccel, accel, size * sizeof(sensor_value_t));	
	memcpy(pgyro, gyro, size * sizeof(sensor_value_t));	
	k_mutex_unlock(&imu_data_mutex);
	
	return 0;
}

int printCycleTime(void) {
/*
	//k_mutex_lock(&max_ns_mutex, K_FOREVER);
	//uint64_t temp_start_time = start_time;
	//uint64_t temp_end_time = end_time;
	//k_mutex_unlock(&max_ns_mutex);

	//total_cycles = timing_cycles_get(&temp_start_time, &temp_end_time);	
	total_ns = timing_cycles_to_ns(total_cycles);
	if(total_ns > max_ns) {
		max_ns = total_ns;
	}

//*/
	printf("%llu", total_ns);
	return 0;
}

int Imu::printImuData(void) {
//*
	printf("\n[%s]:\n"
				 "  temp %g Cel\n"
				 "  accel %f %f %f m/s/s\n"
				 "  gyro  %f %f %f rad/s\n",
				 now_str(),
				 sensor_value_to_double(&temperature),
				 sensor_value_to_double(&accel[0]),
				 sensor_value_to_double(&accel[1]),
				 sensor_value_to_double(&accel[2]),
				 sensor_value_to_double(&gyro[0]),
				 sensor_value_to_double(&gyro[1]),
				 sensor_value_to_double(&gyro[2]));
//*/

	return 0;
}

int Imu::printImuData(
				sensor_value_t *ptemperature, sensor_value_t *paccel, sensor_value_t *pgyro, int size) {
	__ASSERT(size==SENSOR_ARRAY_SIZE, "Invalid size, got %d\n", size);
//*
	printf("\n[%s]:\n"
				 "  temp %g Cel\n"
				 "  accel %f %f %f m/s/s\n"
				 "  gyro  %f %f %f rad/s\n",
				 now_str(),
				 sensor_value_to_double(ptemperature),
				 sensor_value_to_double(&paccel[0]),
				 sensor_value_to_double(&paccel[1]),
				 sensor_value_to_double(&paccel[2]),
				 sensor_value_to_double(&pgyro[0]),
				 sensor_value_to_double(&pgyro[1]),
				 sensor_value_to_double(&pgyro[2]));
//*/

	return 0;
}

int Imu::process_mpu6050(const struct device *dev) {
	
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
	} 

	return rc;
}

static struct sensor_trigger trigger;

void Imu::handle_mpu6050_drdy(const struct device *dev, const struct sensor_trigger *trig) {
	int rc = process_mpu6050(dev);
//*
	if (rc != 0) {
		printf("(%d)",rc);
		//printf("failure detected: %d\n", rc);
		//(void)sensor_trigger_set(dev, trig, NULL);
		//(void)sensor_trigger_set(dev, trig, handle_mpu6050_drdy);
	} //else {
		//printf(".");
	//}
//*/
}

int Imu::startTriggeredImu(void) {

	while (!device_is_ready(mpu6050)) 
	{
		printf("Device %s is not ready\n", mpu6050->name);
		k_msleep(1000);
	}

	trigger = (struct sensor_trigger) {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(mpu6050, &trigger, handle_mpu6050_drdy) < 0) {
		printf("Cannot configure trigger\n");
		return 0;
	}

	printk("Configured for triggered sampling.\n");

	// triggered runs with its own thread after exit 
	return 0;
}

const char *Imu::now_str(void) {
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

/*
	//	timing_init();
 // timing_start();

	end_time = timing_counter_get();
	total_cycles = timing_cycles_get(&start_time, &end_time);	
	total_ns = timing_cycles_to_ns(total_cycles);
	if(total_ns > max_ns) {
		k_mutex_lock(&max_ns_mutex, K_FOREVER);
		max_ns = total_ns;
		k_mutex_unlock(&max_ns_mutex);
	}
	start_time = end_time;
//*/

