#include "rbfc.h"
#include "mpu6050.h"

RbosDrone *RbosDrone::pThis = NULL;

FlySky RbosDrone::flysky = FlySky();
flysky_data_t RbosDrone::flysky_data = {0};

navigation_data RbosDrone::nav_data = {0};	
gnss_satellite RbosDrone::satellites = {0};

const device *RbosDrone::bmp390 = DEVICE_DT_GET_ONE(bosch_bmp390);;
sensor_value RbosDrone::pressure = {0};;

RbosDrone::RbosDrone() :
	mpu6050(MPU6050()),
	gyro{0},
	accel{0},
	pidcontrol{0}
{
	pThis = this;

	k_mutex_init(&gnss_mutex);
	k_mutex_init(&pressure_mutex);

//*
  pidcontrol.pGain[roll] = 2.0;
  pidcontrol.iGain[roll] = 0.02;
  pidcontrol.dGain[roll] = 18.0;
  pidcontrol.max[roll]   = 400;
    ///pitch
  pidcontrol.pGain[pitch] = pidcontrol.pGain[roll];
  pidcontrol.iGain[pitch] = pidcontrol.iGain[roll];
  pidcontrol.dGain[pitch] = pidcontrol.dGain[roll];
  pidcontrol.max[pitch]   = pidcontrol.max[roll];
    ///Yaw
  pidcontrol.pGain[yaw] = 1.0;
  pidcontrol.iGain[yaw] = 0.00;
  pidcontrol.dGain[yaw] = 0.0;
  pidcontrol.max[yaw]   = 400;
//*/
	k_msleep(5000);

	init();
}

RbosDrone::~RbosDrone() 
{

}

void RbosDrone::printImuData(void)
{
	mpu6050.printImuData();
}

navigation_data *RbosDrone::getNavigationData(void)
{
	return &nav_data;	
}

int gnss_dump_nav_data(char *str, uint16_t strsize, const struct navigation_data *nav_data)
{
	int ret;
	const char *fmt = "navigation_data: {lat: %s%lli.%09lli, lon: %s%lli.%09lli, "
			  "bearing %u.%03u, speed %u.%03u, alt: %s%i.%03i}";
	const char *lat_sign = nav_data->latitude < 0 ? "-" : "";
	const char *lon_sign = nav_data->longitude < 0 ? "-" : "";
	const char *alt_sign = nav_data->altitude < 0 ? "-" : "";

	ret = snprintk(str, strsize, fmt,
		       lat_sign,
		       llabs(nav_data->latitude) / 1000000000,
		       llabs(nav_data->latitude) % 1000000000,
		       lon_sign,
		       llabs(nav_data->longitude) / 1000000000,
		       llabs(nav_data->longitude) % 1000000000,
		       nav_data->bearing / 1000, nav_data->bearing % 1000,
		       nav_data->speed / 1000, nav_data->speed % 1000,
		       alt_sign, abs(nav_data->altitude) / 1000, abs(nav_data->altitude) % 1000);

	return (strsize < ret) ? -ENOMEM : 0;
}

void RbosDrone::gnssPrint(navigation_data *pnav_data)
{
	char nav_str[200] = {0};

	gnss_dump_nav_data(nav_str,100,pnav_data);
	printf(" <%s> ", nav_str);
/*	
	printf(" <%llu %llu %u %u %u> ",
		pnav_data->latitude,
		pnav_data->longitude,
		pnav_data->bearing,
		pnav_data->speed,
		pnav_data->altitude
	);
//*/
}

void RbosDrone::gnssCapture(navigation_data *pnav_data)
{
	k_mutex_lock(&gnss_mutex, K_FOREVER);
	memcpy(pnav_data, &nav_data, sizeof(navigation_data)); 
	k_mutex_unlock(&gnss_mutex);
}

void RbosDrone::imuInit(void)
{
	mpu6050.begin();
  mpu6050.calcGyroOffsets(true);// TODO: figure out what this does
}

void RbosDrone::flysky_work_handler(k_work *work)
{
	flysky.capturePulses(&flysky_data);
}

K_WORK_DEFINE(flysky_work, RbosDrone::flysky_work_handler);

void RbosDrone::flysky_timer_handler(k_timer *dummy)
{
  k_work_submit(&flysky_work);
}

K_TIMER_DEFINE(flysky_timer, RbosDrone::flysky_timer_handler, NULL);

void RbosDrone::bmp390_work_handler(k_work *work)
{
	int rc = sensor_sample_fetch(bmp390);

	if (rc == 0) 
	{
		rc = sensor_channel_get(bmp390, SENSOR_CHAN_PRESS, &pressure);
	}
}

K_WORK_DEFINE(bmp390_work, RbosDrone::bmp390_work_handler);

void RbosDrone::bmp390_timer_handler(k_timer *dummy)
{
  k_work_submit(&bmp390_work);
}

K_TIMER_DEFINE(bmp390_timer, RbosDrone::bmp390_timer_handler, NULL);


void RbosDrone::drone_work_handler(k_work *work)
{
	pThis->doDroneThings();
}

K_WORK_DEFINE(drone_work, RbosDrone::drone_work_handler);

void RbosDrone::drone_timer_handler(k_timer *dummy)
{
  k_work_submit(&drone_work);
}

K_TIMER_DEFINE(drone_timer, RbosDrone::drone_timer_handler, NULL);

void RbosDrone::startWorkers(void)
{
	k_timer_start(&flysky_timer, K_MSEC(FLYSKY_SAMPLE_TIME_MS), K_MSEC(FLYSKY_SAMPLE_TIME_MS));
	k_timer_start(&bmp390_timer, K_MSEC(BMP390_SAMPLE_TIME_MS), K_MSEC(BMP390_SAMPLE_TIME_MS));
	k_timer_start(&drone_timer, K_MSEC(DRONE_SAMPLE_TIME_MS), K_MSEC(DRONE_SAMPLE_TIME_MS));
}

void RbosDrone::gnss_data_cb(const device *dev, const gnss_data *data)
{
	uint64_t timepulse_ns;
	k_ticks_t timepulse;

	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
			timepulse_ns = k_ticks_to_ns_near64(timepulse);
//			printf("Got a fix @ %lld ns\n", timepulse_ns);
		} else {
//			printf("Got a fix!\n");
		}
	}

	memcpy(pThis->getNavigationData(), &data->nav_data,sizeof(navigation_data));
}

#if CONFIG_GNSS_SATELLITES
void RbosDrone::gnss_satellites_cb(
		const device *dev, const gnss_satellite *satellites, uint16_t size)
{
	unsigned int tracked_count = 0;

	for (unsigned int i = 0; i != size; ++i) {
		tracked_count += satellites[i].is_tracked;
	}
/*
	printf("\n%u satellite%s reported (of which %u tracked)!\n",
		size, size > 1 ? "s" : "", tracked_count);
//*/
}
#endif

void RbosDrone::gnssInit(void)
{
/*
	gnss_systems_t supported, enabled;
	uint32_t fix_interval;
	int rc;
	GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);
#if CONFIG_GNSS_SATELLITES
	GNSS_SATELLITES_CALLBACK_DEFINE(GNSS_MODEM, gnss_satellites_cb);
#endif
	
	rc = gnss_get_supported_systems(GNSS_MODEM, &supported);
	if (rc < 0) {
		printf("Failed to query supported systems (%d)\n", rc);
		return;
	}
	rc = gnss_get_enabled_systems(GNSS_MODEM, &enabled);
	if (rc < 0) {
		printf("Failed to query enabled systems (%d)\n", rc);
		return;
	}
	printf("GNSS Systems:\n");

	rc = gnss_get_fix_rate(GNSS_MODEM, &fix_interval);
	if (rc < 0) {
		printf("Failed to query fix rate (%d)\n", rc);
		return;
	}
	printf("Fix rate = %d ms\n", fix_interval);
//*/
}

const motor_t RbosDrone::front_right = {
	PWM_DT_SPEC_GET(DT_NODELABEL(servo0)),
	DT_PROP(DT_NODELABEL(servo0), min_pulse),
	DT_PROP(DT_NODELABEL(servo0), max_pulse)
};	

const motor_t RbosDrone::back_right = {
	PWM_DT_SPEC_GET(DT_NODELABEL(servo1)),
	DT_PROP(DT_NODELABEL(servo1), min_pulse),
	DT_PROP(DT_NODELABEL(servo1), max_pulse)
};	

const motor_t RbosDrone::back_left = {
	PWM_DT_SPEC_GET(DT_NODELABEL(servo2)),
	DT_PROP(DT_NODELABEL(servo2), min_pulse),
	DT_PROP(DT_NODELABEL(servo2), max_pulse)
};	

const motor_t RbosDrone::front_left = {
	PWM_DT_SPEC_GET(DT_NODELABEL(servo3)),
	DT_PROP(DT_NODELABEL(servo3), min_pulse),
	DT_PROP(DT_NODELABEL(servo3), max_pulse)
};	

const motor_t *RbosDrone::getFrontRight(void)
{
	return &front_right;
}

const motor_t *RbosDrone::getBackRight(void)
{
	return &back_right;
}

const motor_t *RbosDrone::getBackLeft(void)
{
	return &back_left;
}

const motor_t *RbosDrone::getFrontLeft(void)
{
	return &front_left;
}

void RbosDrone::init()
{
	//imuInit();
  gyro.refNose = gyro.angleZ;

	// GPS setup
	//gnssInit();	
	
	startWorkers();

	///PID Gain Presets

  ///Make the beeping stop asap
//  gyro.currNose = 0;
}

void RbosDrone::updateMotors(void)
{
//*
	const uint64_t motor_pulse_time_ns = 1000 * flysky_data.throttle_pulse_time_us;

	pwm_set_pulse_dt(&getFrontRight()->servo, motor_pulse_time_ns);
	pwm_set_pulse_dt(&getBackRight()->servo, motor_pulse_time_ns);
	pwm_set_pulse_dt(&getBackLeft()->servo, motor_pulse_time_ns);
	pwm_set_pulse_dt(&getFrontLeft()->servo, motor_pulse_time_ns);
	
	printf(" {%llu} ", motor_pulse_time_ns);
//*/
}

void RbosDrone::doImuThings(void)
{
	mpu6050.update();

	gyro.angleX = mpu6050.getAngleX();
	gyro.angleY = mpu6050.getAngleY();
	gyro.angleZ = mpu6050.getAngleZ();

	accel.acc_roll = mpu6050.getAccAngleX();
	accel.acc_pitch = mpu6050.getAccAngleY();
	
	mpu6050.printConditionedImuData();
}

void RbosDrone::doGnssThings(void)
{
	navigation_data data = {0};
	gnssCapture(&data);
	gnssPrint(&data);
}

void RbosDrone::doFlySkyThings(void)
{
	flysky.printPulses(&flysky_data);
}

void RbosDrone::calculatePID(void)
{
	
}

void RbosDrone::doDroneThings()
{
	// update motors from last run
	// loop count 0

	// get imu data	
	doImuThings();
//doGnssThings();

	// get input values from controller
	// capture
	doFlySkyThings();

	// correct controller values
	sensor_value captured_pressure = {0};
	capturePressure(&captured_pressure);
	printf(" (%u.%u kPa) ", captured_pressure.val1, captured_pressure.val2);
	// calculate P(ID)
	calculatePID();
	// if loop time >= 4ms go into next then calculate stuff for next loop
	//
	updateMotors(); 
}

void RbosDrone::capturePressure(sensor_value *p_pressure)
{
	k_mutex_lock(&pressure_mutex, K_FOREVER);
	memcpy(p_pressure, &pressure, sizeof(sensor_value));	
	k_mutex_unlock(&pressure_mutex);
}

int RbosDrone::process_bmp390(const struct device *dev)
{
//*
	//struct sensor_value pressure;
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) 
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure);
	}
	//printf(".");
	//printf("pressure: %u.%u kPa\n", pressure.val1, pressure.val2);
//*/
	//return 0;
	return rc;
}
