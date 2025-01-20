#include "rbfc.h"
#include "mpu6050.h"

RbosDrone *RbosDrone::pThis = NULL;
static RbosDrone *fmlThis = NULL;
struct gnss_data fml_data;

RbosDrone::RbosDrone() :
	nav_data{0},	
	satellites{0},
	mpu6050(MPU6050()),
	gyro{0},
	accel{0},
	pidcontrol{0}
{
	pThis = this;
	fmlThis = this;

	k_mutex_init(&gps_mutex);

//*
  pidcontrol.pGain[1] = 2.0;
  pidcontrol.iGain[1] = 0.02;
  pidcontrol.dGain[1] = 18.0;
  pidcontrol.max[1]   = 400;
    ///pitch
  pidcontrol.pGain[2] = pidcontrol.pGain[1];
  pidcontrol.iGain[2] = pidcontrol.iGain[1];
  pidcontrol.dGain[2] = pidcontrol.dGain[1];
  pidcontrol.max[2]   = pidcontrol.max[1];
    ///Yaw
  pidcontrol.pGain[3] = 1.0;
  pidcontrol.iGain[3] = 0.00;
  pidcontrol.dGain[3] = 0.0;
  pidcontrol.max[3]   = 400;
//*/

	init();
}

RbosDrone::~RbosDrone() 
{

}

void RbosDrone::printImuData(void)
{
	mpu6050.printImuData();
}

FlySky RbosDrone::flysky = FlySky();
flysky_data_t RbosDrone::flysky_data = {0};

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

void RbosDrone::drone_work_handler(k_work *work)
{
	pThis->do_things();
}

K_WORK_DEFINE(drone_work, RbosDrone::drone_work_handler);

void RbosDrone::drone_timer_handler(k_timer *dummy)
{
  k_work_submit(&drone_work);
}

K_TIMER_DEFINE(drone_timer, RbosDrone::drone_timer_handler, NULL);

void RbosDrone::imuInit(void)
{
	mpu6050.begin();
  mpu6050.calcGyroOffsets(true);// TODO: figure out what this does
}

void RbosDrone::timerInit(void)
{
	k_timer_start(&flysky_timer, K_MSEC(FLYSKY_SAMPLE_TIME_MS), K_MSEC(FLYSKY_SAMPLE_TIME_MS));
	k_timer_start(&drone_timer, K_MSEC(DRONE_SAMPLE_TIME_MS), K_MSEC(DRONE_SAMPLE_TIME_MS));
}

static void gnss_data_cb(const struct device *dev, const struct gnss_data *data)
{
	uint64_t timepulse_ns;
	k_ticks_t timepulse;

	if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
		if (gnss_get_latest_timepulse(dev, &timepulse) == 0) {
			timepulse_ns = k_ticks_to_ns_near64(timepulse);
			printf("Got a fix @ %lld ns\n", timepulse_ns);
		} else {
			printf("Got a fix!\n");
		}
	}
//	printf("*\n*\n*\n");
/*
	printf("\n<%llu %llu %u %u %u>\n",
		data->nav_data.latitude,
		data->nav_data.longitude,
		data->nav_data.bearing,
		data->nav_data.speed,
		data->nav_data.altitude
	);
	fmlThis->m_gnss_data.nav_data.latitude = data->nav_data.latitude;
	fmlThis->m_gnss_data.nav_data.longitude = data->nav_data.longitude;
	fmlThis->m_gnss_data.nav_data.bearing = data->nav_data.bearing;
	fmlThis->m_gnss_data.nav_data.speed = data->nav_data.speed;
	fmlThis->m_gnss_data.nav_data.altitude = data->nav_data.altitude; 
*/
	memcpy(&fmlThis->nav_data, &data->nav_data,sizeof(navigation_data));
}
GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gnss_data_cb);

#if CONFIG_GNSS_SATELLITES
static void gnss_satellites_cb(const struct device *dev, const struct gnss_satellite *satellites,
			       uint16_t size)
{
	unsigned int tracked_count = 0;

	for (unsigned int i = 0; i != size; ++i) {
		tracked_count += satellites[i].is_tracked;
	}
	printf("%u satellite%s reported (of which %u tracked)!\n",
		size, size > 1 ? "s" : "", tracked_count);
}
#endif
GNSS_SATELLITES_CALLBACK_DEFINE(GNSS_MODEM, gnss_satellites_cb);

void RbosDrone::gnssInit(void)
{
	gnss_systems_t supported, enabled;
	uint32_t fix_interval;
	int rc;

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

}

void RbosDrone::init()
{
	imuInit();
  gyro.refNose = gyro.angleZ;

	// GPS setup
	gnssInit();	
	
	timerInit();
	
	///Baro Setup

  ///PID Gain Presets

  ///Make the beeping stop asap
//  gyro.currNose = 0;
}

void RbosDrone::do_things()
{
	// update motors from last run
	// loop count 0 
//*
//	k_mutex_lock(&gps_mutex, K_FOREVER);
	printf(" <%llu %llu %u %u %u> ",
		nav_data.latitude,
		nav_data.longitude,
		nav_data.bearing,
		nav_data.speed,
		nav_data.altitude
	);
//	k_mutex_unlock(&gps_mutex);
//*/

	// get imu data	
	mpu6050.update();
	mpu6050.printConditionedImuData();

	gyro.angleX = mpu6050.getAngleX();
	gyro.angleY = mpu6050.getAngleY();
	gyro.angleZ = mpu6050.getAngleZ();

	accel.acc_roll = mpu6050.getAccAngleX();
	accel.acc_pitch = mpu6050.getAccAngleY();
	// get input values from controller
	flysky.printPulses(&flysky_data);

	// correct controller values
	
	// calculate P(ID)
	// if loop time >= 4ms go into next then calculate stuff for next loop

}



