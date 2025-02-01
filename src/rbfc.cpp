#include "rbfc.h"
#include "mpu6050.h"

RbosDrone *RbosDrone::pThis = NULL;

FlySky RbosDrone::flysky = FlySky();
flysky_data_t RbosDrone::flysky_data = {0};

const device *RbosDrone::bmp390 = DEVICE_DT_GET_ONE(bosch_bmp390);
sensor_value RbosDrone::pressure = {0};;

RbosDrone::RbosDrone() :
	mpu6050(MPU6050()),
	gyro{0},
	accel{0},
	pidcontrol{0}
{
	pThis = this;

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

	init();
}

RbosDrone::~RbosDrone() 
{

}

void RbosDrone::printImuData(void)
{
	mpu6050.printImuData();
}

void RbosDrone::imuInit(void)
{
	mpu6050.begin();
  mpu6050.calcGyroOffsets(true);// TODO: figure out what this does
}

//*
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
//*/

//*
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
//*/

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
	imuInit();
 // gyro.refNose = gyro.angleZ;

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

	// get input values from controller
	// capture
	doFlySkyThings();

	mpu6050.printConditionedImuData();
	// correct controller values
	sensor_value captured_pressure = {0};
	capturePressure(&captured_pressure);
	printk(" (%u.%u kPa) ", captured_pressure.val1, captured_pressure.val2);
	// calculate P(ID)
//	calculatePID();
	// if loop time >= 4ms go into next then calculate stuff for next loop
	//
//	updateMotors(); 
}

//*
void RbosDrone::capturePressure(sensor_value *p_pressure)
{
	k_mutex_lock(&pressure_mutex, K_FOREVER);
	memcpy(p_pressure, &pressure, sizeof(sensor_value));	
	k_mutex_unlock(&pressure_mutex);
}
//*/

int RbosDrone::process_bmp390(const struct device *dev)
{
	//struct sensor_value pressure;
	int rc = sensor_sample_fetch(dev);

	if (rc == 0) 
	{
		rc = sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure);
	}

	return rc;
}
