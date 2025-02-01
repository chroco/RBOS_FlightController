#include "rbfc.h"
#include "mpu6050.h"

RbosDrone *RbosDrone::pThis = NULL;

FlySky RbosDrone::flysky = FlySky();
flysky_data_t RbosDrone::flysky_data = {0};

const device *RbosDrone::bmp390 = DEVICE_DT_GET_ONE(bosch_bmp390);
sensor_value RbosDrone::pressure = {0};

timing_t RbosDrone::start_time = {0};  
timing_t RbosDrone::stop_time = {0};  
uint64_t RbosDrone::total_us = {0};
uint64_t RbosDrone::motor_input_us[4] = {0};

RbosDrone::RbosDrone() :
	mpu6050(MPU6050()),
	gyro{0},
	accel{0},
  pid{0},
  kalman{0},
  motor_start(false)
{
	pThis = this;

	k_mutex_init(&pressure_mutex);
	k_mutex_init(&esc_mutex);

	init();

  timing_init();
  timing_start();
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
  mpu6050.calcGyroOffsets();
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

void RbosDrone::printExecutionTimeInUs(void)
{
  printk("(%llu us)", total_us);
}

void RbosDrone::drone_work_handler(k_work *work)
{
  start_time = timing_counter_get();
	pThis->doDroneThings();
  stop_time = timing_counter_get();
  uint64_t total_cycles = timing_cycles_get(&start_time, &stop_time);
  uint64_t total_ns = timing_cycles_to_ns(total_cycles);
  total_us = total_ns / 1000;
}

K_WORK_DEFINE(drone_work, RbosDrone::drone_work_handler);

void RbosDrone::drone_timer_handler(k_timer *dummy)
{
  k_work_submit(&drone_work);
}

K_TIMER_DEFINE(drone_timer, RbosDrone::drone_timer_handler, NULL);

//*
void RbosDrone::print_work_handler(k_work *work)
{
 
//*
	k_mutex_lock(&pThis->esc_mutex, K_FOREVER);
  uint64_t temp_motor_input_us[4] = {
    motor_input_us[0],
    motor_input_us[1],
    motor_input_us[2],
    motor_input_us[3]
  };
	k_mutex_unlock(&pThis->esc_mutex);
//*/

//*
  pThis->flysky.printPulses(&flysky_data);
  printk("->(%llu, %llu, %llu, %llu)", temp_motor_input_us[0],
                                     temp_motor_input_us[1],
                                     temp_motor_input_us[2],
                                     temp_motor_input_us[3]);
//*/

//  sensor_value captured_pressure = {0};
//  pThis->capturePressure(&captured_pressure);
//	printk("(%u.%u kPa)", captured_pressure.val1, captured_pressure.val2);
  
  //pThis->mpu6050.printConditionedImuData();
  pThis->printExecutionTimeInUs();
}

K_WORK_DEFINE(print_work, RbosDrone::print_work_handler);

void RbosDrone::print_timer_handler(k_timer *dummy)
{
  k_work_submit(&print_work);
}

K_TIMER_DEFINE(print_timer, RbosDrone::print_timer_handler, NULL);
//*/

void RbosDrone::startWorkers(void)
{
	k_timer_start(&flysky_timer, K_MSEC(FLYSKY_SAMPLE_TIME_MS), K_MSEC(FLYSKY_SAMPLE_TIME_MS));
	k_timer_start(&bmp390_timer, K_MSEC(BMP390_SAMPLE_TIME_MS), K_MSEC(BMP390_SAMPLE_TIME_MS));
	k_timer_start(&drone_timer, K_MSEC(DRONE_SAMPLE_TIME_MS), K_MSEC(DRONE_SAMPLE_TIME_MS));
	k_timer_start(&print_timer, K_MSEC(DRONE_PRINT_TIME_MS), K_MSEC(DRONE_PRINT_TIME_MS));
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
  ///Init IMU and Calibration
	imuInit();
	
  ///Init PID 
  setPID(pid, 'S');

  ///Volt Meter

  startWorkers();
}

void RbosDrone::updateMotors(void)
{
  if (motor_start == false && 
      flysky_data.throttle_pulse_time_us < 1050 && 
      flysky_data.roll_pulse_time_us     < 1050 && 
      flysky_data.pitch_pulse_time_us    < 1050 && 
      flysky_data.yaw_pulse_time_us      > 1950 
     )
  {
    mpu6050.calcGyroOffsets(); 
    gyro.rateCalibration[roll] -= mpu6050.getGyroYoffset();
    gyro.rateCalibration[pitch] -= mpu6050.getGyroXoffset();
    gyro.rateCalibration[yaw] -= mpu6050.getGyroZoffset();

    motor_start = true;
  }
  if (motor_start == false && 
      flysky_data.throttle_pulse_time_us < 1050 && 
      flysky_data.roll_pulse_time_us     > 1950 && 
      flysky_data.pitch_pulse_time_us    < 1050 && 
      flysky_data.yaw_pulse_time_us      > 1950)
  {
    motor_start = true;
  }
  if (motor_start == true && 
      flysky_data.throttle_pulse_time_us < 1050 && 
      flysky_data.roll_pulse_time_us     > 1950 && 
      flysky_data.pitch_pulse_time_us    < 1050 && 
      flysky_data.yaw_pulse_time_us      < 1050)
  {
    motor_start = false; 
  }

  // End of loop operations
  // getVoltageCompensation(myMeter);

  // Assume the motors are off
//*
  // 1e6 ns == 1000 us
	pwm_set_pulse_dt(&getFrontRight()->servo, 1000000);
	pwm_set_pulse_dt(&getBackRight()->servo,  1000000);
	pwm_set_pulse_dt(&getBackLeft()->servo,   1000000);
	pwm_set_pulse_dt(&getFrontLeft()->servo,  1000000);
  
  uint64_t ESCtimer[4] = {0};
  
  ///In the event that the motors are on
  if (motor_start == true)
  {
    if (flysky_data.throttle_pulse_time_us < 1050)
    {
      flysky_data.throttle_pulse_time_us = 1000;
      setPID(pid, 'R');
    }
    else
    {
      if (flysky_data.throttle_pulse_time_us > MAX_THROTTLE)
      {
        flysky_data.throttle_pulse_time_us = MAX_THROTTLE;
      }

      //  Motor blending equations   
      
      //  (front-right - CCW)     
      ESCtimer[0] = flysky_data.throttle_pulse_time_us 
                    + pid.finalValue[roll] 
                    + pid.finalValue[pitch] 
                    - pid.finalValue[yaw];
      
      //  (back-right  - CW)
      ESCtimer[1] = flysky_data.throttle_pulse_time_us 
                    + pid.finalValue[roll] 
                    - pid.finalValue[pitch] 
                    + pid.finalValue[yaw];

      //  (back-left   - CCW)
      ESCtimer[2] = flysky_data.throttle_pulse_time_us 
                    - pid.finalValue[roll] 
                    - pid.finalValue[pitch] 
                    - pid.finalValue[yaw];

      //  (front-left  - CW)
      ESCtimer[3] = flysky_data.throttle_pulse_time_us 
                    - pid.finalValue[roll] 
                    + pid.finalValue[pitch] 
                    + pid.finalValue[yaw];
      
      ///Keep the motors spinning
      if (ESCtimer[0] < MIN_ESC_OUT){ESCtimer[0] = MIN_ESC_OUT;}
      if (ESCtimer[1] < MIN_ESC_OUT){ESCtimer[1] = MIN_ESC_OUT;}
      if (ESCtimer[2] < MIN_ESC_OUT){ESCtimer[2] = MIN_ESC_OUT;}
      if (ESCtimer[3] < MIN_ESC_OUT){ESCtimer[3] = MIN_ESC_OUT;}
      
      ///Limit output to 2000us
      if (ESCtimer[0] > MAX_ESC_OUT){ESCtimer[0] = MAX_ESC_OUT;}
      if (ESCtimer[1] > MAX_ESC_OUT){ESCtimer[1] = MAX_ESC_OUT;}
      if (ESCtimer[2] > MAX_ESC_OUT){ESCtimer[2] = MAX_ESC_OUT;}
      if (ESCtimer[3] > MAX_ESC_OUT){ESCtimer[3] = MAX_ESC_OUT;}
    }
  }
//*
  motor_input_us[0] = ESCtimer[0];
  motor_input_us[1] = ESCtimer[1];
  motor_input_us[2] = ESCtimer[2];
  motor_input_us[3] = ESCtimer[3];
//*/
  
  pwm_set_pulse_dt(&getFrontRight()->servo, ESCtimer[0] * 1000);
  pwm_set_pulse_dt(&getBackRight()->servo,  ESCtimer[1] * 1000);
  pwm_set_pulse_dt(&getBackLeft()->servo,   ESCtimer[2] * 1000);
  pwm_set_pulse_dt(&getFrontLeft()->servo,  ESCtimer[3] * 1000);
}

void RbosDrone::updateImu(void)
{
	mpu6050.update();

  gyro.rotationRate[0] = mpu6050.getGyroX();
  gyro.rotationRate[1] = mpu6050.getGyroY();
  gyro.rotationRate[2] = mpu6050.getGyroZ();

  accel.acc[0] = mpu6050.getAccX();
  accel.acc[1] = mpu6050.getAccY();
  accel.acc[2] = mpu6050.getAccZ();

  accel.ang[0] = mpu6050.getAccAngleX();
  accel.ang[1] = mpu6050.getAccAngleY();
}

void RbosDrone::updateFlySky(void)
{
  flysky.capturePulses(&flysky_data);
}

void RbosDrone::updateKalman(void)
{
  for (int i = 0; i < 2; ++i)
  {
    kalman_1d(kalman, kalman.kalmanAngle[i], kalman.kalmanUncertaintyAngle[i], 
        gyro.rotationRate[i], accel.ang[i]);
    kalman.kalmanAngle[i] = kalman.kalmanOutput[0];
    kalman.kalmanUncertaintyAngle[i] = kalman.kalmanOutput[1];
  }
}

void RbosDrone::calculatePID(void)
{
	// angle setpoints
  pid.desiredAngle[roll] = -flysky.getRoll()->getCorrectedInput();
  pid.desiredAngle[pitch] = -flysky.getPitch()->getCorrectedInput();
  pid.desiredRate[yaw] = -flysky.getYaw()->getCorrectedInput();

  calculate_pid(pid, kalman, gyro, accel);
}

void RbosDrone::updatePressure(void)
{
	//capturePressure(&captured_pressure);
}

void RbosDrone::doDroneThings()
{
	// get imu data	
	updateImu();

  // Calculate XY Kalman Angles
  updateKalman();

	// capture and correct pulse times from receivers
	updateFlySky();

  ///PID Calculations
  calculatePID();

  // update pressure
  //updatePressure();	
  
  // update motors
	updateMotors(); 
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
