#include "rbfc.h"
#include "mpu6050.h"

/*
timing_t start_time;
timing_t end_time;
uint64_t total_cycles = 0;
uint64_t total_ns = 0;
uint64_t max_ns = 0;

	//timing_init();
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

struct gpio_callback RbosDrone::throttle_data = {0};
struct gpio_callback RbosDrone::roll_data = {0};
struct gpio_callback RbosDrone::pitch_data = {0};
struct gpio_callback RbosDrone::yaw_data = {0};

uint64_t RbosDrone::throttle_time_ns = 0; 
uint64_t RbosDrone::roll_time_ns = 0;
uint64_t RbosDrone::pitch_time_ns = 0;
uint64_t RbosDrone::yaw_time_ns = 0;

RbosDrone::RbosDrone() :
	mpu6050(),
	gyro{0},
	accel{0},
	pidcontrol{0}
{
  pidcontrol.pGain[1] = 2.0;
  pidcontrol.iGain[1] = 0.02;
  pidcontrol.dGain[1] = 18.0;
  pidcontrol.max[1] = 400;
    ///pitch
  pidcontrol.pGain[2] = pidcontrol.pGain[1];
  pidcontrol.iGain[2] = pidcontrol.iGain[1];
  pidcontrol.dGain[2] = pidcontrol.dGain[1];
  pidcontrol.max[2] = pidcontrol.max[1];
    ///Yaw
  pidcontrol.pGain[3] = 1.0;
  pidcontrol.iGain[3] = 0.00;
  pidcontrol.dGain[3] = 0.0;
  pidcontrol.max[3] = 400;
	
	init();

	timing_init();
  timing_start();
}

RbosDrone::~RbosDrone() 
{

}

//*
const struct gpio_dt_spec RbosDrone::throttle = GPIO_DT_SPEC_GET_OR(THROTTLE_NODE, gpios, {0});
const struct gpio_dt_spec RbosDrone::roll 		= GPIO_DT_SPEC_GET_OR(ROLL_NODE,     gpios, {0});
const struct gpio_dt_spec RbosDrone::pitch		= GPIO_DT_SPEC_GET_OR(PITCH_NODE,    gpios, {0});
const struct gpio_dt_spec RbosDrone::yaw  		= GPIO_DT_SPEC_GET_OR(YAW_NODE,      gpios, {0});
//*/

/*
const struct gpio_dt_spec RbosDrone::receiver[4] = {
	GPIO_DT_SPEC_GET_OR(RECEIVER0_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(RECEIVER1_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(RECEIVER2_NODE, gpios, {0}),
	GPIO_DT_SPEC_GET_OR(RECEIVER3_NODE, gpios, {0})
};
//*/

void RbosDrone::printImuData(void)
{
	mpu6050.printImuData();
}

void RbosDrone::do_things()
{
	// update motors from last run
	// loop count 0 

	// get imu data	
	mpu6050.update();
	//mpu6050.printConditionedImuData();
	printReceiver();

	gyro.angleX = mpu6050.getAngleX();
	gyro.angleY = mpu6050.getAngleY();
	gyro.angleZ = mpu6050.getAngleZ();

	accel.acc_roll = mpu6050.getAccAngleX();
	accel.acc_pitch = mpu6050.getAccAngleY();

	// get input values from controller
	// correct controller values
	// calculate P(ID)
	// if loop time >= 4ms go into next then calculate stuff for next loop

}

void RbosDrone::printReceiver(void) {
	printf("(%llu, %llu, %llu, %llu) ", throttle_time_ns, roll_time_ns, pitch_time_ns, yaw_time_ns);
}

//*
void RbosDrone::throttle_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
/*
	pwm_capture_nsec(const struct device *dev, uint32_t channel,
				   pwm_flags_t flags, uint64_t *period,
				   uint64_t *pulse, k_timeout_t timeout)
//*/
	uint64_t pulse_ns = 0;
	uint64_t period_ns = 0;
 	k_timeout_t timeout_ns = K_NSEC(2000000100);

	int err = 0;
	
	pwm_t in;
	in.dev = DEVICE_DT_GET(PWM_LOOPBACK_IN_CTLR);
	in.pwm = PWM_LOOPBACK_IN_CHANNEL;
	in.flags = PWM_CAPTURE_TYPE_PULSE | PWM_POLARITY_NORMAL;
	//in.flags = PWM_CAPTURE_TYPE_BOTH;

	err = pwm_capture_nsec(in.dev, in.pwm, in.flags, &period_ns, &pulse_ns, timeout_ns);


	//pwm_capture_nsec(&in.dev, &in.pwm, 0, PWM_CAPTURE_TYPE_BOTH, &period_ns, &pulse_ns, timeout_ns);
	//pwm_capture_nsec(dev, 0, PWM_CAPTURE_TYPE_BOTH, &period_ns, &pulse_ns, timeout_ns);
	
/*
	timing_t start_time = timing_counter_get();

	while (gpio_pin_get_dt(&throttle) >= 0) 
	{
		//k_usleep(1);
	}
	
	timing_t end_time = timing_counter_get();
	
	uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);	
	uint64_t total_ns = timing_cycles_to_ns(total_cycles);

	throttle_time_ns = total_ns;
	if(total_ns > 395)
	{
		printk("t%llu ", total_ns);
	}
//*/
}
//*/

//*
void RbosDrone::roll_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	timing_t start_time = timing_counter_get();
	
	while (gpio_pin_get_dt(&roll) >= 0) 
	{
		//k_usleep(1);
	}
	
	timing_t end_time = timing_counter_get();
	
	uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);	
	uint64_t total_ns = timing_cycles_to_ns(total_cycles);
	
	roll_time_ns = total_ns;
/*
	if(total_ns > 395)
	{
		printk("r%llu ", total_ns);
	}
//*/
}
//*/

//*
void RbosDrone::pitch_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	timing_t start_time = timing_counter_get();
	
	while (gpio_pin_get_dt(&pitch) >= 0) 
	{
		//k_usleep(1);
	}
	
	timing_t end_time = timing_counter_get();
	
	uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);	
	uint64_t total_ns = timing_cycles_to_ns(total_cycles);
	pitch_time_ns = total_ns;
/*
	if(total_ns > 395)
	{
		printk("p%llu ", total_ns);
	}
//*/
}
//*/

//*
void RbosDrone::yaw_cb(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	timing_t start_time = timing_counter_get();
	
	while (gpio_pin_get_dt(&yaw) >= 0) 
	{
		//k_usleep(1);
	}
	
	timing_t end_time = timing_counter_get();
	
	uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);	
	uint64_t total_ns = timing_cycles_to_ns(total_cycles);
	yaw_time_ns = total_ns;
/*	
	if(total_ns > 395)
	{
		printk("y%llu ", total_ns);
	}
//*/
}
//*/

int RbosDrone::setupReceiver(
		const gpio_dt_spec *preceiver, 
		gpio_callback *preceiver_data, 
		gpio_callback_handler_t receiver_cb
	)
{
	int ret;
	
	while (!gpio_is_ready_dt(preceiver)) {
		printk("Error: device %s is not ready\n", preceiver->port->name);
		k_msleep(1000);
	}

	ret = gpio_pin_configure_dt(preceiver, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", 
						ret, preceiver->port->name, preceiver->pin
		);
	}

	ret = gpio_pin_interrupt_configure_dt(preceiver, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt %s pin %d\n", 
						ret, preceiver->port->name, preceiver->pin);
	}
	
	gpio_init_callback(preceiver_data, receiver_cb, BIT(preceiver->pin));
	gpio_add_callback(preceiver->port, preceiver_data);

	return 0;
}

void RbosDrone::init()
{
	mpu6050.begin();
  mpu6050.calcGyroOffsets(true);// TODO: figure out what this does
  gyro.refNose = gyro.angleZ;

	setupReceiver(&throttle, &throttle_data, throttle_cb);
	setupReceiver(&roll, &roll_data, roll_cb);
	setupReceiver(&pitch, &pitch_data, pitch_cb);
	setupReceiver(&yaw, &yaw_data, yaw_cb);
/*
	for (int i = 0; i < 4; ++i)
	{
		int ret;
		while (!gpio_is_ready_dt(&receiver[i])) {
			printk("Error: button device %s is not ready\n", receiver[i].port->name);
			k_msleep(1000);
		}

		ret = gpio_pin_configure_dt(&receiver[i], GPIO_INPUT);
		if (ret != 0) {
			printk("Error %d: failed to configure %s pin %d\n", 
							ret, receiver[i].port->name, receiver[i].pin
			);
		}

		ret = gpio_pin_interrupt_configure_dt(&receiver[i], GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			printk("Error %d: failed to configure interrupt %s pin %d\n", 
							ret, receiver[i].port->name, receiver[i].pin);
		}
		
		gpio_init_callback(&receiver_data[i], *cb[i], BIT(receiver[i].pin));
		gpio_add_callback(receiver[i].port, &receiver_data[i]);
	}
//*/

/*
	int ret;
	while (!gpio_is_ready_dt(&roll)) {
		printk("Error: button device %s is not ready\n", roll.port->name);
		k_msleep(1000);
	}

	ret = gpio_pin_configure_dt(&roll, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, roll.port->name, roll.pin);
	}

	ret = gpio_pin_interrupt_configure_dt(&roll, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt %s pin %d\n", ret, roll.port->name, roll.pin);
	}

	gpio_init_callback(&roll_cb_data, roll_cb, BIT(roll.pin));
	gpio_add_callback(roll.port, &roll_cb_data);
//*/
	///Baro Setup

  ///PID Gain Presets
    ///Roll
  //setInput();

  ///Make the beeping stop asap
//  memset(receiver_input, 1000, 5 * sizeof(int));
//  for (int i = 1; i <= 4; ++i)
//  {
//    //ESCtimer[i] = 1000;
//  }
  /*
  timerFR = 1000;
  timerBR = 1000;
  timerBL = 1000;
  timerFL = 1000;
  */
//  gyro.currNose = 0;
//  loop_timer = 0;
}


