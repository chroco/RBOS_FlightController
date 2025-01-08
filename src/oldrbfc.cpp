#include "RBFC.h"
#include "MPU6050_tockn_RB.h"

RbosDrone::RbosDrone() :
	mpu6050()
{

}

RbosDrone::~RbosDrone() 
{

}

void RbosDrone::printImuData(void)
{
	mpu6050.printImuData();
}

///Data Objects
accel_t accel;
gyro_t gyro;
PID_t pid_calc;

///MODE
byte moving = 0;
byte motor_start = 0;
int motor_start_counter = 0;
byte autoLevel = 0;

///PID
//float pid_error_temp;

///IMU
//MPU6050 mpu6050(Wire);
byte IMU_read = 0;
double gyro_input[4];
double level_adj[4];

///Baro

///Input Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
/*
byte last_channel[5];
unsigned long input_timer[5];
///These to be changed to: last_channel[] & input_timer[]
*/

int receiver_input[5];
int corrected_input[5];
///These need to be changed to: receiver_input = {1 = roll, 2 = pitch, 3 = yaw, 4 = throttle}

///Motor Timers & Motors
/*
int ESCtimer[5];
Servo motorFR;
Servo motorBR;
Servo motorBL;
Servo motorFL;
//*/

///Main Loop Timers
unsigned long loop_timer, curr_time, prev_time, loop_start;

///Function Prototypes
void getIMU(MPU6050 IMU, gyro_t gyro, accel_t accel);
void setInput();
void ReportThread(void* poop);
//void IRAM_ATTR ISR_INPUT();

///Sensor Thread & Timers
//TaskHandle_t ReportTask;
unsigned long sense_timer, sense_start, sense_curr, sense_prev;

void setup()
{
 // mpu6050.begin();
 // mpu6050.calcGyroOffsets(true); TODO: figure out what this does
  gyro.refNose = gyro.currGyro[3];

  ///Baro Setup

  ///PID Gain Presets
    ///Roll
  pid_calc.pGain[1] = 2.0;
  pid_calc.iGain[1] = 0.02;
  pid_calc.dGain[1] = 18.0;
  pid_calc.max[1] = 400;
    ///pitch
  pid_calc.pGain[2] = pid_calc.pGain[1];
  pid_calc.iGain[2] = pid_calc.iGain[1];
  pid_calc.dGain[2] = pid_calc.dGain[1];
  pid_calc.max[2] = pid_calc.max[1];
    ///Yaw
  pid_calc.pGain[3] = 1.0;
  pid_calc.iGain[3] = 0.00;
  pid_calc.dGain[3] = 0.0;
  pid_calc.max[3] = 400;
  ///PID Init. other
  memset(pid_calc.pMem, 0, 4 * sizeof(double));
  memset(pid_calc.pError, 0, 4 * sizeof(double));
  memset(pid_calc.iMem, 0, 4 * sizeof(double));
  memset(pid_calc.iError, 0, 4 * sizeof(double));
  memset(pid_calc.dMem, 0, 4 * sizeof(double));
  memset(pid_calc.dError, 0, 4 * sizeof(double));
  memset(pid_calc.setpoint, 0, 4 * sizeof(double));


  ///Thread #2
  /*
  xTaskCreatePinnedToCore(
      ReportThread, 
      "ReportTask", 
      50000, 
      NULL,
      1,
      &ReportTask,
      1);
  */

  //setInput();

  ///Motor Pins and Servo Attach
	/*
  pinMode(motorFRPin, OUTPUT);
  pinMode(motorBRPin, OUTPUT);
  pinMode(motorBLPin, OUTPUT);
  pinMode(motorFLPin, OUTPUT);
  motorFR.attach(motorFRPin);
  motorBR.attach(motorBRPin);
  motorBL.attach(motorBLPin);
  motorFL.attach(motorFLPin);
	//*/

  ///Make the beeping stop asap
  memset(receiver_input, 1000, 5 * sizeof(int));
  for (int i = 1; i <= 4; ++i)
  {
    //ESCtimer[i] = 1000;
  }
  /*
  timerFR = 1000;
  timerBR = 1000;
  timerBL = 1000;
  timerFL = 1000;
  */
  gyro.currNose = 0;
  loop_timer = 0;
}


void RbosDrone::do_things()
{
	mpu6050.update();
	mpu6050.printConditionedImuData();
}

void RbosDrone::dont_do_things()
{
  //prev_time = curr_time;
  //curr_time = micros();
	
  ///Loop Start Operations
  if (loop_timer == 0)
  {
    //loop_start = micros();

    ///Set motor pulse from last loop
/*
    motorFR.writeMicroseconds(ESCtimer[1]);
    motorBR.writeMicroseconds(ESCtimer[2]);
    motorBL.writeMicroseconds(ESCtimer[3]);
    motorFL.writeMicroseconds(ESCtimer[4]);
//*/

    ///If the IMU has not been read this loop, then read IMU
    if (IMU_read == 0)
    {
      ///This might have a use, so it is staying for now
      for (int i = 1; i <= 3; ++i)
      {
        gyro.prevGyro[i] = gyro.currGyro[i];
      }
      mpu6050.update();
			mpu6050.printConditionedImuData();
      
			gyro.currGyro[1] = mpu6050.getAngleX();
      gyro.currGyro[2] = mpu6050.getAngleY();
      gyro.currGyro[3] = mpu6050.getAngleZ();

      accel.acc_roll = mpu6050.getAccAngleX();
      accel.acc_pitch = mpu6050.getAccAngleY();
      //IMU_read = 1;
    }
    ///Set gyro_input
    for (int i = 1; i <= 3; ++i)
    {
      gyro_input[i] = gyro.currGyro[i];
    }
  }
  //loop_timer += (curr_time - prev_time);

  //printReport(pid_calc, ESCtimer, gyro, corrected_input);

/*
  correctInput(receiver_input, corrected_input);

  if (motor_start == 0 && receiver_input[4] < 1050 && receiver_input[1] < 1050 && 
			receiver_input[2] < 1050 && receiver_input[3] > 1950)
  {
    mpu6050.calcGyroOffsets(true);

    memset(pid_calc.pMem, 0, 4 * sizeof(double));
    memset(pid_calc.pError, 0, 4 * sizeof(double));
    memset(pid_calc.iMem, 0, 4 * sizeof(double));
    memset(pid_calc.iError, 0, 4 * sizeof(double));
    memset(pid_calc.dMem, 0, 4 * sizeof(double));
    memset(pid_calc.dError, 0, 4 * sizeof(double));
    memset(pid_calc.setpoint, 0, 4 * sizeof(double));

    gyro.refNose = gyro.currGyro[3];

    motor_start = 1;
  }
  if (motor_start == 0 && receiver_input[4] < 1050 && receiver_input[1] > 1950 && 
			receiver_input[2] < 1050 && receiver_input[3] > 1950)
  {
    gyro.refNose = gyro.currGyro[3];

    motor_start = 1;
  }
  if (motor_start == 1 && receiver_input[4] < 1050 && receiver_input[1] > 1950 && 
			receiver_input[2] < 1050 && receiver_input[3] < 1050)
  {
    motor_start = 0;
  }
//*/

  ///Setpoints
/*
  for (int i = 1; i <= 3; ++i)
  {
    switch(i)
    {
      case 1:
      case 2:
        pid_calc.setpoint[i] = 0;
        pid_calc.setpoint[i] = corrected_input[i];
        break;
      case 3:
        pid_calc.setpoint[i] = 0;
        gyro.refNose += corrected_input[3] / 65.5;
        pid_calc.setpoint[i] = gyro.currNose - gyro.refNose;
        break;
      default:
        break;
    }
  }
//*/

  ///PID Calculations for next loop cycle
//*
  for (int i = 1; i <= 2; ++i)///1 = roll, 2 = pitch, 3 = yaw
  {
    pid_calc.pid_error_temp = gyro_input[i] - pid_calc.setpoint[i];
    pid_calc.iMem[i] = pid_calc.iGain[i] * pid_calc.pid_error_temp;

    if(pid_calc.iMem[i] > pid_calc.max[i])
		{
			pid_calc.iMem[i] = pid_calc.max[i];
		} 
		else if(pid_calc.iMem[i] < pid_calc.max[i] * -1)
		{
			pid_calc.iMem[i] = pid_calc.max[i] * -1;
		}
    
		pid_calc.out[i] = pid_calc.pGain[i] * pid_calc.pid_error_temp + pid_calc.iMem[i] + 
											pid_calc.dGain[i] * (pid_calc.pid_error_temp - pid_calc.dError[i]);
    
		if(pid_calc.out[i] > pid_calc.max[i])
		{
			pid_calc.out[i] = pid_calc.max[i];
		}
    else if(pid_calc.out[i] < pid_calc.max[i] * -1)
		{
			pid_calc.out[i] = pid_calc.max[i] * -1;
		}

    pid_calc.dError[i] = pid_calc.pid_error_temp;
  }
//*/
  ///End of loop operations
/*
  if (micros() - loop_start >= 4000)
  {
    ///Assume the motors are off
    ESCtimer[1] = 1000;
    ESCtimer[2] = 1000;
    ESCtimer[3] = 1000;
    ESCtimer[4] = 1000;

    ///In the event that the motors are on
    if (motor_start == 1)
    {
      if (receiver_input[4] < 1050)
      {
        receiver_input[4] = 1000;
      }
      else
      {
        if (receiver_input[4] > 1800)
        {
          receiver_input[4] = 1800;
        }
        //
        //pid_calc.out[2] - pid_calc.out[1]
        //pid_calc.out[2] - pid_calc.out[1]
        //pid_calc.out[2] + pid_calc.out[1]
        //pid_calc.out[2] + pid_calc.out[1]
        //
				//Calculate the pulse for esc 1 (front-right - CCW)
        ESCtimer[1] = receiver_input[4] - pid_calc.out[2] - pid_calc.out[1]; 
        //Calculate the pulse for esc 2 (rear-right - CW)
        ESCtimer[2] = receiver_input[4] + pid_calc.out[2] - pid_calc.out[1]; 
				//Calculate the pulse for esc 3 (rear-left - CCW)
        ESCtimer[3] = receiver_input[4] + pid_calc.out[2] + pid_calc.out[1]; 
				//Calculate the pulse for esc 4 (front-left - CW)
				ESCtimer[4] = receiver_input[4] - pid_calc.out[2] + pid_calc.out[1]; 
        
				if (ESCtimer[1] < 1100){ESCtimer[1] = 1100;}
        if (ESCtimer[2] < 1100){ESCtimer[2] = 1100;}
        if (ESCtimer[3] < 1100){ESCtimer[3] = 1100;}
        if (ESCtimer[4] < 1100){ESCtimer[4] = 1100;}
      }
    }
    
    prev_time = micros() - loop_start;
    loop_timer = 0;
    IMU_read = 0;
  }
//*/
}

void getIMU(MPU6050 mpu6050, gyro_t gyro, accel_t accel)
{
  mpu6050.update();

  gyro.currGyro[1] = mpu6050.getAngleY();
  gyro.currGyro[2] = mpu6050.getAngleX();
  gyro.currGyro[3] = mpu6050.getAngleZ();
}

void setInput()
{
/*
  pinMode(chan1Pin, INPUT);
  pinMode(chan2Pin, INPUT);
  pinMode(chan3Pin, INPUT);
  pinMode(chan4Pin, INPUT);
  attachInterrupt(chan1Pin, ISR_INPUT, CHANGE);
  attachInterrupt(chan2Pin, ISR_INPUT, CHANGE);
  attachInterrupt(chan3Pin, ISR_INPUT, CHANGE);
  attachInterrupt(chan4Pin, ISR_INPUT, CHANGE);
//*/
}

