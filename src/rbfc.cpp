#include "rbfc.h"
#include "mpu6050.h"

RbosDrone::RbosDrone() :
	flysky(FlySky()),	
	mpu6050(),
	gyro{0},
	accel{0},
	pidcontrol{0}
{
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
	
	init();
}

RbosDrone::~RbosDrone() 
{

}

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
	mpu6050.printConditionedImuData();

	gyro.angleX = mpu6050.getAngleX();
	gyro.angleY = mpu6050.getAngleY();
	gyro.angleZ = mpu6050.getAngleZ();

	accel.acc_roll = mpu6050.getAccAngleX();
	accel.acc_pitch = mpu6050.getAccAngleY();

	// get input values from controller
//*
	flyskydata_t flyskydata = {0};
	flysky.getPulseTimes(&flyskydata);
	flysky.printPulses(&flyskydata);
//*/
//
	k_msleep(1);
	//flysky.printPulses();
	
	// correct controller values
	
	// calculate P(ID)
	// if loop time >= 4ms go into next then calculate stuff for next loop

}

void RbosDrone::init()
{
	mpu6050.begin();
  mpu6050.calcGyroOffsets(true);// TODO: figure out what this does
  gyro.refNose = gyro.angleZ;

	///Baro Setup

  ///PID Gain Presets

  ///Make the beeping stop asap
//  gyro.currNose = 0;
}


