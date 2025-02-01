#include "RBPID.h"

///Data Objects
accel_t myAccel;
gyro_t myGyro;
PID_t myPID;
kalman_t myKalman;
voltMeter_t myMeter;

///MODE
byte motor_start = 0;

///IMU
MPU6050 mpu6050(Wire);
byte IMU_read = 0;

///Baro

///Input Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;

///{0 = throttle, 1 = roll, 2 = pitch, 3 = yaw}
int receiver_input[4];
int corrected_input[4];

///Motor Timers & Motors
int ESCtimer[4];
Servo motorFR;
Servo motorBR;
Servo motorBL;
Servo motorFL;

///Main Loop Timers
unsigned long loop_timer, curr_time, prev_time, loop_start, loop_final;

///Function Prototypes
void setInput();
void IRAM_ATTR ISR_INPUT();

void setup()
{
  Serial.begin(1000000);
  while (!Serial) delay(10);
  Wire.begin();
  mpu6050.begin();

  ///Init IMU and Calibration
  mpu6050.calcGyroOffsets(true);
  myGyro.rateCalibration[0] -= mpu6050.getGyroYoffset();
  myGyro.rateCalibration[1] -= mpu6050.getGyroXoffset();
  myGyro.rateCalibration[2] -= mpu6050.getGyroZoffset();
  memset(myKalman.kalmanAngle, 0, 2);
  memset(myKalman.kalmanUncertaintyAngle, 4, 2);
  memset(myKalman.kalmanOutput, 0, 2);

  ///Init PID and Input
  setPID(myPID, 'S');
  setInput();

  ///Volt Meter
  pinMode(voltMeterPin, INPUT);

  ///Motor Pins and Servo Attach
  pinMode(motorFRPin, OUTPUT);
  pinMode(motorBRPin, OUTPUT);
  pinMode(motorBLPin, OUTPUT);
  pinMode(motorFLPin, OUTPUT);
  motorFR.attach(motorFRPin);
  motorBR.attach(motorBRPin);
  motorBL.attach(motorBLPin);
  motorFL.attach(motorFLPin);

  ///Make the beeping stop asap
  memset(receiver_input, 1000, 5);
  for (int i = 0; i < 4; ++i)
  {
    ESCtimer[i] = 1000;
  }
  loop_timer = 0;
}

void loop()
{
  prev_time = curr_time;
  curr_time = micros();
  
  ///Loop Start Operations
  if (loop_timer == 0)
  {
    loop_start = micros();

    ///Set motor pulse from last loop
    motorFR.writeMicroseconds(ESCtimer[0]);
    motorBR.writeMicroseconds(ESCtimer[1]);
    motorBL.writeMicroseconds(ESCtimer[2]);
    motorFL.writeMicroseconds(ESCtimer[3]);

    printReport(myMeter, myPID, myKalman, ESCtimer, myAccel, myGyro, corrected_input, loop_final);

    ///Capture IMU data and correct gyro
    getIMU(mpu6050, myGyro, myAccel);
    myGyro.rotationRate[0] -= myGyro.rateCalibration[0];
    myGyro.rotationRate[1] -= myGyro.rateCalibration[1];
    myGyro.rotationRate[2] -= myGyro.rateCalibration[2];

    ///Calculate XY Kalman Angles
    for (int i = 0; i < 2; ++i)
    {
      kalman_1d(myKalman, myKalman.kalmanAngle[i], myKalman.kalmanUncertaintyAngle[i], myGyro.rotationRate[i], myAccel.ang[i]);
      myKalman.kalmanAngle[i] = myKalman.kalmanOutput[0];
      myKalman.kalmanUncertaintyAngle[i] = myKalman.kalmanOutput[1];
    }
  }
  loop_timer += (curr_time - prev_time);

  ///Capture and Correct Receiver
  correctInput(receiver_input, corrected_input);

  ///PID angle setpoints
  myPID.desiredAngle[0] = corrected_input[1];
  myPID.desiredAngle[1] = corrected_input[2];
  myPID.desiredRate[2] = corrected_input[3];

  ///PID Calculations for next loop cycle
  calculate_pid(myPID, myKalman, myGyro, myAccel);

  ///Motor Start and Stop
  if (motor_start == 0 && receiver_input[0] < 1050 && receiver_input[1] < 1050 && receiver_input[2] < 1050 && receiver_input[3] > 1950)
  {
    mpu6050.calcGyroOffsets(true);
    myGyro.rateCalibration[0] -= mpu6050.getGyroYoffset();
    myGyro.rateCalibration[1] -= mpu6050.getGyroXoffset();
    myGyro.rateCalibration[2] -= mpu6050.getGyroZoffset();

    motor_start = 1;
  }
  if (motor_start == 0 && receiver_input[0] < 1050 && receiver_input[1] > 1950 && receiver_input[2] < 1050 && receiver_input[3] > 1950)
  {
    motor_start = 1;
  }
  if (motor_start == 1 && receiver_input[0] < 1050 && receiver_input[1] > 1950 && receiver_input[2] < 1050 && receiver_input[3] < 1050)
  {
    motor_start = 0;
  }

  ///End of loop operations
  getVoltageCompensation(myMeter);

  ///Assume the motors are off
  ESCtimer[0] = 1000;
  ESCtimer[1] = 1000;
  ESCtimer[2] = 1000;
  ESCtimer[3] = 1000;

  ///In the event that the motors are on
  if (motor_start == 1)
  {
    if (receiver_input[0] < 1050)
    {
      receiver_input[0] = 1000;
      setPID(myPID, 'R');
    }
    else
    {
      if (receiver_input[0] > MAX_THROTTLE)
      {
        receiver_input[0] = MAX_THROTTLE;
      }

    /*(front-right - CCW)
      (back-right  - CW)
      (back-left   - CCW)
      (front-left  - CW)
      Motor blending equations   */
      ESCtimer[0] = receiver_input[0] + myPID.final[0] + myPID.final[1] - myPID.final[2];
      ESCtimer[1] = receiver_input[0] + myPID.final[0] - myPID.final[1] + myPID.final[2];
      ESCtimer[2] = receiver_input[0] - myPID.final[0] - myPID.final[1] - myPID.final[2];
      ESCtimer[3] = receiver_input[0] - myPID.final[0] + myPID.final[1] + myPID.final[2];

      ///Keep the motors spinning
      if (ESCtimer[0] < 1100){ESCtimer[0] = 1100;}
      if (ESCtimer[1] < 1100){ESCtimer[1] = 1100;}
      if (ESCtimer[2] < 1100){ESCtimer[2] = 1100;}
      if (ESCtimer[3] < 1100){ESCtimer[3] = 1100;}
      
      ///Limit output to 2000us
      if (ESCtimer[0] > 2000){ESCtimer[0] = 2000;}
      if (ESCtimer[1] > 2000){ESCtimer[1] = 2000;}
      if (ESCtimer[2] > 2000){ESCtimer[2] = 2000;}
      if (ESCtimer[3] > 2000){ESCtimer[3] = 2000;}
    }
  }
  
  ///wait for 4ms to start the next loop
  while((micros() - loop_start) < 4000);
  
  loop_final = micros() - loop_start;
  loop_timer = 0;
}

void setInput()
{
  pinMode(chan1Pin, INPUT);
  pinMode(chan2Pin, INPUT);
  pinMode(chan3Pin, INPUT);
  pinMode(chan4Pin, INPUT);
  attachInterrupt(chan1Pin, ISR_INPUT, CHANGE);
  attachInterrupt(chan2Pin, ISR_INPUT, CHANGE);
  attachInterrupt(chan3Pin, ISR_INPUT, CHANGE);
  attachInterrupt(chan4Pin, ISR_INPUT, CHANGE);
}

void IRAM_ATTR ISR_INPUT()
{
  current_time = micros();
  //Channel 1=========================================ROLL
  if(digitalRead(chan1Pin) == HIGH)
  {                                                                         
    if(last_channel_1 == 0)                                                 //Is input 8 high?
    {                                                                       //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1)
  {                                                                         //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================PITCH
  if(digitalRead(chan2Pin) == HIGH)
  {                                                                         //Is input 9 high?
    if(last_channel_2 == 0)
    {                                                                       //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1)
  {                                                                         //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================YAW
  if(digitalRead(chan3Pin) == HIGH)
  {                                                                         //Is input 10 high?
    if(last_channel_3 == 0)
    {                                                                       //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1)
  {                                                                         //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[0] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================THROTTLE
  if(digitalRead(chan4Pin) == HIGH)
  {                                                                         //Is input 11 high?
    if(last_channel_4 == 0)
    {                                                                        //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1)
  {                                                                         //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}
