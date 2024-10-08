/*
 *  Copyright (c) 2022, NXROBO Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  Authors: Litian Zhuang <litian.zhuang@nxrobo.com>
 */

#include "spark_base/spark_base_interface.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <netinet/in.h>
#include <sys/types.h>
#include <cstdlib>

#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

// *****************************************************************************
// Constructor
NxSparkBase::SparkBaseInterface::SparkBaseInterface(std::string dev_name)
{
  this->resetOdometry();
  this->wheel_dist = 0;

  encoder_counts_[LEFT] = 0;
  encoder_counts_[RIGHT] = 0;

  last_encoder_counts_[LEFT] = 0;
  last_encoder_counts_[RIGHT] = 0;

  is_first_time_left = true;
  is_first_time_right = true;
  serial_port_ = std::shared_ptr<SparkSerial>(new SparkSerial(dev_name, 115200));
}

// *****************************************************************************
// Destructor
NxSparkBase::SparkBaseInterface::~SparkBaseInterface()
{
  // Clean up!
  // delete serial_port_;
}

// *****************************************************************************
// Open the serial port
int NxSparkBase::SparkBaseInterface::openSerialPort()
{
  if (serial_port_->OpenSerial() < 0)
  {
    return -1;
  }

  this->startOI();
  return (0);
}

int NxSparkBase::SparkBaseInterface::GetDataGram(unsigned char *r_buffer, int *length)
{
  return serial_port_->GetDataGram(r_buffer, length);
}
// *****************************************************************************
// check sum
unsigned char NxSparkBase::SparkBaseInterface::checkSum(unsigned char *buf)
{
  unsigned char sum = 0;
  int i;
  for (i = 0; i < buf[2] - 3; i++)
  {
    sum += buf[i];
  }
  return sum;
}

// *****************************************************************************
// Set the mode
int NxSparkBase::SparkBaseInterface::startOI(void)
{
  unsigned char buffer[8];
  buffer[0] = 0x53;                              // headcode1
  buffer[1] = 0x4b;                              // headcode2
  buffer[3] = 0x00;                              // cmd1
  buffer[4] = 0x16;                              // cmd2
  buffer[2] = 2;                                 // communicate num
  buffer[5] = checkSum((unsigned char *)buffer); // checksum
  buffer[6] = 0x0d;                              // endcode1
  buffer[7] = 0x0a;                              // endcode2
  usleep(1000 * 100);

  serial_port_->WriteBuffer(buffer, 8);

  return (0);
}

// *****************************************************************************
// Send an OP code to the spark base
int NxSparkBase::SparkBaseInterface::sendOpcode(int code)
{
  unsigned char buffer[8];
  buffer[0] = 0x53;                              // headcode1
  buffer[1] = 0x4b;                              // headcode2
  buffer[2] = 8;                                 // communicate num
  buffer[3] = code >> 8;                         // cmd1
  buffer[4] = code & 0x00ff;                     // cmd2
  buffer[5] = checkSum((unsigned char *)buffer); // checksum
  buffer[6] = 0x0d;                              // endcode1
  buffer[7] = 0x0a;                              // endcode2

  serial_port_->WriteBuffer(buffer, buffer[2]);

  return (0);
}
// *****************************************************************************
// Send an OP code9 to the spark base
int NxSparkBase::SparkBaseInterface::sendOpcode9(int code, unsigned char value)
{
  unsigned char buffer[9];
  buffer[0] = 0x53;                              // headcode1
  buffer[1] = 0x4b;                              // headcode2
  buffer[2] = 9;                                 // communicate num
  buffer[3] = code >> 8;                         // cmd1
  buffer[4] = code & 0x00ff;                     // cmd2
  buffer[5] = value;                             // value
  buffer[6] = checkSum((unsigned char *)buffer); // checksum
  buffer[7] = 0x0d;                              // endcode1
  buffer[8] = 0x0a;                              // endcode2

  serial_port_->WriteBuffer(buffer, buffer[2]);

  return (0);
}
// *****************************************************************************
// Close the serial port
int NxSparkBase::SparkBaseInterface::closeSerialPort()
{
  this->drive(0.0, 0.0);
  usleep(1000 * 100);
  serial_port_->CloseSerial();

  return (0);
}

// *****************************************************************************
// Set the speeds
int NxSparkBase::SparkBaseInterface::drive(double linear_speed, double angular_speed)
{
  // int left_speed_mm_s =
  // (int)((linear_speed-SPARKBASE_AXLE_LENGTH*angular_speed/2)*1e3);		// Left
  // wheel velocity in mm/s
  // int right_speed_mm_s =
  // (int)((linear_speed+SPARKBASE_AXLE_LENGTH*angular_speed/2)*1e3);	// Right
  // wheel velocity in mm/s
  //调换了左右轮子的速度,使得半弧向后
  int left_speed_mm_s =
      (int)((linear_speed - SPARKBASE_AXLE_LENGTH * angular_speed / 2) * 1e3); // Left wheel velocity in mm/s
  int right_speed_mm_s =
      (int)((linear_speed + SPARKBASE_AXLE_LENGTH * angular_speed / 2) * 1e3); // Right wheel velocity in mm/s

  //    printf("%d,%d\n",left_speed_mm_s,right_speed_mm_s);
  return this->driveDirect(left_speed_mm_s, right_speed_mm_s);
}

// *****************************************************************************
// Set the motor speeds
int NxSparkBase::SparkBaseInterface::driveDirect(int left_speed, int right_speed)
{
  // Limit velocity
  int16_t left_speed_mm_s = MAX(left_speed, -SPARKBASE_MAX_LIN_VEL_MM_S);
  left_speed_mm_s = MIN(left_speed, SPARKBASE_MAX_LIN_VEL_MM_S);
  int16_t right_speed_mm_s = MAX(right_speed, -SPARKBASE_MAX_LIN_VEL_MM_S);
  right_speed_mm_s = MIN(right_speed, SPARKBASE_MAX_LIN_VEL_MM_S);

  int16_t left_speed_pwm_s = left_speed_mm_s / SPARKBASE_PULSES_TO_MM;
  int16_t right_speed_pwm_s = right_speed_mm_s / SPARKBASE_PULSES_TO_MM;

  // printf("left_wheel_speed,right_wheel_speed,%d,%d\n",left_speed_pwm_s,right_speed_pwm_s);
  return this->drivePWM(left_speed_pwm_s, right_speed_pwm_s);
  return (0);
}

// *****************************************************************************
// Set the motor PWMs
int NxSparkBase::SparkBaseInterface::drivePWM(int left_speed_pwm_s, int right_speed_pwm_s)
{
  // Compose comand

  unsigned char buffer[12];
  buffer[0] = 0x53;                              // headcode1
  buffer[1] = 0x4b;                              // headcode2
  buffer[2] = 0x0c;                              // 12 all communicate num
  buffer[3] = 0x00;                              // cmd1
  buffer[4] = 0x18;                              // cmd2
  buffer[5] = (char)left_speed_pwm_s;            // data1
  buffer[6] = (char)(left_speed_pwm_s >> 8);     // data2
  buffer[7] = (char)right_speed_pwm_s;           // data3
  buffer[8] = (char)(right_speed_pwm_s >> 8);    // data4
  buffer[9] = checkSum((unsigned char *)buffer); // checksum
  buffer[10] = 0x0d;                             // endcode1
  buffer[11] = 0x0a;                             // endcode2
                                                 /*printf("%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-\n",
                                                        buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],
                                                         buffer[7],buffer[8],buffer[9],buffer[10],buffer[11]);*/

  serial_port_->WriteBuffer(buffer, buffer[2]);

  return (0);
}

// *****************************************************************************
// Read the sensors
int NxSparkBase::SparkBaseInterface::getSensorPackets()
{
  unsigned char buffer[12];
  buffer[0] = 0x53;                              // headcode1
  buffer[1] = 0x4b;                              // headcode2+6
  buffer[2] = 8;                                 // communicate num
  buffer[3] = 0x00;                              // cmd1
  buffer[4] = 0x16;                              // cmd2
  buffer[5] = checkSum((unsigned char *)buffer); // checksum
  buffer[6] = 0x0d;                              // endcode1
  buffer[7] = 0x0a;                              // endcode2
                                                 // Fill in the command buffer to send to the robot

  serial_port_->WriteBuffer(buffer, buffer[2]);

  return (0);
}

// *****************************************************************************

int NxSparkBase::SparkBaseInterface::parseSenseState(unsigned char *buffer, int index)
{
  // cliff, Bumps, wheeldrops
  this->cliff_[RIGHT] = (((buffer[index] >> 0) & 0x01) == 0) ? 1 : 0;
  this->cliff_[FRONT_RIGHT] = (((buffer[index] >> 1) & 0x01) == 0) ? 1 : 0;
  this->cliff_[FRONT_LEFT] = (((buffer[index] >> 2) & 0x01) == 0) ? 1 : 0;
  this->cliff_[LEFT] = (((buffer[index] >> 3) & 0x01) == 0) ? 1 : 0;
  this->cliff_[BACK_RIGHT] = (((buffer[index] >> 4) & 0x01) == 0) ? 1 : 0;
  this->cliff_[BACK_LEFT] = (((buffer[index] >> 5) & 0x01) == 0) ? 1 : 0;

  this->ir_bumper_[RIGHT] = (buffer[index + 1]) & 0x01;
  this->ir_bumper_[FRONT_RIGHT] = (buffer[index + 1] >> 1) & 0x01;
  this->ir_bumper_[FRONT] = (buffer[index + 1] >> 2) & 0x01;
  this->ir_bumper_[FRONT_LEFT] = (buffer[index + 1] >> 3) & 0x01;
  this->ir_bumper_[LEFT] = (buffer[index + 1] >> 4) & 0x01;
  this->ir_bumper_[BACK_RIGHT] = (buffer[index + 1] >> 5) & 0x01;
  this->ir_bumper_[BACK_LEFT] = (buffer[index + 1] >> 6) & 0x01;

  this->wheel_drop_[RIGHT] = (buffer[index + 2] >> 0) & 0x01;
  this->wheel_drop_[LEFT] = (buffer[index + 2] >> 1) & 0x01;
  this->wheel_over_current_[RIGHT] = (buffer[index + 2] >> 2) & 0x01;
  this->wheel_over_current_[LEFT] = (buffer[index + 2] >> 3) & 0x01;

  this->search_dock_ = (buffer[index + 2] >> 5) & 0x01;
  this->touch_charge_ = (buffer[index + 2] >> 6) & 0x01;
  this->plug_charge_ = (buffer[index + 2] >> 7) & 0x01;

  this->dock_direction_[LEFT] = (buffer[index + 3] >> 4) & 0x01;
  this->dock_direction_[FRONT] = (buffer[index + 3] >> 5) & 0x01;
  this->dock_direction_[RIGHT] = (buffer[index + 3] >> 6) & 0x01;
  this->dock_direction_[BACK] = (buffer[index + 3] >> 7) & 0x01;

  this->dock_ = ((buffer[index] >> 2) & 0x01);
  this->control_ = (buffer[index + 2] >> 4) & 0x01;
  this->error_ = (buffer[index + 1] >> 7) & 0x01;
  return (0);
}

int NxSparkBase::SparkBaseInterface::parseRightEncoderCounts(unsigned char *buffer, int index)
{
  int encoder_counts_right_tmp;
  // Right encoder counts
  unsigned int right_encoder_counts = buffer2unsigned_int(buffer, index);
  right_encoder_counts = -right_encoder_counts;
  //    printf("Right Encoder: %d,%d,%d\n",
  //    right_encoder_counts,last_encoder_counts_[RIGHT],right_encoder_counts-last_encoder_counts_[RIGHT]);

  if (is_first_time_right ||
      right_encoder_counts == last_encoder_counts_[RIGHT]) // First time, we need 2 to make it work!
  {
    encoder_counts_right_tmp = 0;
    is_first_time_right = false;
  }
  else
  {
    encoder_counts_right_tmp = (int)(right_encoder_counts - last_encoder_counts_[RIGHT]);

    if (encoder_counts_right_tmp > SPARKBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_right_tmp = encoder_counts_right_tmp - SPARKBASE_MAX_ENCODER_COUNTS;
    if (encoder_counts_right_tmp < -SPARKBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_right_tmp = SPARKBASE_MAX_ENCODER_COUNTS + encoder_counts_right_tmp;
  }
  /*static double rc_count = 0;
      rc_count +=   encoder_counts_[RIGHT];
    std::cout<<"sum rec:"<<rc_count<<rc_count*SPARKBASE_PULSES_TO_M<<std::endl;*/
    if(abs(encoder_counts_right_tmp) < 1000)
    {
        encoder_counts_[RIGHT] = encoder_counts_right_tmp;
        last_encoder_counts_[RIGHT] = right_encoder_counts;
    }
    else {
        printf("Right Encoder: %d,%d,%d\n",
        right_encoder_counts,last_encoder_counts_[RIGHT],right_encoder_counts-last_encoder_counts_[RIGHT]);
    }
    //    printf("Right Encoder: %d\n", encoder_counts_[RIGHT]);
  return 0;
}

int NxSparkBase::SparkBaseInterface::parseLeftEncoderCounts(unsigned char *buffer, int index)
{
  int encoder_counts_left_tmp;
  // Left encoder counts
  unsigned int left_encoder_counts = buffer2unsigned_int(buffer, index);
  left_encoder_counts = -left_encoder_counts;
  //    printf("Left Encoder: %d,%d,%d\n", left_encoder_counts,
  //    last_encoder_counts_[LEFT],left_encoder_counts-last_encoder_counts_[LEFT]);

  if (is_first_time_left ||
      left_encoder_counts == last_encoder_counts_[LEFT]) // First time, we need 2 to make it work!
  {
    encoder_counts_left_tmp = 0;
    is_first_time_left = false;
  }
  else
  {
    encoder_counts_left_tmp = (int)(left_encoder_counts - last_encoder_counts_[LEFT]);

    if (encoder_counts_left_tmp > SPARKBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_left_tmp = encoder_counts_left_tmp - SPARKBASE_MAX_ENCODER_COUNTS;
    if (encoder_counts_left_tmp < -SPARKBASE_MAX_ENCODER_COUNTS / 10)
      encoder_counts_left_tmp = SPARKBASE_MAX_ENCODER_COUNTS + encoder_counts_left_tmp;
  }
  if(abs(encoder_counts_left_tmp) < 1000)
  {
        encoder_counts_[LEFT] = encoder_counts_left_tmp;
        last_encoder_counts_[LEFT] = left_encoder_counts;
  }
  else {
      printf("Left Encoder: %d,%d,%d\n", left_encoder_counts,
      last_encoder_counts_[LEFT],left_encoder_counts-last_encoder_counts_[LEFT]);
  }
  return 0;
}
int NxSparkBase::SparkBaseInterface::parseWheelDiffTime(unsigned char *buffer, int index)
{
  current_time = buffer2unsigned_int(buffer, index);
  // printf("current_time:%d",current_time);
  return 0;
}
  union Char2Float_
  {
    float value;
    unsigned char buffer[4];
  };
int NxSparkBase::SparkBaseInterface::parseImuData(unsigned char *buffer, int index)
{
    Char2Float_ getvalue[12];
    memcpy(&getvalue, buffer+index, sizeof(float) * 12);
    float gyro_yaw = getvalue[11].value;

    imu_angle = gyro_yaw / 180 * 3.1415926535898;
  return 0;
}

void NxSparkBase::SparkBaseInterface::parseComInterfaceData(unsigned char *buf, int index)
{
  parseLeftEncoderCounts(buf, index + 14);
  parseRightEncoderCounts(buf, index + 10);
  parseSenseState(buf, index + 5);
  parseWheelDiffTime(buf, index + 18);
  parseImuData(buf, index + 22);  
}
int NxSparkBase::SparkBaseInterface::buffer2signed_int(unsigned char *buffer, int index)
{
  unsigned int unsigned_int;
  unsigned_int = buffer[index] | (buffer[index + 1] << 8) | (buffer[index + 2] << 16) | (buffer[index + 3] << 24);
  return (int)unsigned_int;
}

unsigned int NxSparkBase::SparkBaseInterface::buffer2unsigned_int(unsigned char *buffer, int index)
{
  unsigned int unsigned_int;
  unsigned_int = buffer[index] | (buffer[index + 1] << 8) | (buffer[index + 2] << 16) | (buffer[index + 3] << 24);
  return unsigned_int;
}

// *****************************************************************************
// Calculate Sparkbase odometry
void NxSparkBase::SparkBaseInterface::calculateOdometry()
{
  // double dist = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M +
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / 2.0;
  // double ang = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M -
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / -SPARKBASE_AXLE_LENGTH;
  //该版本特性：半弧为向前的方向
  // double dist = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M +
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / 2.0;
  // double ang = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M -
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / SPARKBASE_AXLE_LENGTH;
  //该版本特性：半弧为向后的方向

  double dist =
      ((double)encoder_counts_[RIGHT] * SPARKBASE_PULSES_TO_M + (double)encoder_counts_[LEFT] * SPARKBASE_PULSES_TO_M) /
      -2.0;
  double ang =
      ((double)encoder_counts_[RIGHT] * SPARKBASE_PULSES_TO_M - (double)encoder_counts_[LEFT] * SPARKBASE_PULSES_TO_M) /
      SPARKBASE_AXLE_LENGTH;

  // Update odometry
  this->odometry_x_ = this->odometry_x_ + dist * cos(odometry_yaw_); // m
  this->odometry_y_ = this->odometry_y_ + dist * sin(odometry_yaw_); // m
  this->odometry_yaw_ = NORMALIZE(this->odometry_yaw_ + ang);        // rad
  this->wheel_dist = this->wheel_dist + dist;
}
// Calculate Sparkbase odometry
void NxSparkBase::SparkBaseInterface::calculateOdometry_new()
{
  // double dist = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M +
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / 2.0;
  // double ang = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M -
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / -SPARKBASE_AXLE_LENGTH;
  //该版本特性：半弧为向前的方向
  // double dist = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M +
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / 2.0;
  // double ang = (encoder_counts_[RIGHT]*SPARKBASE_PULSES_TO_M -
  // encoder_counts_[LEFT]*SPARKBASE_PULSES_TO_M) / SPARKBASE_AXLE_LENGTH;
  //该版本特性：半弧为向后的方向
  double step_time;
  double c_t;
  double v = 0.0;
  double w = 0.0;
  static unsigned int last_base_time=0; 
  double theta = 0.0;
  double delta_theta = 0.0;

  static double last_theta = 0.0;
  double dist =
      ((double)encoder_counts_[RIGHT] * SPARKBASE_PULSES_TO_M + (double)encoder_counts_[LEFT] * SPARKBASE_PULSES_TO_M) /
      -2.0;

  if (true) {
    theta = imu_angle;
    delta_theta = theta - last_theta;
  } else {
   theta = ((double)encoder_counts_[RIGHT] * SPARKBASE_PULSES_TO_M - (double)encoder_counts_[LEFT] * SPARKBASE_PULSES_TO_M) /
      SPARKBASE_AXLE_LENGTH;
    delta_theta = theta;
  }
  // Update odometry
  this->odometry_x_ += dist * cos(this->odometry_yaw_ + (delta_theta / 2.0));
  this->odometry_y_ += dist * sin(this->odometry_yaw_ + (delta_theta / 2.0));
  this->odometry_yaw_ += delta_theta;


  this->wheel_dist = this->wheel_dist + dist;
  // compute odometric instantaneouse velocity
  c_t = current_time - last_base_time;
  last_base_time = current_time;
  step_time = c_t * 0.0001;
  v = dist / step_time;
  w = delta_theta / step_time;

  robot_vel_[0] = v;
  robot_vel_[1] = 0.0;
  robot_vel_[2] = w;

  last_theta = theta;
}

// *****************************************************************************
// Reset Sparkbase odometry
void NxSparkBase::SparkBaseInterface::resetOdometry()
{
  this->setOdometry(0.0, 0.0, 0.0);
}

// *****************************************************************************
// Set Sparkbase odometry
void NxSparkBase::SparkBaseInterface::setOdometry(double new_x, double new_y, double new_yaw)
{
  this->odometry_x_ = new_x;
  this->odometry_y_ = new_y;
  this->odometry_yaw_ = new_yaw;
}

// *****************************************************************************
// Go to the dock
int NxSparkBase::SparkBaseInterface::goDock(int dock)
{
  int opcode;
  if (dock)
    opcode = 0x0010;
  else
    opcode = 0x0005;
  return sendOpcode(opcode);
}
// *****************************************************************************
// search to the dock
int NxSparkBase::SparkBaseInterface::searchDock(int flag)
{
  int opcode = 0x0017; // search cmd
  unsigned char value;
  if (flag)
    value = 0x00;
  else
    value = 0x01;
  return sendOpcode9(opcode, value);
}

// *****************************************************************************

// EOF
