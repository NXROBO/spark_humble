/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, NXROBO Ltd.
 *  litian.zhuang <litian.zhuang@nxrobo.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 113
#define KEYCODE_E 101
/* 带有shift键 */
#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

#define KEYCODE_Q_CAP 81
#define KEYCODE_E_CAP 69

#define KEYCODE_I 73
#define KEYCODE_K 75
#define KEYCODE_J 74
#define KEYCODE_L 76

#define KEYCODE_I_CAP 105
#define KEYCODE_K_CAP 107
#define KEYCODE_J_CAP 106
#define KEYCODE_L_CAP 108
using namespace std;

#define NONE "\e[0m"
#define BLACK "\e[0;30m"
#define RED "\e[0;31m"
#define GREEN "\e[0;32m"
#define YELLOW "\e[1;33m"
#define BLUE "\e[1;34m"
#define WHITE "\e[1;37m"
#define GRAY "\e[0;37m"
#define CLEAR "\033[2J"
#define CYAN "\e[0;36m"
#define MOVETO(x,y) printf("\033[%d;%dH", (x), (y))

using namespace std::chrono_literals;


class SmartCarKeyboardTeleop : public rclcpp::Node
{
private:
  double walk_vel_;
  double yaw_rate_;
  geometry_msgs::msg::Twist cmdvel_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  int kfd;
  struct termios cooked, raw;
  float speed_linear_x;
  float speed_angular_z;
  boost::thread t;

public:
  SmartCarKeyboardTeleop(float linear = 0.2, float angular = 0.4): Node("spark_teleop_node")
  {
    kfd = 0;
    speed_linear_x = linear;
    speed_angular_z = angular;
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    this->declare_parameter("walk_vel", 0.2);
    this->declare_parameter("yaw_rate", 1.0);
    this->get_parameter_or<double>("walk_vel", walk_vel_, 0.2);
    this->get_parameter_or<double>("yaw_rate", walk_vel_, 1.0);
    t = boost::thread(boost::bind(&SmartCarKeyboardTeleop::keyboardLoop, this));
  }

  ~SmartCarKeyboardTeleop()
  {
    /* set the terminal's parameters */
    tcsetattr(kfd, TCSANOW, &cooked);
    t.interrupt();
    t.join();
    stopRobot();
  }

  void stopRobot()
  {
    cmdvel_.linear.x = 0.0;
    cmdvel_.angular.z = 0.0;
    this->pub_->publish(cmdvel_);
  }

  void PrintfColour(std::string keystr)
  {
    printf(CLEAR);
    MOVETO(0,0);
    if(keystr == "")
        printf(YELLOW "为了更好的体验本操作，请右键点击本窗口的菜单栏，选择“Always On Top”" NONE);
    printf("\n");
    printf("请根据提示选择移动方向：\n\n");
    if(keystr == "UP")
        printf(GREEN "           w前进" NONE);
    else
	printf("           w前进");
    printf("\n");
    printf("\n");
    if(keystr == "LEFT")
        printf(YELLOW "    a左转" NONE "   停止  d右转 ");
    else if(keystr == "RIGHT")
        printf("    a左转   停止" BLUE "  d右转 " NONE);
    else if(keystr == "STOP")
        printf("    a左转  " RED " 停止" NONE "  d右转 ");
    else
	printf("    a左转   停止  d右转 ");    
    printf("\n");
    printf("\n");
    if(keystr == "DOWN")
        printf(CYAN "           s后退" NONE);
    else
	printf("           s后退");
    printf("\n");
  }
  void keyboardLoop()
  {
    char c;
    double max_speed_linear_x = walk_vel_;
    double max_speed_angular_z = yaw_rate_;
    float speed = 0;
    float turn = 0;
    int sparkbasebit = 0;
    bool dirty = false;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));

    raw.c_lflag &= ~(ICANON | ECHO);

    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

 //   puts("Reading from keyboard");
//    puts("Use WASD keys to control the robot");
//    puts("Press Shift to move faster");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    PrintfColour("");
    for (;;)
    {
      boost::this_thread::interruption_point();
      sparkbasebit = 0;
      /* get the next event from the keyboard */
      int num;

      if ((num = poll(&ufd, 1, 500)) < 0)
      {
        perror("poll():");
        return;
      }
      else if (num > 0)
      {
        if (read(kfd, &c, 1) < 0)
        {
          perror("read():");
          return;
        }
      }
      else
      {
        if (dirty == true)
        {
          stopRobot();
          dirty = false;
          PrintfColour("STOP");
        }
        continue;
      }
      //printf("------------%d\n", c);
      switch (c)
      {
        case KEYCODE_W:
          max_speed_linear_x = speed_linear_x;
          speed = 1;
          turn = 0;
          dirty = true;
          PrintfColour("UP");
          sparkbasebit = 1;
          break;
        case KEYCODE_S:
          max_speed_linear_x = speed_linear_x;
          speed = -1;
          turn = 0;
          dirty = true;
          PrintfColour("DOWN");
          sparkbasebit = 1;
          break;
        case KEYCODE_A:
          max_speed_angular_z = speed_angular_z;
          speed = 0;
          turn = 1;
          dirty = true;
          PrintfColour("LEFT");
          sparkbasebit = 1;
          break;
        case KEYCODE_D:
          max_speed_angular_z = speed_angular_z;
          speed = 0;
          turn = -1;
          dirty = true;
          PrintfColour("RIGHT");
          sparkbasebit = 1;
          break;

        case KEYCODE_W_CAP:
          max_speed_linear_x = speed_linear_x;
          speed = 1;
          turn = 0;
          dirty = true;
          //ROS_INFO("[UP]");
	        PrintfColour("UP");
          sparkbasebit = 1;
          break;
        case KEYCODE_S_CAP:
          max_speed_linear_x = speed_linear_x;
          speed = -1;
          turn = 0;
          dirty = true;
          PrintfColour("DOWN");
          sparkbasebit = 1;
          break;
        case KEYCODE_A_CAP:
          max_speed_angular_z = speed_angular_z;
          speed = 0;
          turn = 1;
          dirty = true;
          PrintfColour("LEFT");
          sparkbasebit = 1;
          break;
        case KEYCODE_D_CAP:
          max_speed_angular_z = speed_angular_z;
          speed = 0;
          turn = -1;
          dirty = true;
          PrintfColour("RIGHT");
          sparkbasebit = 1;
          break;
        case KEYCODE_Q_CAP:
          max_speed_angular_z = speed_angular_z;
          max_speed_linear_x = speed_linear_x;
          speed = 1;
          turn = 1;
          dirty = true;
          RCLCPP_INFO(this->get_logger(), "[LEFTUP]");
          sparkbasebit = 1;
          break;
        case KEYCODE_E_CAP:
          max_speed_angular_z = speed_angular_z;
          max_speed_linear_x = speed_linear_x;
          speed = 1;
          turn = -1;
          dirty = true;
          RCLCPP_INFO(this->get_logger(), "[RIGHTUP]");
          sparkbasebit = 1;
          break;
        case KEYCODE_Q:
          max_speed_angular_z = speed_angular_z;
          max_speed_linear_x = speed_linear_x;
          speed = 1;
          turn = 1;
          dirty = true;
          RCLCPP_INFO(this->get_logger(), "[LEFTUP]");
          sparkbasebit = 1;
          break;
        case KEYCODE_E:
          max_speed_angular_z = speed_angular_z;
          max_speed_linear_x = speed_linear_x;
          speed = 1;
          turn = -1;
          dirty = true;
          RCLCPP_INFO(this->get_logger(), "[RIGHTUP]");
          sparkbasebit = 1;
          break;
        default:
          max_speed_linear_x = speed_linear_x;
          max_speed_angular_z = speed_angular_z;
          speed = 0;
          turn = 0;
          RCLCPP_INFO(this->get_logger(), "[STOP NOW]");
          dirty = false;
          sparkbasebit = 1;
      }
      if (sparkbasebit == 1)
      {
        cmdvel_.linear.x = speed * max_speed_linear_x;
        cmdvel_.angular.z = turn * max_speed_angular_z;
        this->pub_->publish(cmdvel_);
      }
    }
  }
};

int main(int argc, char **argv)
{
  float linear = 0.2;
  float angular = 0.4;
  rclcpp::init(argc, argv);
  if (argc != 3)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Useage:ros2 run teleop teleop_node linear angular, such as:ros2 run teleop teleop_node 0.2 1 ");
    return 1;
  }
  linear = atof(argv[1]);
  angular = atof(argv[2]);
  auto teleop_node = std::make_shared<SmartCarKeyboardTeleop>(linear, angular);
  rclcpp::spin(teleop_node);
  rclcpp::shutdown();
  return 0;
}
