#ifndef SPARK_BASE_DRIVER_H
#define SPARK_BASE_DRIVER_H
#include <iostream>
#include <thread>
#include <memory>
#include <atomic>
#include <cstdlib>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executor.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "spark_base/spark_base_serial.h"
#include "spark_base/spark_base_constants.h"
#include "spark_base/spark_base_interface.h"
#include "spark_base/msg/spark_base_sensor.hpp"
#include "spark_base/msg/spark_base_dock.hpp"
#include "spark_base/msg/gyro_message.hpp"
#include "spark_base/msg/spark_base_odom.hpp"
#include "spark_base/kfilter.hpp"
#include "spark_base/mylock.hpp"

#define NODE_VERSION 0.01
#define SPARKBASETIMEOUT (1000 * 1e3) //超过1s
#define COUNT_TIMES 20
using namespace std::chrono_literals;

namespace NxSparkBase
{

  class SparkBaseDriver : public rclcpp::Node
  {
  public:
    SparkBaseDriver(std::string node_name);
    ~SparkBaseDriver();

  private:
    void hex_printf(unsigned char *buf, int len);
    void getSparkbaseComData(unsigned char *buf, int len);
    unsigned char checkSum(unsigned char *buf);
    int pubGyroMessage(unsigned char *buf, int len);
    void dealMessageSwitch(unsigned char *recvbuf);
    void startComParamInit();
    void resetOdomCb(const spark_base::msg::SparkBaseOdom::SharedPtr odom);
    void cmdVelReceived(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    int pubWheelJointStates(double linear_speed, double angular_speed);
    void handleGoDock(const std_msgs::msg::String::SharedPtr msg);
    void handleSearchDock(const std_msgs::msg::String::SharedPtr msg);
    void process_base_receive_thread();
    void checkSerialGoon();
    void publish_odom();
  private:
    std::string base_frame_id;
    std::string odom_frame_id;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    // tf::TransformBroadcaster tf_broadcaster;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr search_sub;
    rclcpp::Subscription<spark_base::msg::SparkBaseOdom>::SharedPtr odom_reset_sub;
    std::shared_ptr<SparkBaseInterface> interface_port_;

    std::string serial_port;
    // rclcpp::Timer stimer;
    rclcpp::TimerBase::SharedPtr stimer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr fback_cmd_vel_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub;
    rclcpp::Publisher<spark_base::msg::SparkBaseSensor>::SharedPtr rb_sensor_pub;
    rclcpp::Publisher<spark_base::msg::SparkBaseDock>::SharedPtr rb_dock_pub;
    rclcpp::Publisher<spark_base::msg::GyroMessage>::SharedPtr gyro_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double last_x, last_y, last_yaw;
    double vel_x, vel_y, vel_yaw;
    double dt;
    int idx;
    unsigned int countSerial, lastCountSerial;
    double fb_time[COUNT_TIMES], fb_dist[COUNT_TIMES], fb_dist_x[COUNT_TIMES], odom_x[COUNT_TIMES], odom_y[COUNT_TIMES],
        odom_yaw[COUNT_TIMES], vel_x_list[COUNT_TIMES], vel_y_list[COUNT_TIMES];
    double robot_yaw;
    NxSparkBase::KFilter odom_x_kfilter, odom_y_kfilter;
    double left_wheel_position, right_wheel_position;
    std::shared_ptr<std::thread> base_receive_thread_;
  };

}

#endif // SPARK_BASE_DRIVER_H
