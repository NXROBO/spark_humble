/*
 * Copyright (c) 2016, SHENZHEN NXROBO Co.,LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *      Author: Xiankai Chen (xiankai.chen@nxrobo.com) and Jian Song
 *		(jian.song@nxrobo.com)
 */

#include <chrono>
#include <memory>
#include <stdint.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <sstream>
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
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <boost/make_shared.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/qos.hpp>


#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
//#include <pcl_ros/transforms.h>
using namespace std::chrono_literals;



class NxFollowerNode : public rclcpp::Node
{

private:
  // cmd_vel publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub;

  // point clound subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  // scan subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

  double target_angle_deg_;  /**< The target angle in degrees (positive is clockwise from forward). */
  double angle_range_deg_;   /**< The angle range in degrees to detect obstacles. */
  double min_valid_distance_; /**< The minimum valid distance to consider a point. */
  double max_valid_distance_; /**< The maximum valid distance to consider a point. */

  double min_y_;   /**< The minimum y position of the points in the box. */
  double max_y_;   /**< The maximum y position of the points in the box. */
  double min_x_;   /**< The minimum x position of the points in the box. */
  double max_x_;   /**< The maximum x position of the points in the box. */
  double max_z_;   /**< The maximum z position of the points in the box. */
  double goal_z_;  /**< The distance away from the robot to hold the centroid */
  double z_scale_; /**< The scaling factor for translational robot speed */
  double x_scale_; /**< The scaling factor for rotational robot speed */
  double z_thre;
  double x_thre;
  double max_vx;                 /*max velocity x*/
  double max_vz;                 /*max velocity z*/
  double max_depth_, min_depth_; /**< The maximum z position of the points in the box. */
  double goal_depth_;            /**< The distance away from the robot to hold the centroid */
  double depth_thre;
  double y_thre;

  bool stop_buff;
  double min_dis;

public:
  NxFollowerNode() : min_y_(0.1), max_y_(0.5), min_x_(-0.2), max_x_(0.2), max_z_(0.8), goal_z_(0.6), z_scale_(1.0), x_scale_(5.0), Node("spark_follower")
  {

    min_x_ = -0.2;
    max_x_ = 0.2;
    min_y_ = -0.1;
    max_y_ = 0.3;
    max_z_ = 1.5;
    goal_z_ = 0.7;
    z_scale_ = 0.8;
    x_scale_ = 2;
    z_thre = 0.05;
    x_thre = 0.05;
    y_thre = 0.087222222;

    max_vx = 0.4;
    max_vz = 0.8;

    max_depth_ = 2;
    min_depth_ = 0.4;
    goal_depth_ = 0.9;
    depth_thre = 0.1;
    y_thre = 0.087222222;

    target_angle_deg_ = 180; // 目标角度（正后方）
    angle_range_deg_ = 60;   // 检测角度范围
    min_valid_distance_ = 0.15;
    max_valid_distance_ = 0.8;

    stop_buff = false;  // 障碍物检测标识
    min_dis = 0;

    cmdvel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 1, std::bind(&NxFollowerNode::pointCloudCb, this, std::placeholders::_1));
    
    // 添加雷达scan订阅
    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
       "/scan", rclcpp::SensorDataQoS(), std::bind(&NxFollowerNode::scanCb, this, std::placeholders::_1));


  }
  virtual ~NxFollowerNode()
  {
  }

  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
      // 将目标角度转换为弧度
      double target_angle_rad = target_angle_deg_ * M_PI / 180.0;
      double angle_range_rad = angle_range_deg_ * M_PI / 180.0;

      // 计算角度边界
      double angle_start = target_angle_rad - angle_range_rad / 2.0;
      double angle_end = target_angle_rad + angle_range_rad / 2.0;
  
      // 规范化角度到 [-π, π] 范围
      auto normalize_angle = [](double angle) {
          while (angle > M_PI) angle -= 2.0 * M_PI;
          while (angle <= -M_PI) angle += 2.0 * M_PI;
          return angle;
      };
      
      angle_start = normalize_angle(angle_start);
      angle_end = normalize_angle(angle_end);
      // 输出检测角度范围
      // RCLCPP_INFO(this->get_logger(), "规范化检测角度范围: %.0f度到%.0f度", 
      //           angle_start * 180.0 / M_PI, angle_end * 180.0 / M_PI);
      
      // 获取总数据点数
      size_t total_points = msg->ranges.size();
      if (total_points == 0) {
          RCLCPP_WARN(this->get_logger(), "激光雷达数据为空");
          return;
      }
      
      // 计算起始和结束索引
      int index_start = static_cast<int>((angle_start - msg->angle_min) / msg->angle_increment);
      int index_end = static_cast<int>((angle_end - msg->angle_min) / msg->angle_increment);
      // RCLCPP_INFO(this->get_logger(), "激光雷达数据索引范围: %d到%d", index_start, index_end);
      
      // 确保索引在有效范围内
      index_start = std::max(0, std::min(index_start, static_cast<int>(total_points - 1)));
      index_end = std::max(0, std::min(index_end, static_cast<int>(total_points - 1)));
      
      // 处理角度范围跨越360度边界的情况
      std::vector<int> target_indices;
      if (index_start <= index_end) {
          for (int i = index_start; i <= index_end; ++i) {
              target_indices.push_back(i);
          }
      } else {
          // 处理跨越边界的情况
          for (int i = index_start; i < static_cast<int>(total_points); ++i) {
              target_indices.push_back(i);
          }
          for (int i = 0; i <= index_end; ++i) {
              target_indices.push_back(i);
          }
      }
      // 输出target_indices
      // RCLCPP_INFO(this->get_logger(), "检测角度范围: %.0f度到%.0f度", 
      //           angle_start * 180.0 / M_PI, angle_end * 180.0 / M_PI);
      // RCLCPP_INFO(this->get_logger(), "目标索引范围: %d到%d", index_start, index_end);
      
      // 计算有效距离的平均值和统计信息
      double sum_distance = 0.0;
      int valid_count = 0;
      double min_distance = std::numeric_limits<double>::max();
      double max_distance = std::numeric_limits<double>::min();
      
      for (int idx : target_indices) {
          if (idx < 0 || idx >= static_cast<int>(total_points)) {
              continue;
          }
          
          float range = msg->ranges[idx];
          // RCLCPP_INFO(this->get_logger(), "激光雷达数据索引 %d: 距离 %.3f米", idx, static_cast<double>(range));

          
          // 检查距离值是否有效
          if (std::isfinite(range) && 
              range >= min_valid_distance_ && 
              range <= max_valid_distance_  ) {
              
              sum_distance += range;
              // RCLCPP_INFO(this->get_logger(), "有效距离: %.3f米", static_cast<double>(range));
              valid_count++;
              min_distance = std::min(min_distance, static_cast<double>(range));
              max_distance = std::max(max_distance, static_cast<double>(range));     
              min_dis = min_distance;
              // RCLCPP_WARN(this->get_logger(), "最小障碍物距离: %.3f米", min_distance);
              // RCLCPP_WARN(this->get_logger(), "最大障碍物距离: %.3f米", max_distance);

          }
      }

      // 输出valid_count
      // RCLCPP_INFO(this->get_logger(), "有效点数量: %d", valid_count);

      // 输出结果
      if (valid_count > 0) {
          double average_distance = sum_distance / valid_count;
          // RCLCPP_INFO(this->get_logger(), 
          //           "后方%.0f度范围内平均障碍物距离: %.3f米 (基于%d/%d个有效点)", 
          //           target_angle_deg_ * 180.0 / M_PI, average_distance, 
          //           valid_count, target_indices.size());
          stop_buff = false;
          // 根据距离进行后续处理
          if (min_dis < 0.4) {
              RCLCPP_WARN(this->get_logger(), "后方近距离障碍物警告: %.3f米", min_dis);
              // 触发安全反应机制
              // publish_cmdvel(0.0, 0.0, 0.0);
              stop_buff = true;
              // return;
          }
      } else {
          // RCLCPP_WARN(this->get_logger(), 
          //           "后方%.0f度范围内未检测到有效障碍物数据", 
          //           target_angle_deg_ * 180.0 / M_PI);
      }
  }

  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;

    // PCL still uses boost::shared_ptr internally
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    // This will convert the message into a pcl::PointCloud
    pcl::fromROSMsg(*msg, *cloud);

    // Number of points observed
    unsigned int n = 0;
    pcl::PointXYZRGB pt;
    for (int kk = 0; kk < cloud->points.size(); kk++)
    {
      pt = cloud->points[kk];

      if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
      {
        if (-pt.y > min_y_ && -pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        {
          // Add the point to the totals
          x += pt.x;
          z += pt.z;

          n++;
        }
      }
    }

    if (n > 2000)
    {

      x /= n;
      z /= n;

      // RCLCPP_WARN(this->get_logger(), "x:%.3f, z:%.3f", x, z);

      if (z > max_z_)
      {
        publish_cmdvel(0.0, 0.0, 0.0);
        return;
      }
      if (stop_buff){
        if (z>=0.82){
          pubCmd(-x, z);
        }else{
          publish_cmdvel(0.0, 0.0, 0.0);
        }
      }else{
        pubCmd(-x, z);
      }


    }
    else
    {
      publish_cmdvel(0.0, 0.0, 0.0);
    }
  }

  void pubCmd(const float &y, const float &depth)
  {
    double curr_dist = sqrt(y * y + depth * depth);
    if (curr_dist == 0)
    {
      publish_cmdvel(0.0, 0.0, 0.0);
      return;
    }

    float x_linear = 0;
    float z_angular = 0;
    float z_scale = 1.2;
    float x_scale = 5.0; // 2.0
    x_linear = (depth - goal_depth_) * z_scale;
    z_angular = asin(y / curr_dist) * x_scale;

    if (depth_thre > fabs(depth - goal_depth_))
      x_linear = 0;
    if (y_thre > y && y > -y_thre)
      z_angular = 0;

    publish_cmdvel(x_linear, 0.0, z_angular);
  }
  void publish_cmdvel(float x, float y, float z)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = x;
    cmd.linear.y = y;
    cmd.linear.z = 0.0;

    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = z;
    cmdvel_pub->publish(cmd);
  }
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NxFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
