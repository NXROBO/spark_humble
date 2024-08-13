/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>   
 * ROS2 Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */


#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <swiftpro/msg/position.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
using namespace std::chrono_literals;
#define MATH_PI 				3.141592653589793238463
#define MATH_TRANS  			57.2958    
#define MATH_L1 				106.6
#define MATH_L2 				13.2
#define MATH_LOWER_ARM 			142.07
#define MATH_UPPER_ARM 			158.81

float motor_angle[3] = {90.0, 90.0, 0.0};
rclcpp::Node::SharedPtr this_node = nullptr;
	
/* 
 * Description: forward kinematics of swift pro
 * Inputs: 		angle[3]			3 motor angles(degree)
 * Outputs:		position[3]			3 cartesian coordinates: x, y, z(mm)
 */
void swift_fk(float angle[3], float position[3])
{
	double stretch = MATH_LOWER_ARM * cos(angle[1] / MATH_TRANS) 
				   + MATH_UPPER_ARM * cos(angle[2] / MATH_TRANS) + MATH_L2 + 56.55;

	double height = MATH_LOWER_ARM * sin(angle[1] / MATH_TRANS) 
				  - MATH_UPPER_ARM * sin(angle[2] / MATH_TRANS) + MATH_L1;
	
	position[0] = stretch * sin(angle[0] / MATH_TRANS);
	position[1] = -stretch * cos(angle[0] / MATH_TRANS);
	position[2] = height - 74.55;
}


/* 
 * Description: callback when receive data from move_group/fake_controller_joint_states
 * Inputs: 		msg					3 necessary joints for kinematic chain(degree)
 * Outputs:		motor_angle[3]		3 motor angles(degree)
 */
void joint_Callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
	motor_angle[0] = msg->position[0] * 57.2958 + 90;
	motor_angle[1] = 90 - msg->position[1] * 57.2958;
	motor_angle[2] = (msg->position[1] + msg->position[2]) * 57.2958;
}


/* 
 * Node name:
 *	 swiftpro_moveit_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   position_write_topic
 *
 * Topic subscribe: (queue size = 1)
 *   move_group/fake_controller_joint_states
 */


int main(int argc, char * argv[])
{
	float position[3];
	swiftpro::msg::Position pos;

	rclcpp::init(argc, argv);
	this_node = rclcpp::Node::make_shared("swiftpro_moveit_node");
	auto publisher = this_node->create_publisher<swiftpro::msg::Position>("position_write_topic", 1);
	auto sub = this_node->create_subscription<sensor_msgs::msg::JointState>("move_group/fake_controller_joint_states", 1, joint_Callback);

	rclcpp::WallRate loop_rate(20);

	while (rclcpp::ok()) 
	{
		swift_fk(motor_angle, position);
		pos.x = position[0];
		pos.y = position[1];
		pos.z = position[2];
		try {
			publisher->publish(pos);	
			rclcpp::spin_some(this_node);
		} 
		catch (const rclcpp::exceptions::RCLError & e) {
			RCLCPP_ERROR(
				this_node->get_logger(),
				"unexpectedly failed with %s",
				e.what());
		}
		loop_rate.sleep();
	}
	rclcpp::shutdown();
	return 0;
}