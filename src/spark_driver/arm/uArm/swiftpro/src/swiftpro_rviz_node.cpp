/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>   
 */


 #include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>

#include "swiftpro/msg/swiftpro_state.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MATH_PI 				3.141592653589793238463
#define MATH_TRANS  			57.2958    
#define MATH_L1 				106.6
#define MATH_L2 				13.2
#define MATH_LOWER_ARM 			142.07
#define MATH_UPPER_ARM 			158.81	
#define MATH_UPPER_LOWER 		(MATH_UPPER_ARM / MATH_LOWER_ARM)

#define LOWER_ARM_MAX_ANGLE     135.6
#define LOWER_ARM_MIN_ANGLE     0
#define UPPER_ARM_MAX_ANGLE     100.7
#define UPPER_ARM_MIN_ANGLE     0
#define LOWER_UPPER_MAX_ANGLE   151
#define LOWER_UPPER_MIN_ANGLE   10

float joint_angle[9] = {0.0};		// 9 joint angles of swiftpro(degree)

/* 
 * Description: Get 9 joint angles from 3 motor angles
 * Inputs: 		angle[3]			3 motor angles(degree)
 * Outputs:		joint_angle[9]		9 joint angles(degree)
 */
void all_joints_state(float angle[3])
{
	double alpha2;
	double alpha3;
	
	alpha2 = angle[1];
	alpha3 = angle[2] - 3.8;
	
	// 3 necessary joints for kinematic chain
	joint_angle[0] = angle[0] - 90;
	joint_angle[1] = 90 - alpha2;
	joint_angle[5] = alpha3;

	// 6 passive joints for display
	joint_angle[2] = (alpha2 + alpha3) - 176.11 + 90;
	joint_angle[3] = -90 + alpha2;
	joint_angle[4] = joint_angle[1];
	joint_angle[6] = 90 - (alpha2 + alpha3);
	joint_angle[7] = 176.11 - 180 - alpha3;
	joint_angle[8] = 48.39 + alpha3 - 44.55;
}


/* 
 * Description: inverse kinematics of swift pro
 * Inputs: 		position[3]			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		angle[3]			3 motor angles(degree)
 */
bool swiftpro_ik(float position[3], float angle[3])
{
	float x = position[0];
	float y = position[1];
	float z = position[2];
	float xIn, zIn, phi, rightAll, sqrtZX = 0.0;
	float angleRot, angleLeft, angleRight = 0.0;
	
	z += 74.55;
	zIn = (z - MATH_L1) / MATH_LOWER_ARM;
	
	if (x < 0.1)
		x = 0.1;

	// calculate value of theta1: the rotation angle
	if (y == 0)
		angleRot = 90;
	else if (y < 0)
		angleRot = -atan(x / y) * MATH_TRANS;
	else if (y > 0)
		angleRot = 180 - atan(x / y) * MATH_TRANS;

	xIn 	= (x / sin(angleRot / MATH_TRANS) - MATH_L2 - 56.55) / MATH_LOWER_ARM;
	phi 	= atan(zIn / xIn) * MATH_TRANS;
	sqrtZX 	= sqrt(zIn * zIn + xIn * xIn);
	rightAll   = (sqrtZX * sqrtZX + MATH_UPPER_LOWER * MATH_UPPER_LOWER  - 1) 
			   / (2 * MATH_UPPER_LOWER  * sqrtZX);
	angleRight = acos(rightAll) * MATH_TRANS;

	// calculate value of theta2 and theta3
	rightAll   = (sqrtZX * sqrtZX + 1 - MATH_UPPER_LOWER * MATH_UPPER_LOWER ) / (2 * sqrtZX);
	angleLeft  = acos(rightAll) * MATH_TRANS;
	angleLeft  = angleLeft + phi;
	angleRight = angleRight - phi;

	if (std::isnan(angleRot) || std::isnan(angleLeft) || std::isnan(angleRight))
		return false;

	angle[0] = angleRot;
	angle[1] = angleLeft;
	angle[2] = angleRight;
	return true;
}


/* 
 * Description: callback when receive data from position_read_topic
 * Inputs: 		msg(SwiftproState)	data about swiftpro
 * Outputs:		joint_angle[9]		9 joint angles(degree)
 */
void SwiftproState_Callback(const swiftpro::msg::SwiftproState::SharedPtr msg)
{
	float position[3];
	float angle[3];
	
	position[0] = msg->x;
	position[1] = msg->y;
	position[2] = msg->z;
	
	if ( swiftpro_ik(position, angle) )
		all_joints_state(angle);
	else
		printf("Inverse kinematic is wrong");

}
auto createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

/* 
 * Node name:
 *	 swiftpro_rviz_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 joint_states
 *
 * Topic subscribe: (queue size = 1)
 *	 SwiftproState_topic
 */
int main(int argc, char **argv)
{	
	rclcpp::init(argc, argv);
	auto this_node = rclcpp::Node::make_shared("swiftpro_rviz_node");
	swiftpro::msg::SwiftproState swiftpro_state;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	auto sub = this_node->create_subscription<swiftpro::msg::SwiftproState>(
        "SwiftproState_topic", 1, SwiftproState_Callback);
	auto publisher = this_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this_node);

	rclcpp::WallRate loop_rate(20);
	sensor_msgs::msg::JointState 		joint_state;
    geometry_msgs::msg::TransformStamped odom_trans;
	rclcpp::Time time_now;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id  = "Base";
	while (rclcpp::ok())
	{

		joint_state.header.stamp = this_node->get_clock()->now();
		joint_state.name.resize(9);
		joint_state.position.resize(9);
		joint_state.name[0] = "Joint1";
		joint_state.position[0] = joint_angle[0] / 57.2958;
	    joint_state.name[1] = "Joint2";
		joint_state.position[1] = joint_angle[1] / 57.2958;
		joint_state.name[2] = "Joint3";
		joint_state.position[2] = joint_angle[2] / 57.2958;
		joint_state.name[3] = "Joint4";
		joint_state.position[3] = joint_angle[3] / 57.2958;
		joint_state.name[4] = "Joint5";
		joint_state.position[4] = joint_angle[4] / 57.2958;
		joint_state.name[5] = "Joint6";
		joint_state.position[5] = joint_angle[5] / 57.2958;
		joint_state.name[6] = "Joint7";
		joint_state.position[6] = joint_angle[6] / 57.2958;
		joint_state.name[7] = "Joint8";
		joint_state.position[7] = joint_angle[7] / 57.2958;
		joint_state.name[8] = "Joint9";
		joint_state.position[8] = joint_angle[8] / 57.2958;

		odom_trans.header.stamp = this_node->get_clock()->now();
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = createQuaternionMsgFromYaw(10); //tf::createQuaternionMsgFromYaw(10);
		
		publisher->publish(joint_state);
		tf_broadcaster_->sendTransform(odom_trans);
		rclcpp::spin_some(this_node);
		loop_rate.sleep();
	}		
	rclcpp::shutdown();
	return 0;
}

