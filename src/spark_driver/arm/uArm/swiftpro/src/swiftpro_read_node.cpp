/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>
 *		   David Long <xiaokun.long@ufactory.cc>	   
 * ROS2 Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
 

#include <serial/serial.h>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include "swiftpro/msg/swiftpro_state.hpp"


serial::Serial _serial;				// serial object
float position[4] = {0.0};			// 3 cartesian coordinates: x, y, z(mm) and 1 angle(degree)
char  strdata[2048];				// global variables for handling string


void handlestr()
{
	char  *pch = strtok(strdata, " ");
	float value[8];
	int   index = 0;

	while (pch != NULL && index < 5)
	{
		value[index] = atof(pch+1);
		pch = strtok(NULL, " ");
		index++;
	}
	position[0] = value[1];
	position[1] = value[2];
	position[2] = value[3];
	position[3] = value[4];
}


void handlechar(char c)
{
	static int index = 0;

	switch(c)
	{
		case '\r':
			break;	

		case '\n':
			strdata[index] = '\0';
			handlestr();
			index = 0;
			break;

		default:
			strdata[index++] = c;
			break;
	}
}

/* 
 * Node name:
 *	 swiftpro_read_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *   position_read_topic
 */
int main(int argc, char** argv)
{	
	rclcpp::init(argc, argv);
	auto this_node = rclcpp::Node::make_shared("swiftpro_read_node");
	swiftpro::msg::SwiftproState swiftpro_state;
	std_msgs::msg::String result;
	auto publisher = this_node->create_publisher<swiftpro::msg::SwiftproState>("SwiftproState_topic", 1);
	rclcpp::WallRate loop_rate(20);
	try
	{
		_serial.setPort("/dev/ttyACM0");
		_serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		_serial.setTimeout(to);
		_serial.open();
		RCLCPP_INFO(this_node->get_logger(), "Port has been open successfully");
	}
	catch (serial::IOException& e)
	{
		RCLCPP_ERROR(this_node->get_logger(), "Unable to open port");
		return -1;
	}
	
	if (_serial.isOpen())
	{
		rclcpp::sleep_for(std::chrono::milliseconds(3000));			// wait 3s
		_serial.write("M2019\r\n");				// detach
		rclcpp::sleep_for(std::chrono::milliseconds(500));				// wait 0.5s
		_serial.write("M2120 V0.05\r\n");		// report position per 0.05s
		RCLCPP_INFO(this_node->get_logger(), "Start to report data");
	}
	while (rclcpp::ok()) 						// publish positionesian coordinates
	{
		if (_serial.available())
		{
			result.data = _serial.read(_serial.available());
			// ROS_INFO_STREAM("Read:" << result.data);
			for (int i = 0; i < result.data.length(); i++)
				handlechar(result.data.c_str()[i]);

			swiftpro_state.pump = 0;
			swiftpro_state.gripper = 0;
			swiftpro_state.swiftpro_status = 0;
			swiftpro_state.motor_angle1 = 0.0;
			swiftpro_state.motor_angle2 = 0.0;
			swiftpro_state.motor_angle3 = 0.0;
			swiftpro_state.motor_angle4 = position[3];
			swiftpro_state.x = position[0];
			swiftpro_state.y = position[1];
			swiftpro_state.z = position[2];
			publisher->publish(swiftpro_state);
			RCLCPP_INFO(this_node->get_logger(), "position: %.2f %.2f %.2f %.2f", position[0], position[1], position[2], position[3]);
		}
		rclcpp::spin_some(this_node);
		loop_rate.sleep();
	}	
  	rclcpp::shutdown();
}




