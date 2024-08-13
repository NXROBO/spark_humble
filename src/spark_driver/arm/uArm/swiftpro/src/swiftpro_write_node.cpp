/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2017, UFactory, Inc.
 * All rights reserved.
 * Author: Roger Cui  <roger@ufactory.cc>	   
 */



#include <serial/serial.h>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "swiftpro/msg/swiftpro_state.hpp"
#include <swiftpro/msg/status.hpp>
#include <swiftpro/msg/position.hpp>
#include <swiftpro/msg/angle4th.hpp>
serial::Serial _serial;				// serial object
swiftpro::msg::SwiftproState pos;
rclcpp::Node::SharedPtr this_node = nullptr;

/* 
 * Description: callback when receive data from position_write_topic
 * Inputs: 		msg(float)			3 cartesian coordinates: x, y, z(mm)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void position_write_callback(const swiftpro::msg::Position::SharedPtr msg)
{
	std::string Gcode = "";
	std_msgs::msg::String result;
	char x[10];
	char y[10];
	char z[10];

	pos.x = msg->x;
	pos.y = msg->y;
	pos.z = msg->z;
	sprintf(x, "%.2f", msg->x);
	sprintf(y, "%.2f", msg->y);
	sprintf(z, "%.2f", msg->z);
	Gcode = (std::string)"G0 X" + x + " Y" + y + " Z" + z + " F10000" + "\r\n";
	//printf("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from angle4th_topic
 * Inputs: 		msg(float)			angle of 4th motor(degree)
 * Outputs:		Gcode				send gcode to control swift pro
 */
void angle4th_callback(const swiftpro::msg::Angle4th::SharedPtr msg)
{
	std::string Gcode = "";
	std_msgs::msg::String result;
	char m4[10];
	
	pos.motor_angle4 = msg->angle4th;
	sprintf(m4, "%.2f", msg->angle4th);
	Gcode = (std::string)"G2202 N3 V" + m4 + "\r\n";
	printf("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from swiftpro_status_topic
 * Inputs: 		msg(uint8)			status of gripper: attach if 1; detach if 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void swiftpro_status_callback(const swiftpro::msg::Status::SharedPtr msg)
{
	std::string Gcode = "";
	std_msgs::msg::String result;

	if (msg->status == 1)
		Gcode = (std::string)"M17\r\n";   // attach
	else if (msg->status == 0)
		Gcode = (std::string)"M2019\r\n";
	else
	{
		printf("Error:Wrong swiftpro status input");
		return;
	}
	
	pos.swiftpro_status = msg->status;
	RCLCPP_INFO(this_node->get_logger(), "%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from gripper_topic
 * Inputs: 		msg(uint8)			status of gripper: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void gripper_callback(const swiftpro::msg::Status::SharedPtr msg)
{
	std::string Gcode = "";
	std_msgs::msg::String result;

	if (msg->status == 1)
		Gcode = (std::string)"M2232 V1" + "\r\n";
	else if (msg->status == 0)
		Gcode = (std::string)"M2232 V0" + "\r\n";
	else
	{
		printf("Error:Wrong gripper status input");
		return;
	}
	
	pos.gripper = msg->status;
	RCLCPP_INFO(this_node->get_logger(), "%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Description: callback when receive data from pump_topic
 * Inputs: 		msg(uint8)			status of pump: work if 1; otherwise 0
 * Outputs:		Gcode				send gcode to control swift pro
 */
void pump_callback(const swiftpro::msg::Status::SharedPtr msg)
{
	std::string Gcode = "";
	std_msgs::msg::String result;

	if (msg->status == 1)
		Gcode = (std::string)"M2231 V1" + "\r\n";
	else if (msg->status == 0)
		Gcode = (std::string)"M2231 V0" + "\r\n";
	else
	{
		printf("Error:Wrong pump status input");
		return;
	}
	
	pos.pump = msg->status;
	printf("%s", Gcode.c_str());
	_serial.write(Gcode.c_str());
	result.data = _serial.read(_serial.available());
}


/* 
 * Node name:
 *	 swiftpro_write_node
 *
 * Topic publish: (rate = 20Hz, queue size = 1)
 *	 swiftpro_state_topic
 *
 * Topic subscribe: (queue size = 1)
 *	 position_write_topic
 *	 swiftpro_status_topic
 *	 angle4th_topic
 *	 gripper_topic
 *	 pump_topic
 */
int main(int argc, char** argv)
{	
	rclcpp::init(argc, argv);
	swiftpro::msg::SwiftproState swiftpro_state;

	this_node = rclcpp::Node::make_shared("swiftpro_moveit_node");
	auto publisher = this_node->create_publisher<swiftpro::msg::SwiftproState>("SwiftproState_topic", 1);
	auto sub1 = this_node->create_subscription<swiftpro::msg::Status>("swiftpro_status_topic", 1, swiftpro_status_callback);
	auto sub3 = this_node->create_subscription<swiftpro::msg::Angle4th>("angle4th_topic", 1, angle4th_callback);
	auto sub4 = this_node->create_subscription<swiftpro::msg::Status>("gripper_topic", 1, gripper_callback);
	auto sub5 = this_node->create_subscription<swiftpro::msg::Status>("pump_topic", 1, pump_callback);

	rclcpp::WallRate loop_rate(20);
	try
	{
		_serial.setPort("/dev/ttyACM0");	//ttyACM0
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
		//_serial.write("M2019\r\n");				// detach
		rclcpp::sleep_for(std::chrono::milliseconds(1000));				// wait 0.5s
		//_serial.write("M2120 V0.5\r\n");		// report position per 0.05s
		RCLCPP_INFO(this_node->get_logger(), "Start to report data");
	}
	auto sub2 = this_node->create_subscription<swiftpro::msg::Position>("position_write_topic", 10, position_write_callback);

	while (rclcpp::ok()) 						// publish positionesian coordinates
	{
		publisher->publish(pos);
		rclcpp::spin_some(this_node);
		loop_rate.sleep();
	}	
  	rclcpp::shutdown();
	return 0;
}


