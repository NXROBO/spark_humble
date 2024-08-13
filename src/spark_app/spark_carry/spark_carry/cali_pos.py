#!/usr/bin/env python3
import sys
import rclpy
import time
import _thread
from std_msgs.msg import String
from swiftpro.msg import *
start_cali = 0
g_node = None

def msg_callback(data):
	global start_cali
	if(data.data == "start"):
		start_cali = 1
	else:
		start_cali = 0
def talker(threadName, delay):
	global g_node
	global sta
	global start_cali
	pub1 = g_node.create_publisher(Position, 'position_write_topic', 10)
	pub2 = g_node.create_publisher(Status, 'cali_pix_topic', 10)


	pos = Position()
	sta = Status()

	r1 = g_node.create_rate(1)  # 1s
	r2 = g_node.create_rate(0.05) # 20s
	r3 = g_node.create_rate(0.2)  # 5s
	r4 = g_node.create_rate(0.4)  # 2.5s
	t_num = g_node.count_subscribers('position_write_topic')

	while(t_num < 1):
		r1.sleep()
		t_num = g_node.count_subscribers('position_write_topic')
		print('the subscribers is ', t_num)
	print('---------------------the subscribers is ', t_num)

	pos.x = 120.0
	pos.y = 0.0
	pos.z = 35.0
	pub1.publish(pos)
	print('cali_pos pub(ori):', pos.x, pos.y)

	# r2.sleep()
	time.sleep( 10 )
	print ("wait to start----")
	while(start_cali==0):
		r1.sleep()
	if(start_cali == 0):
		print("start_cali == 0 exit")
		return
	else :
		time.sleep( 5 )
		# r1.sleep()
		# r1.sleep()
		# r1.sleep()
		# r1.sleep()
		# r1.sleep()
	print ("start to move the uarm----")
	for i in range(20):	
		if not rclpy.ok():
			break
		# r1.sleep()
		time.sleep( 1 )
		pos.x = 180.0 + i * 5
		pos.y = 200.0 - i * 10
		pos.z = -110.0
		pub1.publish(pos)
		print('cali_pos pub:', pos.x, pos.y)
		# r1.sleep()
		# time.sleep( 1 )
		if i == 0:
			# r2.sleep()
			time.sleep( 6 )
			
		else:
			# r4.sleep()
			time.sleep( 1.5 )
		sta.status = 1
		pub2.publish(sta)
		# r4.sleep()
		time.sleep( 1.5 )
	pos.x = 120.0
	pos.y = 0.0
	pos.z = 35.0
	pub1.publish(pos)

	r3.sleep()

def main(args=None):
	global g_node
	rclpy.init(args=args)
	g_node = rclpy.create_node('cali_pos')
	sub1 = g_node.create_subscription(String, 'start_topic', msg_callback, 10)
	_thread.start_new_thread(talker, ("Thread-1", 2, ) )
	try:
		rclpy.spin(g_node)
	except KeyboardInterrupt:
		print("Shutting down")


if __name__ == '__main__':
	main()



