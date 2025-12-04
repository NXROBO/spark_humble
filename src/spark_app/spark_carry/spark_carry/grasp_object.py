#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time 
import random
import ctypes
import rclpy
from rclpy.node import Node
import threading
import _thread
import string
import math
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from swiftpro.srv import *
from swiftpro.msg import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped 


class GraspObject(Node):
    def __init__(self):
        super().__init__('grasp_object')
        self.get_logger().info('GraspObject node initialized')
        # 订阅目标中心点信息
        # 修改订阅者创建
        self.sub = self.create_subscription(PointStamped, 'target_center', self.target_center_cb, 10)
        
        # 创建发布者
        self.pos_pub = self.create_publisher(Position, 'position_write_topic', 10)
        self.pump_pub = self.create_publisher(Status, 'pump_topic', 10)

        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        arr=s.split()
        file_pix.close()
        print(s)

        global a, b
        a = [0]*2
        b = [0]*2
        a[0]=float(arr[0])
        a[1]=float(arr[1])
        b[0]=float(arr[2])
        b[1]=float(arr[3])
        print('k and b value:',a[0],a[1],b[0],b[1])

        self.grasping = False
        self.direction = True

    def target_center_cb(self, msg):
        """目标中心点回调函数"""
        if self.grasping:
            return
        # 修改时间戳获取方式
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f"Received message at timestamp: {timestamp}")
        # 获取当前的时间戳
        current_time = self.get_clock().now().nanoseconds * 1e-9
        # 比较时间戳，确保消息是最近的
        if current_time - timestamp > 0.5:
            self.get_logger().warn("Message is stale, discarding")
            return
        
        self.grasping = True
        # 修改坐标获取方式
        xc = int(msg.point.x)
        yc = int(msg.point.y)
        self.get_logger().info(f"Received target center: ({xc}, {yc})")
        
        # 发布位置信息
        pos_msg = Position()
        pos_msg.x = a[0] * yc + a[1]
        pos_msg.y = b[0] * xc + b[1]
        pos_msg.z = 20.0
        print("z = 20.0\n")
        self.pos_pub.publish(pos_msg)
        # 输出位置信息
        self.get_logger().info(f"Published position: ({pos_msg.x}, {pos_msg.y}, {pos_msg.z})")
        
        # 设置休眠    
        time.sleep(3)

        # go down 
        pos_msg.z = -50.0
        self.pos_pub.publish(pos_msg)
        print("z = -50.0\n")
        time.sleep(2)

        # start pump
        pump_act = Status()
        pump_act.status = 1
        self.pump_pub.publish(pump_act)
        time.sleep(2)

        # go back home
        pos_msg.x = 180.0
        pos_msg.y = 0.0
        pos_msg.z = 50.0
        self.pos_pub.publish(pos_msg)
        time.sleep(3)

        # go left_right  pos
        if self.direction:
            pos_msg.x = 150.0
            pos_msg.y = 150.0
            pos_msg.z = 35.0
            self.pos_pub.publish(pos_msg)
            time.sleep(3)
            self.direction = False
        else:
            pos_msg.x = 150.0
            pos_msg.y = -150.0
            pos_msg.z = 35.0
            self.pos_pub.publish(pos_msg)
            time.sleep(3)
            self.direction = True

        pos_msg.z = -50.0
        self.pos_pub.publish(pos_msg)
        time.sleep(2)

        # start pump
        pump_act = Status()
        pump_act.status = 0
        self.pump_pub.publish(pump_act)
        time.sleep(2)

        # go back home
        pos_msg.x = 180.0
        pos_msg.y = 0.0
        pos_msg.z = 50.0
        self.pos_pub.publish(pos_msg)
        time.sleep(3)

        self.grasping = False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        grasp_object = GraspObject()
        rclpy.spin(grasp_object)
    except KeyboardInterrupt:
        pass
    finally:
        grasp_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()