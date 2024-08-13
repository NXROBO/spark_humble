#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign
from rclpy.parameter import Parameter

class VoiceNav(Node):
    def __init__(self):
        super().__init__('voice_nav')

        
        #rospy.on_shutdown(self.cleanup)
        
        # Set a number of parameters affecting the robot's speed
        self.max_speed = self.get_parameter_or('~max_speed', Parameter('aaa', Parameter.Type.DOUBLE, 0.4)).value
        self.max_angular_speed = self.get_parameter_or('~max_angular_speed', Parameter('bbb', Parameter.Type.DOUBLE, 1.5)).value
        self.speed = self.get_parameter_or("~start_speed", Parameter('ccc', Parameter.Type.DOUBLE, 0.1)).value
        self.angular_speed = self.get_parameter_or("~start_angular_speed", Parameter('ddd', Parameter.Type.DOUBLE, 0.5)).value
        self.linear_increment = self.get_parameter_or("~linear_increment", Parameter('eee', Parameter.Type.DOUBLE, 0.05)).value
        self.angular_increment = self.get_parameter_or("~angular_increment", Parameter('fff', Parameter.Type.DOUBLE, 0.4)).value

        # We don't have to run the script very fast
        self.rate = self.get_parameter_or("~rate", Parameter('ggg', Parameter.Type.INTEGER, 5)).value
        self.r = self.create_rate(self.rate) 
        self.STEPS = self.get_parameter_or("~steps", Parameter('hhh', Parameter.Type.INTEGER, 20)).value
        self.stp_counts=0
        
        # A flag to determine whether or not voice control is paused
        self.paused = False
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribe to the /voice/stt topic to receive voice commands.
        self.sub1 = self.create_subscription(String, '/voice/stt', self.speech_callback, 10)
        self.sub1
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'stop': ['stop','Stop','停止','停'],
                                    'forward': ['forward','Forward','前进','前進'],
                                    'backward': ['backward','Backward','后退','後退'],
                                    'turn left': ['left','Left','左转','左轉'],
                                    'turn right': ['right','Right','右转','右轉']}
        
        self.get_logger().info("Ready to receive voice commands")
        self.create_timer(timer_period_sec=0.2, callback=self.timer_callback)

    def timer_callback(self):

        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        if rclpy.ok():
            self.stp_counts+=1
            if(self.stp_counts>=self.STEPS):
                self.cmd_vel = Twist()
                self.stp_counts=0
            self.cmd_vel_pub.publish(self.cmd_vel)
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        print(data)
        for (command, keywords) in self.keywords_to_command.items():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speech_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = self.get_command(msg.data)
        self.stp_counts=0
        
        # Log the command to the screen
        self.get_logger().info("Command: " + str(command))
        # If the user has asked to pause/continue voice control,
        # set the flag accordingly 
        if command == 'pause':
            self.paused = True
        elif command == 'continue':
            self.paused = False
        
        # If voice control is paused, simply return without
        # performing any action
        if self.paused:
            return       
        
        # The list of if-then statements should be fairly
        # self-explanatory
        if command == 'forward':    
            self.cmd_vel.linear.x = self.speed
            self.cmd_vel.angular.z = 0.0
            
        elif command == 'rotate left':
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.angular_speed
                
        elif command == 'rotate right':  
            self.cmd_vel.linear.x = 0.0      
            self.cmd_vel.angular.z = -self.angular_speed
            
        elif command == 'turn left':
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z += self.angular_increment
            else:        
                self.cmd_vel.angular.z = self.angular_speed
                
        elif command == 'turn right':    
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.angular.z -= self.angular_increment
            else:        
                self.cmd_vel.angular.z = -self.angular_speed
                
        elif command == 'backward':
            self.cmd_vel.linear.x = -self.speed
            self.cmd_vel.angular.z = 0.0
            
        elif command == 'stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            self.cmd_vel = Twist()
        
        elif command == 'faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x += copysign(self.linear_increment, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z += copysign(self.angular_increment, self.cmd_vel.angular.z)
            
        elif command == 'slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x -= copysign(self.linear_increment, self.cmd_vel.linear.x)
            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z -= copysign(self.angular_increment, self.cmd_vel.angular.z)
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed = copysign(self.max_speed / 4, self.speed)
        
            elif command == 'half':
                self.speed = copysign(self.max_speed / 2, self.speed)
            
            elif command == 'full':
                self.speed = copysign(self.max_speed, self.speed)
            
            if self.cmd_vel.linear.x != 0:
                self.cmd_vel.linear.x = copysign(self.speed, self.cmd_vel.linear.x)

            if self.cmd_vel.angular.z != 0:
                self.cmd_vel.angular.z = copysign(self.angular_speed, self.cmd_vel.angular.z)
                
        else:
            return

        self.cmd_vel.linear.x = min(self.max_speed, max(-self.max_speed, self.cmd_vel.linear.x))
        self.cmd_vel.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.cmd_vel.angular.z))

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rclpy.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    g_node = VoiceNav()
    g_node.get_logger().info("=============spin")

    rclpy.spin(g_node)
    g_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass

