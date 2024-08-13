#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess


class VoiceTTS(Node):
    def __init__(self):
        super().__init__('ekho_tts')
        self.subscription = self.create_subscription(
            String,
            '/voice/tts',
            self.tts_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Ready to receive TTS')

    def tts_callback(self, msg):        
        # Log the tts to the screen
        self.get_logger().info("TTS: " + str(msg.data))
        subprocess.call(["ekho",msg.data])
        

def main(args=None):
    rclpy.init(args=args)
    voice_ttS = VoiceTTS()

    rclpy.spin(voice_ttS)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voice_ttS.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()