#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import logging
import time
from threading import Thread, Event
from .lib import Microphone,AipSpeech

""" 你的 APPID AK SK """
APP_ID = '23797700'
API_KEY = 'dK8rz5DG9pPi90cgQVtx1ddn'
SECRET_KEY = 'X1lzayj5RQwfkoLHXHd5DALRvFZpjhBP'

def main(args=None):
    logging.basicConfig(level=logging.DEBUG)
    quit_event = Event()
    mic = Microphone(quit_event=quit_event)
    client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

    rclpy.init(args=args)
    g_node = rclpy.create_node('ali_asr')    
    pub = g_node.create_publisher(String, 'voice/stt', 10)
    while rclpy.ok():
        data = mic.listen()
        result = client.asr(b''.join(list(data)), 'pcm', 16000, {'dev_pid': 1537,})
        print (result)
        if result['err_no']==0 and len(result["result"][0])>1:
            text = result["result"][0]
            pub.publish(text)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
