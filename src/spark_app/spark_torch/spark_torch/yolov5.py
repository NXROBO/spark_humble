import rclpy
from rclpy.node import Node
from rclpy import qos
import cv2
import torch
import numpy as np
from sensor_msgs.msg import Image as ImageMsg


def image_msg_to_image_np(image_msg):
    n_channels = 3
    dtype = 'uint8'
    img_buf = np.asarray(image_msg.data, dtype=dtype)

    image_np = np.ndarray(shape=(image_msg.height, image_msg.width, n_channels),
                          dtype=dtype, buffer=img_buf)

    return image_np


def image_np_to_image_msg(image_np):
    image_msg = ImageMsg()

    image_msg.height = image_np.shape[0]
    image_msg.width = image_np.shape[1]
    image_msg.encoding = 'rgb8'
    image_msg.data = image_np.tostring()
    image_msg.step = len(image_msg.data) // image_msg.height
    image_msg.header.frame_id = 'map'

    return image_msg


class  Yolov5_node(Node):

    def __init__(self):
        super().__init__('Yolov5_node')
        self.get_logger().info('yolov5s')
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.input_topic_p = self.declare_parameter('input_topic', '/camera/color/image_raw')
        img_topic = self.input_topic_p.value
        self.create_subscription(ImageMsg, img_topic, self.image_detection_callback, 1)
        self.image_pub = self.create_publisher(ImageMsg, 'detections_image', 1)

    def image_detection_callback(self, img_msg):
        image_np = image_msg_to_image_np(img_msg)
        results = self.model(image_np)
        #results.print()
        results.ims # array of original images (as np array) passed to model for inference
        results.render()  # updates results.ims with boxes and labels
        for im in results.ims:
            img_msg = image_np_to_image_msg(im)
            self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)

    yolov5_node = Yolov5_node()

    rclpy.spin(yolov5_node)
    yolov5_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()