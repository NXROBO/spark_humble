import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image as ImageMsg
import insightface
from insightface.app import FaceAnalysis
from insightface.data import get_image as ins_get_image


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


class  Insightface_node(Node):

    def __init__(self):
        super().__init__('Insightface_node')
        self.get_logger().info('Insightface')
        #self.app = FaceAnalysis(allowed_modules=['detection']) #指定只使用detection检测模型
        #self.app = FaceAnalysis(name='buffalo_sc') #指定模型名称buffalo_sc为最小模型，适合算力不高的情况
        self.app = FaceAnalysis() #默认为buffalo_l模型
        self.app.prepare(ctx_id=0, det_size=(640, 640))
        self.input_topic_p = self.declare_parameter('input_topic', '/camera/color/image_raw')
        img_topic = self.input_topic_p.value
        self.create_subscription(ImageMsg, img_topic, self.image_detection_callback, 1)
        self.image_pub = self.create_publisher(ImageMsg, 'detections_image', 1)

    def image_detection_callback(self, img_msg):
        image_np = image_msg_to_image_np(img_msg)
        image_np = image_np[:, :, ::-1] #RGB转BGR
        faces = self.app.get(image_np)
        #results.print()
        rimg = self.app.draw_on(image_np, faces)
        img_msg = image_np_to_image_msg(rimg[:, :, ::-1])
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)

    insightface_node = Insightface_node()

    rclpy.spin(insightface_node)
    insightface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
