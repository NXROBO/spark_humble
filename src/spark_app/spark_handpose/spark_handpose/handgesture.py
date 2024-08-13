import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image as ImageMsg

import cv2
import os
import torch
import torch.nn.functional as F
from .resnet import resnet18


num_classes=14 #  分类类别个数
model_path='./models/resnet_18-size-192_epoch-200.pth'
labels=['one','five','fist','ok','heart Single','yearh','three','four','six','i love you','gun','thumb Up','nine','pink']


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


class  Handgesture(Node):

    def __init__(self):
        super().__init__('Handgesture')
        self.get_logger().info('Handgesture')
        self.model_ = resnet18(num_classes=num_classes,img_size=192)
        self.use_cuda = torch.cuda.is_available()
        self.device = torch.device("cuda:0" if self.use_cuda else "cpu")
        self.model_ = self.model_.to(self.device)
        self.model_.eval() # 设置为前向推断模式
        # 加载测试模型
        if os.access(model_path,os.F_OK):# checkpoint
            chkpt = torch.load(model_path, map_location=self.device)
            self.model_.load_state_dict(chkpt)
            print('load test model : {}'.format(model_path))
        self.input_topic_p = self.declare_parameter('input_topic', '/camera/color/image_raw')
        img_topic = self.input_topic_p.value
        self.create_subscription(ImageMsg, img_topic, self.image_detection_callback, 1)
        self.image_pub = self.create_publisher(ImageMsg, 'detections_image', 1)

    def image_detection_callback(self, img_msg):
        image_np = image_msg_to_image_np(img_msg)
        with torch.no_grad():
            img_ = cv2.resize(image_np, (192,192), interpolation = cv2.INTER_CUBIC)
            img_ = img_[:, :, ::-1] #RGB转BGR
            img_ = img_.astype(np.float32)
            img_ = (img_-128.)/256.

            img_ = img_.transpose(2, 0, 1)
            img_ = torch.from_numpy(img_)
            img_ = img_.unsqueeze_(0)

            if self.use_cuda:
                img_ = img_.cuda()  # (bs, 3, h, w)
            pre_ = self.model_(img_.float()) # 模型推理
            outputs = F.softmax(pre_,dim = 1)
            outputs = outputs[0]

            output = outputs.cpu().detach().numpy()
            output = np.array(output)

            max_index = np.argmax(output)

            score_ = output[max_index]
            if score_>0.8:
                cv2.putText(image_np, labels[max_index], (20, 20), 0, 1, [0, 255, 0], lineType=cv2.LINE_AA)
            
            print('pre {}     --->>>    confidence {}'.format(max_index,score_))

        img_msg = image_np_to_image_msg(image_np)
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)

    handgesture_node = Handgesture()

    rclpy.spin(handgesture_node)
    handgesture_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
