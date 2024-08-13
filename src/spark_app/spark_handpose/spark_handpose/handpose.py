import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image as ImageMsg

import cv2
import os
import random
import torch
from .rexnetv1 import ReXNetV1


num_classes=42 #  手部21关键点， (x,y)*2 = 42
model_path='./models/ReXNetV1-size-256.pth'


def draw_bd_handpose(img_,hand_,x,y):
    thick = 2
    colors = [(0,215,255),(255,115,55),(5,255,55),(25,15,255),(225,15,55)]
    #
    cv2.line(img_, (int(hand_['0']['x']+x), int(hand_['0']['y']+y)),(int(hand_['1']['x']+x), int(hand_['1']['y']+y)), colors[0], thick)
    cv2.line(img_, (int(hand_['1']['x']+x), int(hand_['1']['y']+y)),(int(hand_['2']['x']+x), int(hand_['2']['y']+y)), colors[0], thick)
    cv2.line(img_, (int(hand_['2']['x']+x), int(hand_['2']['y']+y)),(int(hand_['3']['x']+x), int(hand_['3']['y']+y)), colors[0], thick)
    cv2.line(img_, (int(hand_['3']['x']+x), int(hand_['3']['y']+y)),(int(hand_['4']['x']+x), int(hand_['4']['y']+y)), colors[0], thick)

    cv2.line(img_, (int(hand_['0']['x']+x), int(hand_['0']['y']+y)),(int(hand_['5']['x']+x), int(hand_['5']['y']+y)), colors[1], thick)
    cv2.line(img_, (int(hand_['5']['x']+x), int(hand_['5']['y']+y)),(int(hand_['6']['x']+x), int(hand_['6']['y']+y)), colors[1], thick)
    cv2.line(img_, (int(hand_['6']['x']+x), int(hand_['6']['y']+y)),(int(hand_['7']['x']+x), int(hand_['7']['y']+y)), colors[1], thick)
    cv2.line(img_, (int(hand_['7']['x']+x), int(hand_['7']['y']+y)),(int(hand_['8']['x']+x), int(hand_['8']['y']+y)), colors[1], thick)

    cv2.line(img_, (int(hand_['0']['x']+x), int(hand_['0']['y']+y)),(int(hand_['9']['x']+x), int(hand_['9']['y']+y)), colors[2], thick)
    cv2.line(img_, (int(hand_['9']['x']+x), int(hand_['9']['y']+y)),(int(hand_['10']['x']+x), int(hand_['10']['y']+y)), colors[2], thick)
    cv2.line(img_, (int(hand_['10']['x']+x), int(hand_['10']['y']+y)),(int(hand_['11']['x']+x), int(hand_['11']['y']+y)), colors[2], thick)
    cv2.line(img_, (int(hand_['11']['x']+x), int(hand_['11']['y']+y)),(int(hand_['12']['x']+x), int(hand_['12']['y']+y)), colors[2], thick)

    cv2.line(img_, (int(hand_['0']['x']+x), int(hand_['0']['y']+y)),(int(hand_['13']['x']+x), int(hand_['13']['y']+y)), colors[3], thick)
    cv2.line(img_, (int(hand_['13']['x']+x), int(hand_['13']['y']+y)),(int(hand_['14']['x']+x), int(hand_['14']['y']+y)), colors[3], thick)
    cv2.line(img_, (int(hand_['14']['x']+x), int(hand_['14']['y']+y)),(int(hand_['15']['x']+x), int(hand_['15']['y']+y)), colors[3], thick)
    cv2.line(img_, (int(hand_['15']['x']+x), int(hand_['15']['y']+y)),(int(hand_['16']['x']+x), int(hand_['16']['y']+y)), colors[3], thick)

    cv2.line(img_, (int(hand_['0']['x']+x), int(hand_['0']['y']+y)),(int(hand_['17']['x']+x), int(hand_['17']['y']+y)), colors[4], thick)
    cv2.line(img_, (int(hand_['17']['x']+x), int(hand_['17']['y']+y)),(int(hand_['18']['x']+x), int(hand_['18']['y']+y)), colors[4], thick)
    cv2.line(img_, (int(hand_['18']['x']+x), int(hand_['18']['y']+y)),(int(hand_['19']['x']+x), int(hand_['19']['y']+y)), colors[4], thick)
    cv2.line(img_, (int(hand_['19']['x']+x), int(hand_['19']['y']+y)),(int(hand_['20']['x']+x), int(hand_['20']['y']+y)), colors[4], thick)

def plot_box(bbox, img, color=None, label=None, line_thickness=None):
    tl = line_thickness or round(0.002 * max(img.shape[0:2])) + 1
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl)# 目标的bbox
    if label:
        tf = max(tl - 2, 1)
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 4, thickness=tf)[0] # label size
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3 # 字体的bbox
        cv2.rectangle(img, c1, c2, color, -1)  # label 矩形填充
        # 文本绘制
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 4, [225, 255, 255],thickness=tf, lineType=cv2.LINE_AA)

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


class  Handpose(Node):

    def __init__(self):
        super().__init__('Handpose')
        self.get_logger().info('Handpose')
        self.model_ = ReXNetV1(num_classes=num_classes)
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
            img_ = cv2.resize(image_np, (256,256), interpolation = cv2.INTER_CUBIC)
            img_ = img_[:, :, ::-1] #RGB转BGR
            img_ = img_.astype(np.float32)
            img_ = (img_-128.)/256.

            img_ = img_.transpose(2, 0, 1)
            img_ = torch.from_numpy(img_)
            img_ = img_.unsqueeze_(0)

            if self.use_cuda:
                img_ = img_.cuda()  # (bs, 3, h, w)
            pre_ = self.model_(img_.float()) # 模型推理
            output = pre_.cpu().detach().numpy()
            output = np.squeeze(output)
            x_max = -65535
            y_max = -65535
            x_min = 65535
            y_min = 65535

            pts_hand = {} #构建关键点连线可视化结构
            for i in range(int(output.shape[0]/2)):
                x = (output[i*2+0]*float(img_msg.width))
                y = (output[i*2+1]*float(img_msg.height))

                #------------- 绘制关键点
                cv2.circle(image_np, (int(x),int(y)), 3, (255,50,60),-1)
                cv2.circle(image_np, (int(x),int(y)), 1, (255,150,180),-1)

                pts_hand[str(i)] = {}
                pts_hand[str(i)] = {
                    "x":x,
                    "y":y,
                    }
                x_min = x if x_min>x else x_min
                y_min = y if y_min>y else y_min
                x_max = x if x_max<x else x_max
                y_max = y if y_max<y else y_max
            #gain_x=abs(x_max-x_min)/2
            
            #print("{},{},{},{}".format(x_min,y_min,x_max,y_max))
            plot_box((x_min-15,y_min-15,x_max+15,y_max+15), image_np, color=(255,100,100), label="hand", line_thickness=2)
            draw_bd_handpose(image_np,pts_hand,0,0) # 绘制关键点连线

            #------------- 绘制关键点
            # for i in range(int(output.shape[0]/2)):
            #     x = (output[i*2+0]*float(img_width))
            #     y = (output[i*2+1]*float(img_height))

            #     cv2.circle(image_np, (int(x),int(y)), 3, (255,50,60),-1)
            #     cv2.circle(image_np, (int(x),int(y)), 1, (255,150,180),-1)

        img_msg = image_np_to_image_msg(image_np)
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)

    handpose_node = Handpose()

    rclpy.spin(handpose_node)
    handpose_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
