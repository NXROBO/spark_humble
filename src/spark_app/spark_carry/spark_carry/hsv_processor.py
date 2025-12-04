#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
import math
from geometry_msgs.msg import PointStamped
import sys
import os
import yaml

class HSVProcessor(Node):
    def __init__(self):
        super().__init__('hsv_processor')
        
        # 初始化CvBridge用于ROS和OpenCV图像转换[5](@ref)
        self.bridge = CvBridge()
        
        # 声明参数
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_node_name', '/camera') 
        
        # 获取参数值
        image_topic = self.get_parameter('image_topic').value
        self.camera_node_name = self.get_parameter('camera_node_name').value
        
        # 创建订阅者
        self.subscription = self.create_subscription(Image,image_topic,self.image_callback,10)
        
        # 创建目标中心点信息发布者(数组)
        self.pub_target_center = self.create_publisher(PointStamped, 'target_center', 0)

        
        # 声明曝光参数（用于内部跟踪）
        self.declare_parameter('exposure', 200)
        self.exposure_value = self.get_parameter('exposure').value

        # 设置参数变更回调
        self.add_on_set_parameters_callback(self.parameters_callback)

        # 加载HSV配置文件
        try:
            pt_path = sys.path[0]
            # 输出当前路径
            self.get_logger().info(f'当前路径: {pt_path}')
            self.config_path = os.path.join(pt_path, 'resource/hsv_config.yaml')
            # 判断是否存在配置文件，则创建文件并写入默认HSV值
            if not os.path.exists(self.config_path) or os.stat(self.config_path).st_size == 0:

                self.get_logger().warning('将创建默认配置文件')
                # 创建默认配置文件
                default_config = {
                    'h_low': 0,
                    's_low': 0,
                    'v_low': 0,
                    'h_high': 179,
                    's_high': 255,
                    'v_high': 255,
                }
                
                with open(self.config_path, 'w') as file:
                    yaml.dump(default_config, file)
                self.get_logger().info(f'已创建默认HSV配置文件: {self.config_path}')
            
            
            self.get_logger().info(f'已加载HSV配置文件: {self.config_path}')
            # 如果存在配置文件，加载参数
            with open(self.config_path, 'r') as file:
                config = yaml.safe_load(file)
                self.h_low = config['h_low']
                self.s_low = config['s_low']
                self.v_low = config['v_low']
                self.h_high = config['h_high']
                self.s_high = config['s_high']
                self.v_high = config['v_high']
        except Exception as e:
            # 输出报错原因,从try中捕获异常
            self.get_logger().error(f'无法加载HSV配置文件: {e}')
            return
        
        global xc, yc, xc_prev, yc_prev, found_count
        xc = 0
        yc = 0
        xc_prev = xc
        yc_prev = yc
        found_count = 0

        # 声明一个布尔参数来控制是否发送中心位置，默认为True（发送）
        self.publish_center = False

        # 定义按钮的视觉属性（位置、大小、颜色）
        self.button_rect = (100, 50, 150, 50)  # (x, y, width, height)
        self.button_color = (100, 200, 100) if self.publish_center else (100, 100, 100) # 根据状态改变颜色
        self.button_text = "SEND: ON" if self.publish_center else "SEND: OFF"

        # 定义保存HSV按钮的视觉属性（位置、大小、颜色）
        self.save_button_rect = (250, 50, 150, 50)  
        self.save_button_color = (200, 100, 0) # 按钮颜色为橙色
        self.save_button_text = "SAVE HSV"
     
        # 初始化OpenCV窗口和滑动条
        self.init_opencv_window()
        
        self.get_logger().info('HSV处理器节点已启动')
    
    def init_opencv_window(self):
        """初始化OpenCV窗口和滑动条[5](@ref)"""
        self.init_window = True
        # 创建窗口
        cv2.namedWindow('HSV Processing', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Origin Image', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Controls', cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow('HSV Processing', 1400, 1000)
        cv2.resizeWindow('Origin Image', 720, 480)
        cv2.resizeWindow('Controls', 1400, 1000)
        cv2.moveWindow('HSV Processing', 100, 100) 
        cv2.moveWindow('Controls', 1400, 10) 
        cv2.moveWindow('Origin Image', 1400, 1000) 
        
        # 创建HSV下限滑动条
        cv2.createTrackbar('H Low', 'Controls', self.h_low, 179, self.on_trackbar,)
        cv2.createTrackbar('S Low', 'Controls', self.s_low, 255, self.on_trackbar)
        cv2.createTrackbar('V Low', 'Controls', self.v_low, 255, self.on_trackbar)
        
        # 创建HSV上限滑动条
        cv2.createTrackbar('H High', 'Controls', self.h_high, 179, self.on_trackbar)
        cv2.createTrackbar('S High', 'Controls', self.s_high, 255, self.on_trackbar)
        cv2.createTrackbar('V High', 'Controls', self.v_high, 255, self.on_trackbar)
        
        # 创建曝光控制滑动条
        cv2.createTrackbar('Exposure', 'HSV Processing', self.exposure_value, 1000, self.on_exposure_trackbar)
        
        self.get_logger().info('OpenCV窗口和滑动条初始化完成')
        self.init_window = False
    
    def on_trackbar(self, val):
        """滑动条回调函数，更新HSV值"""
        if self.init_window:
            return
        # 从滑动条获取当前值
        self.h_low = cv2.getTrackbarPos('H Low', 'Controls')
        self.s_low = cv2.getTrackbarPos('S Low', 'Controls')
        self.v_low = cv2.getTrackbarPos('V Low', 'Controls')
        self.h_high = cv2.getTrackbarPos('H High', 'Controls')
        self.s_high = cv2.getTrackbarPos('S High', 'Controls')
        self.v_high = cv2.getTrackbarPos('V High', 'Controls')
    
    def parameters_callback(self, params):
        """参数变更回调函数"""
        for param in params:
            if param.name == 'exposure':
                self.exposure_value = param.value
                # 更新滑动条位置
                cv2.setTrackbarPos('Exposure', 'HSV Processing', self.exposure_value)
                self.get_logger().info(f'曝光参数已更新: {self.exposure_value}')
                
                # 设置相机曝光  
                self.set_camera_exposure()
        
        return rclpy.node.SetParametersResult(successful=True)


    def set_camera_exposure(self):
        """设置相机曝光值"""
        try:
            # 使用ros2 param set命令设置相机节点的曝光参数
            # 这里需要根据实际相机驱动进行调整
            import subprocess
            command = f"ros2 param set {self.camera_node_name} rgb_camera.exposure {self.exposure_value}"
            subprocess.run(command, shell=True, check=True)
            self.get_logger().info(f'相机曝光已设置为: {self.exposure_value}')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'设置相机曝光失败: {e}')
        except Exception as e:
            self.get_logger().error(f'设置相机曝光时发生错误: {e}')


    def on_exposure_trackbar(self, val):
        """曝光滑动条回调函数 - 修改为设置参数"""
        exposure_value = cv2.getTrackbarPos('Exposure', 'HSV Processing')
        
        # 设置节点参数，这会触发parameters_callback
        param = rclpy.parameter.Parameter(
            'exposure',
            rclpy.Parameter.Type.INTEGER,
            exposure_value
        )
        self.set_parameters([param])

    def save_hsv_values(self):
        """保存当前HSV值到配置文件"""
        # 构建配置内容
        config_content = f"""
        h_low: {self.h_low}
        s_low: {self.s_low}
        v_low: {self.v_low}
        h_high: {self.h_high}
        s_high: {self.s_high}
        v_high: {self.v_high}
        """
        
        # 写入文件
        try:
            with open(self.config_path, 'w') as f:
                f.write(config_content)
            self.get_logger().info(f'HSV值已保存到 {self.config_path}')
        except Exception as e:
            self.get_logger().error(f'保存HSV值到文件时出错: {e}')

    def mouse_callback(self, event, x, y, flags, param):
        global ix, iy, drawing
        # img_copy = img.copy()  # 创建图像副本用于实时绘制预览

        if event == cv2.EVENT_LBUTTONDOWN:
            drawing = True
            ix, iy = x, y  # 记录矩形框的起始点
            print(f"区域起点: ({ix}, {iy})")

        elif event == cv2.EVENT_LBUTTONUP:
            drawing = False
            print(f"区域终点: ({x}, {y})")
            if self.publish_center:
                # 修改消息创建和发布
                target_center = PointStamped()
                # 现在可以设置header了
                target_center.header.stamp = self.get_clock().now().to_msg()
                # 设置坐标
                target_center.point.x = float(x)
                target_center.point.y = float(y)
                target_center.point.z = 0.0  # 2D点可以设置z=0
                # 发布位置信息
                self.pub_target_center.publish(target_center)
    
    def control_mouse_callback(self, event, x, y, flags, param):
        # 检查事件是否为鼠标左键按下，且发生在'Controls'窗口
        if event == cv2.EVENT_LBUTTONDOWN:
            # 获取按钮的坐标和尺寸
            btn_x, btn_y, btn_w, btn_h = self.button_rect
            # 判断点击位置是否在按钮区域内
            if btn_x <= x <= btn_x + btn_w and btn_y <= y <= btn_y + btn_h:
                # 切换发布状态
                self.publish_center = not self.publish_center
                
                # 更新按钮的显示文本和颜色
                self.button_text = "SEND: ON" if self.publish_center else "SEND: OFF"
                self.button_color = (100, 200, 100) if self.publish_center else (100, 100, 100)
                
                # 通过ROS2参数服务器设置新值，这会触发参数回调函数  
                self.get_logger().info(f'发布中心点状态已切换为: {self.publish_center}')

            # 检查点击是否在保存按钮区域
            save_btn_x, save_btn_y, save_btn_w, save_btn_h = self.save_button_rect
            if save_btn_x <= x <= save_btn_x + save_btn_w and save_btn_y <= y <= save_btn_y + save_btn_h:
                # 调用保存函数
                self.save_hsv_values()
                self.get_logger().info('HSV值已保存')


    
    def image_callback(self, msg):
        """图像回调函数[1,4](@ref)"""
        global xc, yc, xc_prev, yc_prev, found_count

        try:
            # 将ROS图像消息转换为OpenCV格式[5](@ref)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 转换为HSV颜色空间[5](@ref)
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 应用HSV阈值处理[1,2](@ref)
            lower_bound = (self.h_low, self.s_low, self.v_low)
            upper_bound = (self.h_high, self.s_high, self.v_high)
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            hsv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            gray_image = hsv_image[:, :, 0]

            def smooth(object_image):
                blurred = cv2.blur(object_image, (9, 9))
                (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
                object_image = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
                object_image = cv2.erode(object_image, None, iterations=4)
                result_image = cv2.dilate(object_image, None, iterations=4)

                return result_image

            hsv_image = smooth(hsv_image)
            gray_image = smooth(gray_image)


            # 边缘识别、中心点识别
            contours, hier = cv2.findContours(gray_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                size = []
                size_max = 0
                for i, c in enumerate(contours):
                    rect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                    y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                    w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                    h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2) 
                    size.append(w * h)
                    xc_prev = x_mid
                    yc_prev = y_mid
                    if size[i] > size_max:
                        size_max = size[i]
                        index = i
                        xc = x_mid
                        yc = y_mid
                        # 绘制当前中心点
                        cv2.circle(hsv_image, (int(xc), int(yc)), 5, (0, 0, 255), -1)
                        cv2.putText(hsv_image, f'({int(xc)}, {int(yc)})', (int(xc)+10, int(yc)-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                # if box is not moving for 20 times
                if found_count >= 20 and self.publish_center:
                    # 修改图像回调中的消息创建和发布
                    target_center = PointStamped()
                    # 设置header
                    target_center.header.stamp = self.get_clock().now().to_msg()
                    # 设置坐标
                    target_center.point.x = float(xc)
                    target_center.point.y = float(yc)
                    target_center.point.z = 0.0  # 2D点可以设置z=0
                    # 发布位置信息
                    self.pub_target_center.publish(target_center)
                    found_count = 0
                    
                else:
                    # if box is not moving
                    if abs(xc - xc_prev) <= 2 and abs(yc - yc_prev) <= 2:
                        found_count = found_count + 1
                        # 输出found_count
                        # self.get_logger().info(f'found_count: {found_count}')
                    else:
                        found_count = 0
            else:
                found_count = 0


            # 绘制按钮
            btn_x, btn_y, btn_w, btn_h = self.button_rect
            controls_img = np.ones((150, 500, 3), dtype=np.uint8) * 255   
            cv2.rectangle(controls_img, (btn_x, btn_y), (btn_x + btn_w, btn_y + btn_h), self.button_color, -1)  # -1表示填充矩形
            # 添加按钮文字
            text_size = cv2.getTextSize(self.button_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            text_x = btn_x + (btn_w - text_size[0]) // 2
            text_y = btn_y + (btn_h + text_size[1]) // 2
            cv2.putText(controls_img, self.button_text, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            # 绘制保存按钮
            save_btn_x, save_btn_y, save_btn_w, save_btn_h = self.save_button_rect
            cv2.rectangle(controls_img, (save_btn_x, save_btn_y), (save_btn_x + save_btn_w, save_btn_y + save_btn_h), self.save_button_color, -1)  # -1表示填充矩形
            # 添加保存按钮文字
            save_text_size = cv2.getTextSize(self.save_button_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            save_text_x = save_btn_x + (save_btn_w - save_text_size[0]) // 2
            save_text_y = save_btn_y + (save_btn_h + save_text_size[1]) // 2
            cv2.putText(controls_img, self.save_button_text, (save_text_x, save_text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)

            # 添加文本显示当前HSV范围
            cv2.putText(hsv_image, f'H: [{self.h_low}-{self.h_high}]', (10, 30), 
                       cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 1)
            cv2.putText(hsv_image, f'S: [{self.s_low}-{self.s_high}]', (10, 60), 
                       cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 1)
            cv2.putText(hsv_image, f'V: [{self.v_low}-{self.v_high}]', (10, 90), 
                       cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 1)
            cv2.putText(hsv_image, f'Exposure: {self.exposure_value}', (10, 120), 
                       cv2.FONT_HERSHEY_DUPLEX, 0.8, (0, 255, 0), 1)
            
            # 显示图像
            cv2.imshow('HSV Processing', hsv_image)
            cv2.setMouseCallback('HSV Processing', self.mouse_callback)
            cv2.imshow('Origin Image',cv_image)
            cv2.setMouseCallback('Origin Image', self.mouse_callback)
            cv2.imshow('Controls',controls_img)
            cv2.setMouseCallback('Controls', self.control_mouse_callback)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
    
    def destroy_node(self):
        """节点销毁时关闭OpenCV窗口"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        hsv_processor = HSVProcessor()
        rclpy.spin(hsv_processor)
    except KeyboardInterrupt:
        pass
    finally:
        hsv_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()