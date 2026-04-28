#!/usr/bin/env python3
# by LIPS KevinChang @ 2024-06-17
"""
這隻程式的目的是將 OpenNI2 的 RGB 和 Depth 資料轉換成 ROS2 的 Image 和 CameraInfo 訊息，並發布到對應的 topic 上。
同時也會發布一個靜態的 TF 來描述相機在機器人座標系中的位置。
使用方式：
1. 確保已經安裝好 OpenNI2 的 Python 綁定，並且相機驅動正常。
2. 執行這個腳本，並且可以透過命令行參數來調整相機的 TF 位置和 OpenNI 的 Redist 路徑。
example:python3 openni_to_ros2.py --tx 0.4 --ty 0.05 --tz 1.05 --qx 0.7071 --qy -0.7071 --qz 0.0 --qw 0.0 --redist /path/to/OpenNI/Redist
3. 在 ROS2 中訂閱 'camera/rgb/image_raw'、'camera/depth/image_raw'、'camera/rgb/camera_info' 和 'camera/depth
"""

import sys
import argparse
import numpy as np
import cv2
import json, os
from openni import openni2, _openni2 as c_api

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class OpenNI2RosWorker(Node):
    def __init__(self, args):
        super().__init__('openni2_standalone_node')
        self.bridge = CvBridge()
        
        # 1. 讀取網路配置 (模擬你提到的 network.json 讀取)
        self.load_network_config()

        # 2. 定義相容性高的 QoS (解決 RELIABILITY 不匹配問題)
        # 如果訂閱端是 Reliable，發布端也必須是 Reliable
        self.qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # 3. 設定希望的解析度與 FPS
        self.target_w = 640
        self.target_h = 480
        self.target_fps = 15

        self.fx = 608.92333984375
        self.fy = 608.6539916992188
        self.cx = 315.65606689453125
        self.cy = 239.46075439453125
        # k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        # p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # 4. 初始化發布者
        self.rgb_pub = self.create_publisher(Image, 'camera/rgb/image_raw', self.qos)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', self.qos)
        self.rgb_info_pub = self.create_publisher(CameraInfo, 'camera/rgb/camera_info', self.qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, 'camera/depth/camera_info', self.qos)
        
        # 5. 發布靜態 TF (維持原邏輯)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.args = args
        self.publish_static_tf()

        # 6. 初始化 OpenNI
        self.init_openni(args.redist)

        # 7. 預備 CameraInfo (這裡通常填寫相機標定後的數值)
        self.camera_info_msg = self.generate_dummy_camera_info(640, 480)
        print(f"CameraInfo: {self.camera_info_msg}")

        self.timer = self.create_timer(1.0 / 30.0, self.stream_callback)
        self.get_logger().info("OpenNI2 Standalone Node with CameraInfo Started.")

    def load_network_config(self):
        if os.path.exists('network.json'):
            with open('network.json', 'r') as f:
                config = json.load(f)
                self.get_logger().info(f"Loaded config: {config}")

    def generate_dummy_camera_info(self, width, height):
        """
        產生基礎相機內參。
        注意：為了精準的 Isaac ROS 運算，建議填入實際標定值 (fx, fy, cx, cy)。
        """
        info = CameraInfo()
        info.header.frame_id = "openni_rgb_optical_frame"
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = self.fx
        fy = self.fy
        cx = self.cx
        cy = self.cy
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def publish_static_tf(self):
        """
        發布靜態 TF 來描述相機在機器人座標系中的位置。
        可調整frame_id和child_frame_id以符合實際使用情況。
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "openni_rgb_optical_frame"
        t.transform.translation.x = self.args.tx
        t.transform.translation.y = self.args.ty
        t.transform.translation.z = self.args.tz
        t.transform.rotation.x = self.args.qx
        t.transform.rotation.y = self.args.qy
        t.transform.rotation.z = self.args.qz
        t.transform.rotation.w = self.args.qw
        self.tf_static_broadcaster.sendTransform(t)

    def init_openni(self, redist_path):
        try:
            openni2.initialize(redist_path)
            self.dev = openni2.Device.open_any()
            
            # --- 配置 Color Stream ---
            self.color_stream = self.dev.create_color_stream()
            color_mode = c_api.OniVideoMode(
                pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888,
                resolutionX=self.target_w, 
                resolutionY=self.target_h, 
                fps=self.target_fps
            )
            self.color_stream.set_video_mode(color_mode)
            
            # --- 配置 Depth Stream ---
            self.depth_stream = self.dev.create_depth_stream()
            depth_mode = c_api.OniVideoMode(
                pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,
                resolutionX=self.target_w, 
                resolutionY=self.target_h, 
                fps=self.target_fps
            )
            self.depth_stream.set_video_mode(depth_mode)
            
            # 同步設定 (非常重要，否則 RGB 跟 Depth 會有時間差)
            self.dev.set_depth_color_sync_enabled(True)
            self.dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
            
            self.color_stream.start()
            self.depth_stream.start()
            
            self.get_logger().info(f"Camera started at {self.target_w}x{self.target_h} @ {self.target_fps}fps")
        except Exception as e:
            self.get_logger().error(f"OpenNI set_video_mode failed: {e}")

    def stream_callback(self):
        try:
            d_frame = self.depth_stream.read_frame()
            c_frame = self.color_stream.read_frame()
            
            d_data = np.frombuffer(d_frame.get_buffer_as_uint16(), dtype=np.uint16).reshape(480, 640)
            c_data = np.frombuffer(c_frame.get_buffer_as_uint8(), dtype=np.uint8).reshape(480, 640, 3)
            # c_data = cv2.cvtColor(c_data, cv2.COLOR_RGB2BGR)

            now = self.get_clock().now().to_msg()
            
            # 更新 Info Header
            self.camera_info_msg.header.stamp = now

            # 發布影像
            img_msg = self.bridge.cv2_to_imgmsg(c_data, "rgb8")
            img_msg.header.stamp = now
            img_msg.header.frame_id = "openni_rgb_optical_frame"
            
            depth_msg = self.bridge.cv2_to_imgmsg(d_data, "16UC1")
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "openni_rgb_optical_frame"

            self.rgb_pub.publish(img_msg)
            self.depth_pub.publish(depth_msg)
            self.rgb_info_pub.publish(self.camera_info_msg)
            self.depth_info_pub.publish(self.camera_info_msg)

        except Exception as e:
            pass

def main():
    """
    Main function to start the OpenNI to ROS2 bridge.
    這裡使用 argparse 來處理命令行參數，允許用戶在啟動節點時指定相機的 TF 參數和 OpenNI 的 Redist 路徑。
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--tx', type=float, default=0.25)
    parser.add_argument('--ty', type=float, default=0.0)
    parser.add_argument('--tz', type=float, default=1.05)
    parser.add_argument('--qx', type=float, default=0.7071)
    parser.add_argument('--qy', type=float, default=-0.7071)
    parser.add_argument('--qz', type=float, default=0.0)
    parser.add_argument('--qw', type=float, default=0.0)
    parser.add_argument('--redist', type=str, default="/home/lips/Isaac_Sim_Robot/Camera_diver/AE430_AE470/Redist")
    
    # 處理 ROS2 可能傳入的額外參數
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = OpenNI2RosWorker(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        openni2.unload()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()