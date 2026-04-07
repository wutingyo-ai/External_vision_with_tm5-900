#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tm_msgs.srv import AskItem
import re
import numpy as np
from std_msgs.msg import String
import os

class GetFlangePoseNode(Node):
    def __init__(self):
        super().__init__('demo_ask_item')
        
        # 初始化變量
        self.flange_pose_values_list = None
        self.tool_pose_values_list = None
        self.start_subscribing = False
        self.SAVE_PATH = "/home/lips/TM_ROS2_CONTROL/src/Motion_vision_package/eye_hand_calibration_data"
        
        # 創建字符串消息的訂閱者
        self.string_sub = self.create_subscription(
            String,
            "user_input",
            self.string_callback,
            10
        )
        
        # 創建服務客戶端
        self.client = self.create_client(AskItem, '/ask_item')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # 創建定時器，1Hz
        self.timer = self.create_timer(1.0, self.timer_callback)

    def extract_pose_values(self, coord_string):
        # 使用正則表達式提取花括號內的內容
        match = re.search(r'\{(.*?)\}', coord_string)
        if match:
            # 將提取的內容按逗號分割
            values_str = match.group(1).split(',')
            # 將字符串轉換為浮點數
            values = [float(v.strip()) for v in values_str]
            return values
        else:
            return None

    def string_callback(self, msg):
        if msg.data == "ok":
            self.get_logger().info("Received 'ok', starting to subscribe to pose messages.")
            self.start_subscribing = True
        elif msg.data == "finish":
            self.get_logger().info("Received 'finish', saving pose values to CSV files.")
            self.start_subscribing = False
            # 保存為CSV文件
            if self.flange_pose_values_list is not None:
                np.savetxt(os.path.join(self.SAVE_PATH, 'flange_pose_values.csv'), 
                          self.flange_pose_values_list, delimiter=',', 
                          header='x,y,z,rx,ry,rz', comments='')
                self.get_logger().info("Saved flange_pose_values_list to flange_pose_values.csv")
            if self.tool_pose_values_list is not None:
                np.savetxt(os.path.join(self.SAVE_PATH, 'tool_pose_values.csv'), 
                          self.tool_pose_values_list, delimiter=',', 
                          header='x,y,z,rx,ry,rz', comments='')
                self.get_logger().info("Saved tool_pose_values_list to tool_pose_values.csv")

    def timer_callback(self):
        if self.start_subscribing:
            self.start_subscribing = False
            
            # 查詢 Coord_Robot_Flange
            request = AskItem.Request()
            request.id = "demo"
            request.item = "Coord_Robot_Flange"
            request.wait_time = 0.5
            
            future = self.client.call_async(request)
            future.add_done_callback(self.handle_flange_response)
            
            # 查詢 Coord_Robot_Tool
            request2 = AskItem.Request()
            request2.id = "demo"
            request2.item = "Coord_Robot_Tool"
            request2.wait_time = 0.5
            
            future2 = self.client.call_async(request2)
            future2.add_done_callback(self.handle_tool_response)

    def handle_flange_response(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f"AskItem to robot: item is Coord_Robot_Flange, value is {response.value}")
                coord_string = response.value
                pose_values = self.extract_pose_values(coord_string)
                if pose_values:
                    pose_values = np.asarray(pose_values).reshape(1, -1)
                    if self.flange_pose_values_list is None:
                        self.flange_pose_values_list = pose_values
                    else:
                        self.flange_pose_values_list = np.vstack((self.flange_pose_values_list, pose_values))
                else:
                    self.get_logger().warn("Failed to extract flange pose values")
            else:
                self.get_logger().warn("AskItem to robot, but response not yet ok")
        except Exception as e:
            self.get_logger().error(f"Error AskItem to robot: {e}")

    def handle_tool_response(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info(f"AskItem to robot: item is Coord_Robot_Tool, value is {response.value}")
                coord_string = response.value
                pose_values = self.extract_pose_values(coord_string)
                if pose_values:
                    pose_values = np.asarray(pose_values).reshape(1, -1)
                    if self.tool_pose_values_list is None:
                        self.tool_pose_values_list = pose_values
                    else:
                        self.tool_pose_values_list = np.vstack((self.tool_pose_values_list, pose_values))
                else:
                    self.get_logger().warn("Failed to extract tool pose values")
            else:
                self.get_logger().warn("AskItem to robot, but response not yet ok")
        except Exception as e:
            self.get_logger().error(f"Error AskItem to robot: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = GetFlangePoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()