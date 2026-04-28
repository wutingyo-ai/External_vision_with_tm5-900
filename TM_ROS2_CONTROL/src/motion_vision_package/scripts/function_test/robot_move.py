#!/home/lips/TM_ROS2_CONTROL/venv/bin/python3


import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import rclpy.waitable
from tm_msgs.srv import SendScript,SetIO,WriteItem
from tm_msgs.msg import FeedbackState 
import math
import numpy as np
import time
from std_msgs.msg import Int32
from std_msgs.msg import String 
from std_msgs.msg import Float64
from geometry_msgs.msg import Point #float型態(x y z)
from std_msgs.msg import MultiArrayDimension 
from std_msgs.msg import Float32MultiArray
from typing import Optional
# import ROS2_gripper as rq
# from aces_ws import ROS2_gripper as rq
import threading
# import minimalmodbus
from time import sleep

class Moving_node(Node):
    # 定義一個繼承自 `Node` 的類 `ScriptSender`。

    def __init__(self,name):
        # 初始化方法，類似於 C++ 的構造函數。
        super().__init__(name)
        self.client_script = self.create_client(SendScript, 'send_script')
        self.client_IO = self.create_client(SetIO, 'set_io')
        self.client_write=self.create_client(WriteItem,'write_item')
        self.subscription=self.create_subscription(FeedbackState,'feedback_states',self.Feedback_state_call_back,10 ) # Queue size)
         # 訂閱 "feedback_states" 主題
        # self.subscription = self.create_subscription(FeedbackState,'feedback_states',self.tm_msg_callback,10)
        # 呼叫父類別的初始化方法，並設定節點名稱為 "demo_send_script"。
        

        self.latest_msg: Optional[FeedbackState] = None
        self.IO_state : Optional[FeedbackState] = None
      
        


# -------------------------------------------------------------------------------------------------訂閱部份
    # 初始化訂閱者
        # self.subscription_front_reverse=self.create_subscription(String, 'front_reverse', self.front_reverse_callback, 10)
        # self.subscription_robot_point_xy=self.create_subscription(Point, 'robot_point_xy', self.python_point_callback, 10)
        # self.subscription_Ry_angle=self.create_subscription(Float64, 'Ry_angle', self.Ry_angle_callback, 10)
        # self.subscription_Place_xy=self.create_subscription(Float32MultiArray, 'Place_xy', self.Point_place_callback, 10)
        # self.subscription_refill=self.create_subscription(Float32MultiArray, 'refill', self.refill_position_callback, 10)
        
    # ####################################################
    

    
    # ####################################################第二階段影像與點位訊息
  



    def Feedback_state_call_back(self,msg):
        self.latest_msg=msg
        self.IO_state=msg.cb_digital_output
        # print(msg.cb_digital_output[0])
        # print(self.latest_msg)

    def get_IO(self,pin):
        rclpy.spin_once(self)
        return self.IO_state[pin] # type: ignore

# -------------------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------------------監測部份


    def new_monitor(self,monitor_target_point):
        
        point_offset = 0.02
        arrive = 0

        # 使用變數來存儲接收到的訊息
        # self.latest_msg = None
        # latest_msg=self.latest_msg
        # print(latest_msg)

        """ def callback(msg):
            nonlocal latest_msg
            latest_msg = msg """

        """ # 創建訂閱者並指定回調函式
        subscription = node.create_subscription(
            FeedbackState,
            'feedback_states',
            callback,
            10  # Queue size
        )"""


        try:
            while rclpy.ok():
                # 執行一次訊息處理循環
                rclpy.spin_once(self, timeout_sec=0.1)
                # self.latest_msg = None
                latest_msg=self.latest_msg
                # print(latest_msg)


                if latest_msg is None:
                    self.get_logger().info("NO msg")
                    continue

                # 提取 tool_pose 資訊並轉換為數值列表
                if len(latest_msg.tool_pose) == 6:
                    robot_pose = [
                        latest_msg.tool_pose[0] * 1000,  # x msg unit: m
                        latest_msg.tool_pose[1] * 1000,  # y msg unit: m
                        latest_msg.tool_pose[2] * 1000,  # z msg unit: m
                        math.degrees(latest_msg.tool_pose[3]),  # rx msg unit: rad
                        math.degrees(latest_msg.tool_pose[4]),  # ry msg unit: rad
                        math.degrees(latest_msg.tool_pose[5])   # rz msg unit: rad
                    ]
                else:
                    self.get_logger().error("Invalid tool_pose length")
                    continue

                # 計算 robot_pose 與目標點位的變化程度
                change_of_point = abs((np.array(monitor_target_point[:3]) - np.array(robot_pose[:3])))
                
                # 檢查機器人是否到達目標點
                if all(abs(monitor_target_point[i] - robot_pose[i]) < point_offset for i in range(3)):
                    arrive += 1
                    self.get_logger().info(f"Change of point: {change_of_point[:3]}")
                else:
                    arrive = 0

                # 如果連續達到目標點三次，退出循環
                if arrive >= 3:
                    break

                # 重置 latest_msg 為 None，等待下一條訊息
                latest_msg = None

        finally:
            # self.destroy_subscription(self)
            print('Monitor Finish!')
# -------------------------------------------------------------------------------------------------

# -------------------------------------------------------------------------------------------------發送訊息部份
    def set_IO(self,pin=0,state=1.0,type=SetIO.Request.TYPE_DIGITAL_OUT):
        # 定義一個方法 `send_cmd`，用於發送指令。
        

                # 等待服務可用
        while not self.client_IO.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')  

        request = SetIO.Request()
        request.module = SetIO.Request.MODULE_CONTROLBOX
        request.type = type
        request.pin = pin
        request.state = state  # STATE_ON

        future = self.client_IO.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().ok: # type: ignore
                self.get_logger().info('Service call succeeded: OK')
            else:
                self.get_logger().info('Service call succeeded: not OK')
        else:
            self.get_logger().error('Service call failed')

        # rclpy.shutdown()

    def write_item(self):
        request = WriteItem.Request()
        request.id = "detect"
        request.item = "g_complete_signal"
        request.value = "true"
        while not self.client_write.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service. Exiting.")
                return
            self.get_logger().info("Service not available, waiting again...")
        
        future = self.client_write.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            if future.result().ok: # type: ignore
                self.get_logger().info("OK")
            else:
                self.get_logger().info("Not OK")
        else:
            self.get_logger().error("Failed to call service")    

    def move_send_script(self,*args,motion_type='Line', coordinate_type='CPP', speed=30, time=200, trajectory_percent=0, precision_arrive=False):
        # 定義一個方法 `send_cmd`，用於發送指令。
        

                # 等待服務可用
        while not self.client_script.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        # 如果是列表則解包
        
            
        if len(args) == 1 and isinstance(args[0], list) and len(args[0]) == 6:
            x, y, z, rx, ry, rz = args[0]
        elif len(args) == 6:
            # 否則將每個單一值賦給 x, y, z, rx, ry, rz
            x, y, z, rx, ry, rz = args
        else:
            raise ValueError("Invalid arguments: Please provide either a list of six values or six separate values for position.")

        # 等待服務 'tm_driver/send_script' 可用
        # rospy.wait_for_service('tm_driver/send_script')
        
        # 格式化並生成所需的字符串
        command = f'{motion_type}("{coordinate_type}",{x},{y},{z},{rx},{ry},{rz},{speed},{time},{trajectory_percent},{precision_arrive})'
        request = SendScript.Request()

        
        # 創建一個新的服務請求物件。


        request.id = 'demo'
        # 設定請求的 `id` 欄位為 "demo"。

        request.script = command
        # 將指令字串 `cmd` 賦值給請求的 `script` 欄位。
        

        future = self.client_script.call_async(request)
        # 發送非同步請求，並返回一個 future 對象。
        
        rclpy.spin_until_future_complete(self, future)
        # 等待請求完成。

        if future.result() is not None:
            # 如果請求成功完成，檢查結果。

            if future.result().ok: # type: ignore
                self.get_logger().info('OK')
                # 如果 `ok` 為 true，記錄成功信息。

            else:
                self.get_logger().info('Not OK')
                # 如果 `ok` 為 false，記錄失敗信息。

        else:
            self.get_logger().error('Service call failed')
            # 如果請求失敗，記錄錯誤信息。



    def leave_listen_node(self):
        # rclpy.init()  # 初始化rclpy
        # node = rclpy.create_node('leave_listen_node')  # 建立一個節點

        # 等待服務 'tm_driver/send_script' 可用
        self.get_logger().info('Waiting for service "Exit listen node"...')
        

        if not self.client_script.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service "Exit listen node" not available!')
            return

        # 創建服務請求
        request = SendScript.Request()
        request.id = "demo"  # 設定 ID
        request.script = "ScriptExit()"  # 設定指令為 ScriptExit()

        # 發送請求並等待回應
        future = self.client_script.call_async(request)

        # 等待回應
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Service call successful: {future.result()}')
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')

        # node.destroy_node()  # 銷毀節點
        # rclpy.shutdown()  # 關閉 rclpy


    def change_tcp(self,your_script:str="ChangeTCP(\"NOTOOL\")")->None:
        self.get_logger().info('Waiting for service "Exit listen node"...')
        

        if not self.client_script.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service "Exit listen node" not available!')
            return

        # 創建服務請求
        request = SendScript.Request()
        request.id = "demo"  # 設定 ID
        request.script = your_script # 設定指令

        # 發送請求並等待回應
        future = self.client_script.call_async(request)

        # 等待回應
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Service call successful: {future.result()}')
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')

    """ def gripper_pick(self,pick_distance:int,inital_bool:bool=True)->None:
        instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 9)
        instrument.debug=False
        instrument.serial.baudrate = 115200
        myGripper = rq.RobotiqGripper(portname='/dev/ttyUSB0',slaveaddress=9)

        if not inital_bool:
            myGripper.resetActivate()
            myGripper.calibrate(0, 255)
        myGripper.goTo(pick_distance, 90, 255)   #行程 速度 力量.  """





def main(args=None):

    
############################初始化##############################
    # 初始化 rclpy
    rclpy.init(args=args)

    # 創建節點物件
    node = Moving_node('move_to_fixture_node')
    node.change_tcp()
    print("change tcp to NOTOOL")
    First_point = [644.78, -274.35, 475.04, 135.35, 13.02, 39.10]
    node.move_send_script(First_point)
    node.new_monitor(First_point)
    print("arrive first point")
   




    exit()    
  

if __name__ == '__main__':
    main()

