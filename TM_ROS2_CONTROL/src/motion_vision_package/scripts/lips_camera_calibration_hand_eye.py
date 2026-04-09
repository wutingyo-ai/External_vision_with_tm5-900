#!/usr/bin/env python3

import os
import sys
import cv2
from openni import openni2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CameraSaveOnOkNode(Node):
    def __init__(self):
        super().__init__('camera_save_on_ok_node')

        self.save_dir = '/home/lips/TM_ROS2_CONTROL/src/Motion_vision_package/tag_detect_image'
        os.makedirs(self.save_dir, exist_ok=True)

        self.save_rgb_requested = False

        self.create_subscription(String, 'user_input', self.user_input_callback, 10)
        self.get_logger().info("Node started. Waiting for 'ok' on topic user_input.")

    def user_input_callback(self, msg: String):
        if msg.data.strip().lower() == 'ok':
            self.save_rgb_requested = True
            self.get_logger().info("Received 'ok'. RGB image will be saved on next frame.")

    def consume_save_request(self):
        if self.save_rgb_requested:
            self.save_rgb_requested = False
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    global image_count
    image_count = 0
    node = None
    device = None
    color = None
    depth = None
    try:
        node = CameraSaveOnOkNode()

        openni2.initialize('/home/lips/TM_ROS2_CONTROL/Camera_diver/AE430_AE470/Redist')
        uris = openni2.Device.enumerate_uris()
        if not uris:
            node.get_logger().error('Camera not found')
            raise RuntimeError('Camera not found')

        device = openni2.Device.open_file(uris[0])
        color = device.create_color_stream()
        color.start()  # type: ignore

        depth = device.create_depth_stream()
        depth.start()  # type: ignore

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            rgb_frame = color.read_frame()  # type: ignore
            rgb_mat = np.frombuffer(
                rgb_frame.get_buffer_as_uint8(), dtype=np.uint8
            ).reshape(rgb_frame.height, rgb_frame.width, 3)  # type: ignore
            rgb_mat = cv2.cvtColor(rgb_mat, cv2.COLOR_BGR2RGB)
            cv2.imshow('RGB', rgb_mat)

            depth_frame = depth.read_frame()  # type: ignore
            depth_mat = np.frombuffer(
                depth_frame.get_buffer_as_uint16(), dtype=np.uint16
            ).reshape(depth_frame.height, depth_frame.width, 1)  # type: ignore
            depth_mat = cv2.convertScaleAbs(depth_mat, alpha=255.0 / 1024.0)
            depth_mat = cv2.applyColorMap(depth_mat, cv2.COLORMAP_JET)
            cv2.imshow('Depth', depth_mat)

            if node.consume_save_request():
                rgb_path = os.path.join(node.save_dir, f'{image_count}.png')
                cv2.imwrite(rgb_path, rgb_mat)
                node.get_logger().info(f'Saved RGB image: {rgb_path}')
                image_count += 1

            key = cv2.waitKey(1)
            if key == ord('q'):
                node.get_logger().info('Pressed q, shutting down node.')
                break

        rclpy.shutdown()
    except RuntimeError as exc:
        print(str(exc))
        sys.exit(1)
    finally:
        cv2.destroyAllWindows()
        if color is not None:
            color.stop()  # type: ignore
        if depth is not None:
            depth.stop()  # type: ignore
        if device is not None:
            device.close()
        openni2.unload()

        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()