#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
import struct
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField



class FrameChanger(Node):
    def __init__(self):
        super().__init__('change_frame')
        self.pc_sub = self.create_subscription(PointCloud2, '/depth_camera1/points', self.pc_callback, 10)
        self.correct_pc_pub = self.create_publisher(PointCloud2, '/camera_pc', 10)

    def pc_callback(self, data):
        wrong_pc = []
        # Getting the original pointcloud points
        wrong_pc = point_cloud2.read_points_list(data, field_names=["x", "y", "z"], skip_nans=False)
        header = Header()
        header.frame_id = "base_link"
        header.stamp = self.get_clock().now().to_msg()
        fields = [
                    PointField(name="z", offset=0, datatype=7, count=1),
                    PointField(name="y", offset=4, datatype=7, count=1),
                    PointField(name="x", offset=8, datatype=7, count=1),
                ]
        correct_pc = point_cloud2.create_cloud(header=header, fields=fields, points=wrong_pc)
        self.correct_pc_pub.publish(correct_pc)

def main(args=None):
    rclpy.init(args=args)
    frame_changer = FrameChanger()
    rclpy.spin(frame_changer)
    frame_changer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
