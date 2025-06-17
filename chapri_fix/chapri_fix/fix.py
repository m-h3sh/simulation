import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

class PointCloudFrameChanger(Node):
    def __init__(self):
        super().__init__('pointcloud_frame_changer')
        self.declare_parameter('input_topic', '/depth_camera1/points')
        self.declare_parameter('output_topic', '/zed/zed_node/point_cloud/cloud_registered')
        self.declare_parameter('new_frame_id', 'depth_camera_optical_link')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.new_frame_id = self.get_parameter('new_frame_id').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.get_logger().info(f"Subscribed to {input_topic}, publishing to {output_topic} with new frame_id: {self.new_frame_id}")

    def listener_callback(self, msg):
        msg.header.frame_id = self.new_frame_id
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameChanger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

