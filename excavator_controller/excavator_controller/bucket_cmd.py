import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class BucketCmdNode(Node):

    def __init__(self):
        super().__init__('bucket_cmd_node')
        self.declare_parameter('target_pose', 0.0)  # Default value is 0.0
        self.publisher_ = self.create_publisher(Float64, '/zx120/bucket/cmd', 10)
        self.publish_once()

    def publish_once(self):
        msg = Float64()
        # msg.data = 1.2  ## -0.7 ~ 1.29
        msg.data = self.get_parameter('target_pose').get_parameter_value().double_value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = BucketCmdNode()
    rclpy.spin(node)

    # Cleanup when the node is shut down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()