import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class CombinedNode(Node):

    def __init__(self):
        super().__init__('arm_cmd_node')
        # self.declare_parameter('target_pose', 0.0)  # Default value is 0.0
        self.boom_publisher_ = self.create_publisher(Float64, '/zx120/boom/cmd', 10)
        self.arm_publisher_ = self.create_publisher(Float64, '/zx120/arm/cmd', 10)
        self.bucket_publisher_ = self.create_publisher(Float64, '/zx120/bucket/cmd', 10)
        self.rotator_publisher_ = self.create_publisher(Float64, '/zx120/rotator/cmd', 10)
        self.publish_once()

    def publish_once(self):
        msg_boom = Float64()
        msg_arm = Float64()
        msg_bucket = Float64()
        msg_rotator = Float64()

        msg_boom.data = 0.0
        self.boom_publisher_.publish(msg_boom)
        msg_arm.data = 0.0
        self.arm_publisher_.publish(msg_arm)
        msg_bucket.data = 0.0
        self.bucket_publisher_.publish(msg_bucket)
        msg_rotator.data = 0.0
        self.rotator_publisher_.publish(msg_rotator)


        # self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = CombinedNode()
    # rclpy.spin(node)

    # Cleanup when the node is shut down
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()