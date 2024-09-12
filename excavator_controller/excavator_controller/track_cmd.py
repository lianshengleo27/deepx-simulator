import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, '/zx120/track_cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Start moving...")


    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0   ## to be changed
        msg.angular.z = 1.0  ## to be changed
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    rclpy.spin(velocity_publisher)

    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()