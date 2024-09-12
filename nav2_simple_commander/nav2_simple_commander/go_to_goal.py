import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped

# from turtlesim.msg import Pose
import sys
import math

class Turtle_GTG(Node):
    def __init__(self):
        super().__init__("Go_to_Goal_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/zx120/track_cmd_vel', 10)
        self.pose_sub = self.create_subscription(TFMessage, '/zx120/pose', self.pose_callback, 10)
        self.goal_pose_sub = self.create_subscription(Float64MultiArray, '/zx120/goal_pose', self.goal_pose_callback, 10)
        self.pose = TransformStamped()
        self.goal_pose = None
        self.goal_reached = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        self.pose = msg.transforms[0]

    def goal_pose_callback(self, msg):
        self.goal_pose = msg.data
        # once a msg goes to the /zx120/goal_pose topic, extract its data and trigger go_to_goal function
        self.goal_reached = False
        # self.go_to_goal()

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z
    
    def timer_callback(self):
        # if goal.reached = False, loop go to goal function
        if not self.goal_reached:
            self.go_to_goal()
    
    def go_to_goal(self):
        # If goal_pose is not in format of [x, y], exit
        if self.goal_pose is None or len(self.goal_pose) < 2:
            return
        
        new_vel = Twist()
        x = self.pose.transform.translation.x
        y = self.pose.transform.translation.y
        x_rotate = self.pose.transform.rotation.x
        y_rotate = self.pose.transform.rotation.y
        z_rotate = self.pose.transform.rotation.z
        w_rotate = self.pose.transform.rotation.w
        x_goal = self.goal_pose[0]
        y_goal = self.goal_pose[1]
        # x_goal = self.goal_pose
        # y_goal = 2.0
        
        # Ecludian Distance
        distance_to_goal = math.sqrt( (x_goal - x)**2  + (y_goal - y)**2 )
        # Angle to Goal
        angle_to_goal =math.atan2(y_goal - y , x_goal - x)

        distance_tolerance = 1.0
        angle_tolerance = 0.5
        k = 10 #proportional coefficient

        roll, pitch, yaw = self.euler_from_quaternion(x_rotate, y_rotate, z_rotate, w_rotate)
        angle_error = angle_to_goal - yaw
        self.get_logger().info('distance to goal: {}, angle to goal: {}'.format(distance_to_goal,angle_error))

        # if not facing towards the goal, then rotate it first
        if abs(angle_error) > angle_tolerance:
            # new_vel.angular.z = 0.5 if angle_error < 0 else -0.5
            new_vel.angular.z = k * angle_error
        # then move it towards the goal
        else :
            if (distance_to_goal) >= distance_tolerance: #  if not reach the goal (i.e., threshold)
                new_vel.linear.x = k * distance_to_goal # shorter the distance to goal, slower the speed
            else :
                new_vel.linear.x = 0.0
                new_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(new_vel)
                self.get_logger().info("Goal (x: {}, y: {}) Reached".format(x_goal, y_goal))
                self.goal_reached = True
                # self.destroy_node()
                # rclpy.shutdown()
                return

        self.cmd_vel_pub.publish(new_vel)
        # self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Turtle_GTG()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()