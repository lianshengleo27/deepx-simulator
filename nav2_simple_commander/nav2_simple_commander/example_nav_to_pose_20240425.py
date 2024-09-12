#####################################################################
# SINGLE TASK --RUN FORWARD--
#####################################################################

from typing import List
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float32

from geometry_msgs.msg import Pose2D
from nav_2d_msgs.msg import Path2D, Pose2DStamped
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

import time
import copy, os
class sc_dump(Node):
    def __init__(self):
        super().__init__('sc_dump')
        # ID
        # ID = os.environ['ROS_DOMAIN_ID']
        topic_ns_du = '/zx120'

        self.ex_fin_flag = False
        self.goal_flag = Bool()
        self.msg_path = Path2D() # 経路：[x,y]->pub_path
        self.msg_vel = Twist()
        self.task_state = 0
        
        # Pub
        self.pub_enb = self.create_publisher(Bool, topic_ns_du+'/cmd/pursuit/enable', 10)
        self.pub_cwp = self.create_publisher(Empty, topic_ns_du+'/cmd/pursuit/clear_path', 10)
        self.pub_path = self.create_publisher(Path2D, topic_ns_du+'/cmd/pursuit/path', 10)
        self.pub_vel = self.create_publisher(Twist, topic_ns_du+'/track_cmd_vel', 10)
        # self.pub_vessel = self.create_publisher(Float32, topic_ns_du+'/cmd/vessel/ang', 10)
        # self.pub_goal = self.create_publisher(Bool, topic_ns_du+'status/pursuit/goal_flag', 1)

        # # Sub
        # self.sub_ex_fin = self.create_subscription(Bool, '/ex_fin', self.ex_fin_callback, 10)
        # self.sub_pose = self.create_subscription(TFMessage, topic_ns_du+'/pose', self.pose_callback, 10)
        # self.sub_goal = self.create_subscription(Bool, topic_ns_du+'/status/pursuit/goal_flag', self.goal_callback, 1)

        # Timer
        timer_priod = 1.0
        self.timer = self.create_timer(timer_priod, self.timer_callback)


    

    # def pose_callback(self,msg):
    #     self.pose_now = [0.0, 0.0, 0.0]
    #     self.pose_now[0] = msg.transforms.transform.translation.x #subscribeしたメッセージのうちxの値を表示
    #     self.pose_now[1] = msg.transforms.transform.translation.y #subscribeしたメッセージのうちyの値を表示
    #     self.pose_now[2] = msg.transforms.transform.rotation.w #subscribeしたメッセージのうちthetaの値を表示
    #     print('現在値 : ',self.pose_now)

    # def ex_fin_callback(self,msg): # バックホウ掘削完了通知
    #     self.ex_fin_flag = msg.data
    #     self.get_logger().info(f'/ex_fin callback: {self.ex_fin_flag}')

    # def goal_callback(self,msg):
    #     self.goal_flag = msg.data
    #     print('########## : ',self.goal_flag)

    # def vessel_up(self):
    #     print('### up! ###')
    #     msg_vessel = 90.0
    #     self.pub_vessel.publish(Float32(data=msg_vessel))
    #     time.sleep(3.0)

    # def vessel_down(self):
    #     print('### down! ###')
    #     msg_vessel = 0.0
    #     self.pub_vessel.publish(Float32(data=msg_vessel))
    #     time.sleep(3.0)
    
    def clear_run(self):
        self.msg_vel.linear.x = 0.0
        self.msg_vel.angular.z = 0.0
        self.pub_vel.publish(self.msg_vel)
        msg_cwp = Empty()
        self.pub_cwp.publish(msg_cwp)
        self.msg_path.poses.clear()

    def run(self,path_tmp,msg_vel):
        pose = Pose2D()
        msg_enb = True # 経路追従オン->pub_enb
        self.pub_enb.publish(Bool(data=msg_enb))
        for pose_tmp in path_tmp:
            pose.x = pose_tmp[0]
            pose.y = pose_tmp[1]
            self.msg_path.poses.append(copy.deepcopy(pose))
        self.pub_path.publish(self.msg_path)
        self.pub_vel.publish(msg_vel)
        # time.sleep(5.0)

    def timer_callback(self):
        print(self.task_state)
        if self.task_state == 0: # 走行開始指示
            # if self.ex_fin_flag == True:
            path_tmp = [[-10.00, 10.60],
                        [-10.00, -10.60],
                        [10.75, -10.60],
                        [10.75, 10.60],
                        [10.00, 10.60]]
            self.msg_vel.linear.x = 0.2
            self.msg_vel.angular.z = 0.2

            self.run(path_tmp,self.msg_vel)
            goal = False
            print(goal)
            self.task_state += 1

        elif self.task_state == 1: # 走行中（goal_flagの監視）
            goal = self.goal_flag
            if goal == True:
                print('goal!')
                self.clear_run()
                self.task_state += 1

        # elif self.task_state == 2: # 走行開始指示
        #     if self.ex_fin_flag == True:
        #         path_tmp = [[0.90, 0.60]]
        #         msg_vel = -0.1
        #         self.run(path_tmp,msg_vel)
        #         goal = False
        #         print(goal)
        #         self.run_now = True
        #         self.task_state += 1
 
        # elif self.task_state == 3: # 走行中（goal_flagの監視）
        #     goal = self.goal_flag
        #     if goal == True:
        #         print('goal!')
        #         self.clear_run()
        #         self.run_now = False
        #         self.task_state += 1

        # elif self.task_state == 4: # ベッセル上昇
        #     self.vessel_up()
        #     self.task_state += 1

        # elif self.task_state == 5: # ベッセル下降
        #     self.vessel_down()
        #     self.task_state += 1

        
            


def main(args=None):
    rclpy.init(args=args)
    node = sc_dump()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # msg_vel.data = 0.0
        # pub_vel.publish(msg_vel)
        pass

if __name__ == '__main__':
    main()