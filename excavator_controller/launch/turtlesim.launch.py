from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessStart
# from launch.actions import IncludeLaunchDescription, TimerAction

def generate_launch_description():
    boom_cmd_node = Node(
        package='excavator_controller',
        executable='boom_cmd',
        parameters=[{'target_pose': -0.3}],
    ),
    # arm_cmd_node = Node(
    #     package='excavator_controller',
    #     executable='arm_cmd',
    #     parameters=[{'target_pose': -0.7}],
    # ),
    # # delayed_arm_cmd_node = TimerAction(period=5.0,actions=[arm_cmd_node])

    # bucket_cmd_node = Node(
    #     package='excavator_controller',
    #     executable='bucket_cmd',
    #     parameters=[{'target_pose': 1.2}],
    # ),
    # # delayed_bucket_cmd_node = TimerAction(period=5.0,actions=[bucket_cmd_node])

    return LaunchDescription([
        boom_cmd_node,
        # delayed_arm_cmd_node,
        # # delayed_bucket_cmd_node,
        # arm_cmd_node,
        # bucket_cmd_node,

    ])