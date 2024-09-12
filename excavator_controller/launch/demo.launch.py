from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    boom_cmd_node = Node(
        package='excavator_controller',
        executable='boom_cmd',
        parameters=[{'target_pose': -0.3}],
    ),
    arm_cmd_node = Node(
        package='excavator_controller',
        executable='arm_cmd',
        parameters=[{'target_pose': -0.7}],
    ),

    # bucket_cmd_node = Node(
    #     package='excavator_controller',
    #     executable='bucket_cmd',
    #     parameters=[{'target_pose': 1.2}],
    # ),

    return LaunchDescription([
        boom_cmd_node,
        arm_cmd_node,
        # bucket_cmd_node,

    ])