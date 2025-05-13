from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crawling_robot_controller',
            executable='position_control',
            name='position_control_node',
            output='screen'
        ),
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'ros2', 'run', 'keyboard', 'keyboard_teleop'],
                    shell=True
                )
            ]
        )
    ])
