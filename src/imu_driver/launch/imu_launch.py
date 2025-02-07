from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_driver',
            executable='driver',     
            name='imu_driver_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
            }]
        )
    ])