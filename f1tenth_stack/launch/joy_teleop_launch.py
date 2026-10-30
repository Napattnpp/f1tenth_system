import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_stack = get_package_share_directory('f1tenth_stack')
    default_joy_config = os.path.join(pkg_stack, 'config', 'joy_teleop.yaml')

    # Prioritize active racecar profile config from race_bringup if available
    try:
        pkg_bringup = get_package_share_directory('race_bringup')
        racecar_version = os.environ.get('RACECAR_VERSION', 'ORIN1')
        bringup_joy_config = os.path.join(pkg_bringup, 'config', racecar_version, 'joy_teleop.yaml')
        if os.path.exists(bringup_joy_config):
            default_joy_config = bringup_joy_config
    except Exception:
        pass

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=default_joy_config,
        description='Path to joy and joy_teleop configuration YAML'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[LaunchConfiguration('joy_config')]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        parameters=[LaunchConfiguration('joy_config')]
    )

    return LaunchDescription([
        joy_la,
        joy_node,
        joy_teleop_node
    ])
