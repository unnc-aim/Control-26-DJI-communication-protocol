"""
裁判系统串口读取节点启动文件

本启动文件用于启动裁判系统串口读取节点，并加载配置参数。

使用方法：
    ros2 launch dji_referee_protocol referee_serial_launch.py

参数：
    - serial_port_normal: 常规链路串口设备路径
    - serial_port_video: 图传链路串口设备路径
    - config_file: 配置文件路径
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    生成启动描述

    Returns:
        LaunchDescription: 启动描述对象
    """
    # ==================== 声明启动参数 ====================

    # 常规链路串口设备路径
    declare_serial_port_normal = DeclareLaunchArgument(
        'serial_port_normal',
        default_value='/dev/ttyUSB0',
        description='常规链路串口设备路径'
    )

    # 图传链路串口设备路径
    declare_serial_port_video = DeclareLaunchArgument(
        'serial_port_video',
        default_value='/dev/ttyUSB1',
        description='图传链路串口设备路径'
    )

    # 配置文件路径
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='话题配置文件路径（空则使用默认配置）'
    )

    # 是否发布所有话题
    declare_publish_all = DeclareLaunchArgument(
        'publish_all_topics',
        default_value='true',
        description='是否发布所有话题（忽略配置文件）'
    )

    # ==================== 创建节点 ====================

    referee_node = Node(
        package='dji_referee_protocol',
        executable='referee_serial_node',
        name='referee_serial_node',
        output='screen',
        parameters=[{
            'serial_port_normal': LaunchConfiguration('serial_port_normal'),
            'serial_port_video': LaunchConfiguration('serial_port_video'),
            'config_file': LaunchConfiguration('config_file'),
            'publish_all_topics': LaunchConfiguration('publish_all_topics'),
        }],
        remappings=[
            # 可在此添加话题重映射
        ]
    )

    # ==================== 返回启动描述 ====================

    return LaunchDescription([
        # 声明参数
        declare_serial_port_normal,
        declare_serial_port_video,
        declare_config_file,
        declare_publish_all,
        # 启动节点
        referee_node,
    ])
