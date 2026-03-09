"""
DJI机甲大师裁判系统通信协议ROS 2功能包安装配置

本文件定义了功能包的安装配置，包括：
- 包信息
- 入口点（节点）
- 数据文件
"""
from setuptools import find_packages, setup
import os
from glob import glob

# 获取包目录路径
package_name = 'dji_referee_protocol'

setup(
    # 包基本信息
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),

    # 安装数据文件
    data_files=[
        # 启动文件目录
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 配置文件目录
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 包资源标记文件
        (os.path.join('share', package_name), ['package.xml']),
        # 资源文件
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],

    # 安装依赖
    install_requires=['setuptools'],
    zip_safe=True,

    # 维护者信息
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='DJI RoboMaster裁判系统通信协议ROS 2功能包',
    license='MIT',
    tests_require=['pytest'],

    # 控制台入口点（节点定义）
    entry_points={
        'console_scripts': [
            # 主节点：裁判系统串口读取节点
            # 用法：ros2 run dji_referee_protocol referee_serial_node
            'referee_serial_node = dji_referee_protocol.referee_serial_node:main',
        ],
    },
)
