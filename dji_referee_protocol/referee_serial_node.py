"""
裁判系统串口读取节点

本节点是ROS 2功能包的主节点，实现以下功能：
1. 从配置的串口读取裁判系统数据
2. 使用协议解析器解析数据
3. 将解析后的数据发布为独立的ROS 2话题
4. 支持通过YAML配置文件控制每个话题的发布

使用方法：
    ros2 run dji_referee_protocol referee_serial_node

参数：
    - serial_port_normal: 常规链路串口设备路径（默认：/dev/ttyUSB0）
    - serial_port_video: 图传链路串口设备路径（默认：/dev/ttyUSB1）
    - config_file: 配置文件路径（默认：config/topic_config.yaml）

发布话题：
    - /referee/game_status (GameStatus)
    - /referee/game_result (GameResult)
    - /referee/robot_hp (RobotHP)
    - ... 等其他话题

协议版本：V1.2.0
兼容ROS 2版本：Humble
"""

import threading
import time
from typing import Dict, Any, Optional

# ROS 2 导入
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# 尝试导入串口库
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("警告：pyserial未安装，串口功能将不可用")

# 本地模块导入
from .protocol_constants import CommandID, SerialConfig
from .protocol_parser import ProtocolParser
from .crc_utils import CRCUtils


class RefereeSerialNode(Node):
    """
    裁判系统串口读取节点

    从裁判系统串口读取数据，解析后发布为ROS 2话题。

    Attributes:
        parser: 协议解析器实例
        serial_normal: 常规链路串口实例
        serial_video: 图传链路串口实例
        publishers: 话题发布器字典
        topic_config: 话题配置字典
        running: 节点运行标志
    """

    def __init__(self) -> None:
        """
        初始化裁判系统串口读取节点

        - 声明ROS 2参数
        - 加载配置文件
        - 初始化串口
        - 创建话题发布器
        """
        super().__init__('referee_serial_node')

        # ==================== 声明参数 ====================
        self.declare_parameter('serial_port_normal', '/dev/ttyUSB0')
        self.declare_parameter('serial_port_video', '/dev/ttyUSB1')
        self.declare_parameter('serial_baud_normal', SerialConfig.NORMAL_BAUDRATE)
        self.declare_parameter('serial_baud_video', SerialConfig.VIDEO_TRANSMISSION_BAUDRATE)
        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_all_topics', True)

        # 获取参数
        self.serial_port_normal = self.get_parameter('serial_port_normal').value
        self.serial_port_video = self.get_parameter('serial_port_video').value
        self.serial_baud_normal = self.get_parameter('serial_baud_normal').value
        self.serial_baud_video = self.get_parameter('serial_baud_video').value
        self.config_file = self.get_parameter('config_file').value
        self.publish_all_topics = self.get_parameter('publish_all_topics').value

        # ==================== 初始化协议解析器 ====================
        self.parser_normal = ProtocolParser()
        self.parser_video = ProtocolParser()

        # ==================== 加载话题配置 ====================
        self.topic_config: Dict[str, bool] = self._load_topic_config()

        # ==================== 初始化串口 ====================
        self.serial_normal: Optional[serial.Serial] = None
        self.serial_video: Optional[serial.Serial] = None
        self._init_serial_ports()

        # ==================== 创建话题发布器 ====================
        self.publishers: Dict[int, Any] = {}
        self._create_publishers()

        # ==================== QoS配置 ====================
        # 使用可靠传输，保留最近10条消息
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ==================== 运行控制 ====================
        self.running = True
        self.read_thread_normal: Optional[threading.Thread] = None
        self.read_thread_video: Optional[threading.Thread] = None

        # 启动读取线程
        self._start_read_threads()

        self.get_logger().info('裁判系统串口读取节点已启动')
        self.get_logger().info(f'常规链路串口: {self.serial_port_normal} @ {self.serial_baud_normal}')
        self.get_logger().info(f'图传链路串口: {self.serial_port_video} @ {self.serial_baud_video}')

    def _load_topic_config(self) -> Dict[str, bool]:
        """
        加载话题配置文件

        从YAML配置文件加载每个话题的启用/禁用状态。

        Returns:
            Dict[str, bool]: 话题名称到启用状态的映射
        """
        # 默认配置：所有话题都启用
        default_config = {
            'game_status': True,
            'game_result': True,
            'robot_hp': True,
            'field_event': True,
            'referee_warning': True,
            'dart_launch_data': True,
            'robot_performance': True,
            'robot_heat': True,
            'robot_position': True,
            'robot_buff': True,
            'damage_state': True,
            'shoot_data': True,
            'allowed_shoot': True,
            'rfid_status': True,
            'dart_operator_cmd': True,
            'ground_robot_position': True,
            'radar_mark_progress': True,
            'sentry_decision_sync': True,
            'radar_decision_sync': True,
            'map_click_data': True,
            'map_radar_data': True,
            'map_path_data': True,
            'map_robot_data': True,
            'enemy_position': True,
            'enemy_hp': True,
            'enemy_ammo': True,
            'enemy_team_status': True,
            'enemy_buff': True,
            'enemy_jamming_key': True,
        }

        # 如果指定了配置文件，尝试加载
        if self.config_file:
            try:
                import yaml
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    user_config = yaml.safe_load(f)
                    if user_config and 'topics' in user_config:
                        # 合并用户配置
                        default_config.update(user_config['topics'])
                        self.get_logger().info(f'已加载配置文件: {self.config_file}')
            except Exception as e:
                self.get_logger().warn(f'加载配置文件失败: {e}，使用默认配置')

        return default_config

    def _init_serial_ports(self) -> None:
        """
        初始化串口

        尝试打开配置的串口设备。如果失败，记录警告但不会退出。
        """
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial未安装，串口功能不可用')
            return

        # 初始化常规链路串口
        try:
            self.serial_normal = serial.Serial(
                port=self.serial_port_normal,
                baudrate=self.serial_baud_normal,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SerialConfig.TIMEOUT
            )
            self.get_logger().info(f'已打开常规链路串口: {self.serial_port_normal}')
        except serial.SerialException as e:
            self.get_logger().warn(f'无法打开常规链路串口 {self.serial_port_normal}: {e}')

        # 初始化图传链路串口
        try:
            self.serial_video = serial.Serial(
                port=self.serial_port_video,
                baudrate=self.serial_baud_video,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SerialConfig.TIMEOUT
            )
            self.get_logger().info(f'已打开图传链路串口: {self.serial_port_video}')
        except serial.SerialException as e:
            self.get_logger().warn(f'无法打开图传链路串口 {self.serial_port_video}: {e}')

    def _create_publishers(self) -> None:
        """
        创建ROS 2话题发布器

        根据配置为每个数据类型创建对应的发布器。
        使用标准ROS 2消息类型或自定义消息类型。
        """
        # 定义话题映射：命令码 -> (话题名, 是否启用配置键)
        topic_mappings = {
            CommandID.GAME_STATUS: ('game_status', 'game_status'),
            CommandID.GAME_RESULT: ('game_result', 'game_result'),
            CommandID.ROBOT_HP: ('robot_hp', 'robot_hp'),
            CommandID.FIELD_EVENT: ('field_event', 'field_event'),
            CommandID.REFEREE_WARNING: ('referee_warning', 'referee_warning'),
            CommandID.DART_LAUNCH_DATA: ('dart_launch_data', 'dart_launch_data'),
            CommandID.ROBOT_PERFORMANCE: ('robot_performance', 'robot_performance'),
            CommandID.ROBOT_HEAT: ('robot_heat', 'robot_heat'),
            CommandID.ROBOT_POSITION: ('robot_position', 'robot_position'),
            CommandID.ROBOT_BUFF: ('robot_buff', 'robot_buff'),
            CommandID.DAMAGE_STATE: ('damage_state', 'damage_state'),
            CommandID.SHOOT_DATA: ('shoot_data', 'shoot_data'),
            CommandID.ALLOWED_SHOOT: ('allowed_shoot', 'allowed_shoot'),
            CommandID.RFID_STATUS: ('rfid_status', 'rfid_status'),
            CommandID.DART_OPERATOR_CMD: ('dart_operator_cmd', 'dart_operator_cmd'),
            CommandID.GROUND_ROBOT_POSITION: ('ground_robot_position', 'ground_robot_position'),
            CommandID.RADAR_MARK_PROGRESS: ('radar_mark_progress', 'radar_mark_progress'),
            CommandID.SENTRY_DECISION_SYNC: ('sentry_decision_sync', 'sentry_decision_sync'),
            CommandID.RADAR_DECISION_SYNC: ('radar_decision_sync', 'radar_decision_sync'),
            CommandID.MAP_CLICK_DATA: ('map_click_data', 'map_click_data'),
            CommandID.MAP_RADAR_DATA: ('map_radar_data', 'map_radar_data'),
            CommandID.MAP_PATH_DATA: ('map_path_data', 'map_path_data'),
            CommandID.MAP_ROBOT_DATA: ('map_robot_data', 'map_robot_data'),
            CommandID.ENEMY_POSITION: ('enemy_position', 'enemy_position'),
            CommandID.ENEMY_HP: ('enemy_hp', 'enemy_hp'),
            CommandID.ENEMY_AMMO: ('enemy_ammo', 'enemy_ammo'),
            CommandID.ENEMY_TEAM_STATUS: ('enemy_team_status', 'enemy_team_status'),
            CommandID.ENEMY_BUFF: ('enemy_buff', 'enemy_buff'),
            CommandID.ENEMY_JAMMING_KEY: ('enemy_jamming_key', 'enemy_jamming_key'),
        }

        # 为每个话题创建发布器
        for cmd_id, (topic_name, config_key) in topic_mappings.items():
            # 检查配置是否启用
            if self.publish_all_topics or self.topic_config.get(config_key, True):
                # 所有话题使用自定义消息类型（通过dict传输）
                # 实际使用时可以根据需要定义自定义消息
                self.publishers[cmd_id] = self.create_publisher(
                    PoseStamped,  # 使用PoseStamped作为通用消息类型
                    f'/referee/{topic_name}',
                    10
                )
                self.get_logger().debug(f'创建话题: /referee/{topic_name}')

    def _start_read_threads(self) -> None:
        """
        启动串口读取线程

        为常规链路和图传链路分别创建读取线程。
        """
        # 常规链路读取线程
        if self.serial_normal:
            self.read_thread_normal = threading.Thread(
                target=self._read_serial_normal,
                daemon=True
            )
            self.read_thread_normal.start()

        # 图传链路读取线程
        if self.serial_video:
            self.read_thread_video = threading.Thread(
                target=self._read_serial_video,
                daemon=True
            )
            self.read_thread_video.start()

    def _read_serial_normal(self) -> None:
        """
        常规链路串口读取线程

        持续从常规链路串口读取数据并解析。
        """
        while self.running and self.serial_normal:
            try:
                if self.serial_normal.in_waiting > 0:
                    data = self.serial_normal.read(self.serial_normal.in_waiting)
                    self.parser_normal.feed_data(data)

                    # 尝试解包数据
                    while True:
                        result = self.parser_normal.unpack()
                        if result is None:
                            break
                        cmd_id, parsed_data = result
                        self._publish_data(cmd_id, parsed_data)

                time.sleep(0.001)  # 1ms间隔，避免CPU占用过高

            except serial.SerialException as e:
                self.get_logger().error(f'常规链路串口读取错误: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'常规链路解析错误: {e}')

    def _read_serial_video(self) -> None:
        """
        图传链路串口读取线程

        持续从图传链路串口读取数据并解析。
        """
        while self.running and self.serial_video:
            try:
                if self.serial_video.in_waiting > 0:
                    data = self.serial_video.read(self.serial_video.in_waiting)
                    self.parser_video.feed_data(data)

                    # 尝试解包数据
                    while True:
                        result = self.parser_video.unpack()
                        if result is None:
                            break
                        cmd_id, parsed_data = result
                        self._publish_data(cmd_id, parsed_data)

                time.sleep(0.001)  # 1ms间隔，避免CPU占用过高

            except serial.SerialException as e:
                self.get_logger().error(f'图传链路串口读取错误: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'图传链路解析错误: {e}')

    def _publish_data(self, cmd_id: int, data: Any) -> None:
        """
        发布解析后的数据

        将解析后的数据发布到对应的ROS 2话题。

        Args:
            cmd_id: 命令码ID
            data: 解析后的数据对象
        """
        if cmd_id not in self.publishers:
            return

        try:
            # 创建消息头
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'referee'

            # 根据数据类型创建对应的ROS消息
            msg = self._create_ros_message(cmd_id, data, header)
            if msg:
                self.publishers[cmd_id].publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布数据错误 (cmd_id=0x{cmd_id:04X}): {e}')

    def _create_ros_message(self, cmd_id: int, data: Any, header: Header) -> Optional[PoseStamped]:
        """
        创建ROS消息

        将解析后的数据转换为ROS消息。当前使用PoseStamped作为通用消息类型，
        将位置信息放入pose字段，其他信息可通过扩展方式传递。

        Args:
            cmd_id: 命令码ID
            data: 解析后的数据对象
            header: ROS消息头

        Returns:
            Optional[PoseStamped]: ROS消息对象
        """
        msg = PoseStamped()
        msg.header = header

        # 对于包含位置信息的数据，提取坐标
        if hasattr(data, 'x') and hasattr(data, 'y'):
            msg.pose.position.x = float(data.x)
            msg.pose.position.y = float(data.y)
            if hasattr(data, 'angle'):
                # 将角度转换为四元数（假设是2D旋转）
                import math
                yaw = float(data.angle) * math.pi / 180.0
                msg.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.orientation.w = math.cos(yaw / 2.0)

        return msg

    def destroy_node(self) -> None:
        """
        销毁节点

        关闭串口并停止读取线程。
        """
        self.running = False

        # 等待线程结束
        if self.read_thread_normal and self.read_thread_normal.is_alive():
            self.read_thread_normal.join(timeout=1.0)
        if self.read_thread_video and self.read_thread_video.is_alive():
            self.read_thread_video.join(timeout=1.0)

        # 关闭串口
        if self.serial_normal and self.serial_normal.is_open:
            self.serial_normal.close()
        if self.serial_video and self.serial_video.is_open:
            self.serial_video.close()

        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """
    节点主入口函数

    初始化ROS 2，创建节点实例，并进入spin循环。

    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)

    try:
        node = RefereeSerialNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
