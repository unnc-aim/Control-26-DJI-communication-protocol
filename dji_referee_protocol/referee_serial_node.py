"""
裁判系统串口读取节点

本节点是ROS 2功能包的主节点，实现以下功能：
1. 从配置的串口读取裁判系统数据
2. 使用协议解析器解析数据
3. 将解析后的数据发布为独立的ROS 2话题
4. 支持通过YAML配置文件控制每个话题的发布
5. 使用自定义消息类型替代JSON序列化
6. 支持glob模式匹配配置

使用方法：
    ros2 run dji_referee_protocol referee_serial_node

参数：
    - serial_port: 串口设备路径（默认：/dev/ttyUSB0）
    - config_file: 配置文件路径（默认：config/topic_config.yaml）

Glob模式配置示例：
    '/referee/common/*': false      # 禁用所有 /referee/common/ 下的话题
    '/referee/common/game_status': true   # 但启用 game_status
    '/referee/parsed/**': true      # 启用所有 /referee/parsed/ 下的话题（包括子目录）

发布话题（原始数据）：
    /referee/common/game_status (GameStatus)
    /referee/common/robot_hp (RobotHP)
    ... 等其他话题

发布话题（解析后数据）：
    /referee/parsed/common/constraints (Constraints)
    /referee/parsed/common/self_color (SelfColor)

协议版本：V1.2.0
兼容ROS 2版本：Humble
"""

import fnmatch
import threading
import time
from typing import Dict, Any, Optional, List, Tuple

# ROS 2 导入
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# 自定义消息导入
from dji_referee_protocol.msg import (
    Constants,
    GameStatus,
    GameResult,
    RobotHP,
    FieldEvent,
    RefereeWarning,
    DartLaunchData,
    RobotPerformance,
    RobotHeat,
    RobotPosition,
    RobotBuff,
    DamageState,
    ShootData,
    AllowedShoot,
    RFIDStatus,
    DartOperatorCmd,
    GroundRobotPosition,
    RadarMarkProgress,
    SentryDecisionSync,
    RadarDecisionSync,
    MapClickData,
    MapRadarData,
    MapPathData,
    MapRobotData,
    EnemyPosition,
    EnemyHP,
    EnemyAmmo,
    EnemyTeamStatus,
    EnemyBuff,
    EnemyJammingKey,
    Constraints,
    SelfColor,
)

# 尝试导入串口库
import serial

# 本地模块导入
from .protocol_constants import CommandID, SerialConfig
from .protocol_parser import ProtocolParser
from .protocol_constants import OperatorClientID
from .ui_protocol import (
    UIDrawingProtocol,
    UIDataCommandID,
    UIGraphic,
    UIGraphicOperation,
    UIGraphicType,
    UIColor,
)


class RefereeSerialNode(Node):
    """
    裁判系统串口读取节点

    从裁判系统串口读取数据，解析后发布为ROS 2话题。
    使用自定义消息类型，发布到/referee/common/和/referee/parsed/common/命名空间。
    """

    def __init__(self) -> None:
        super().__init__('referee_serial_node')

        # ==================== 声明参数 ====================
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', SerialConfig.NORMAL_BAUDRATE)
        self.declare_parameter('config_file', '')
        self.declare_parameter('publish_all_topics', True)
        self.declare_parameter('serial_auto_baud_scan', True)
        self.declare_parameter('serial_no_data_reopen_sec', 3.0)
        self.declare_parameter('heat_lock_margin', 15.0)
        self.declare_parameter('power_high_ratio', 0.9)
        self.declare_parameter('power_hard_ratio', 1.0)
        self.declare_parameter('min_speed_scale', 0.35)
        self.declare_parameter('ui_enable_tx', False)
        self.declare_parameter('ui_update_period_sec', 0.5)
        self.declare_parameter('ui_target_client_id', 0)
        self.declare_parameter('ui_layer', 8)
        self.declare_parameter('ui_color', int(UIColor.SELF))
        self.declare_parameter('ui_anchor_x', 80)
        self.declare_parameter('ui_anchor_y', 860)
        self.declare_parameter('ui_line_gap', 35)
        self.declare_parameter('ui_font_size', 20)
        self.declare_parameter('ui_line_width', 2)

        # 获取参数
        self.serial_port_path = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.config_file = self.get_parameter('config_file').value
        self.publish_all_topics = self.get_parameter('publish_all_topics').value
        self.serial_auto_baud_scan = bool(self.get_parameter('serial_auto_baud_scan').value)
        self.serial_no_data_reopen_sec = float(self.get_parameter('serial_no_data_reopen_sec').value or 3.0)
        self.heat_lock_margin = float(self.get_parameter('heat_lock_margin').value or 15.0)
        self.power_high_ratio = float(self.get_parameter('power_high_ratio').value or 0.9)
        self.power_hard_ratio = float(self.get_parameter('power_hard_ratio').value or 1.0)
        self.min_speed_scale = float(self.get_parameter('min_speed_scale').value or 0.35)
        self.ui_enable_tx = bool(self.get_parameter('ui_enable_tx').value)
        self.ui_update_period_sec = float(self.get_parameter('ui_update_period_sec').value or 0.5)
        self.ui_target_client_id = int(self.get_parameter('ui_target_client_id').value or 0)
        self.ui_layer = int(self.get_parameter('ui_layer').value or 8)
        self.ui_color = int(self.get_parameter('ui_color').value or int(UIColor.SELF))
        self.ui_anchor_x = int(self.get_parameter('ui_anchor_x').value or 80)
        self.ui_anchor_y = int(self.get_parameter('ui_anchor_y').value or 860)
        self.ui_line_gap = int(self.get_parameter('ui_line_gap').value or 35)
        self.ui_font_size = int(self.get_parameter('ui_font_size').value or 20)
        self.ui_line_width = int(self.get_parameter('ui_line_width').value or 2)

        # ==================== 初始化协议解析器 ====================
        self.parser = ProtocolParser()

        # ==================== 加载话题配置 ====================
        self.topic_config: Dict[str, bool] = {}
        self.glob_patterns: List[Tuple[str, bool]] = []  # [(pattern, enabled), ...]
        self._load_topic_config()

        # ==================== 初始化串口 ====================
        self.serial_port: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self._init_serial_port()

        # ==================== 创建话题发布器 ====================
        self._publishers_dict: Dict[int, Any] = {}
        self._create_publishers()

        # 串口接收状态
        self.last_packet_time = time.monotonic()
        self.last_byte_time = time.monotonic()
        self.rx_bytes = 0
        self.rx_packets = 0
        self.baud_candidates = []
        preferred_baud = int(self.serial_baud) if self.serial_baud is not None else SerialConfig.NORMAL_BAUDRATE
        for b in [preferred_baud, SerialConfig.NORMAL_BAUDRATE]:
            if b not in self.baud_candidates:
                self.baud_candidates.append(b)
        self.baud_index = 0

        # 裁判约束状态
        self.state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.constraint_pub = self.create_publisher(
            Constraints, '/referee/parsed/common/constraints', self.state_qos)
        self.self_color_pub = self.create_publisher(
            SelfColor, '/referee/parsed/common/self_color', self.state_qos)

        # 约束状态变量
        self.latest_shooter_heat = 0.0
        self.latest_heat_limit = 0.0
        self.latest_chassis_power = 0.0
        self.latest_chassis_power_limit = 0.0
        self.latest_robot_id = 0
        self.latest_self_color = Constants.COLOR_UNKNOWN
        self.latest_projectile_allowance_17mm = 0
        self.latest_projectile_allowance_42mm = 0
        self.latest_remaining_gold_coin = 0
        self.latest_fortress_reserve_17mm = 0
        self.latest_fire_allowed = True
        self.latest_speed_scale = 1.0
        self.ui_tx_seq = 0
        self.ui_last_receiver_id = 0
        self.ui_initialized = False

        # 状态周期发布定时器
        self.state_timer = self.create_timer(1.0, self._publish_state_heartbeat)
        self.serial_watchdog_timer = self.create_timer(1.0, self._serial_watchdog_tick)
        self.ui_timer = None
        if self.ui_enable_tx:
            self.ui_timer = self.create_timer(max(0.1, self.ui_update_period_sec), self._ui_timer_tick)

        # ==================== 运行控制 ====================
        self.running = True
        self.read_thread: Optional[threading.Thread] = None

        # 启动读取线程
        self._start_read_thread()

        self.get_logger().info('裁判系统串口读取节点已启动')
        self.get_logger().info(f'串口: {self.serial_port_path} @ {self.serial_baud}')

    def _match_glob_pattern(self, topic_name: str, pattern: str) -> bool:
        """
        检查话题名是否匹配glob模式

        支持的通配符：
        - * : 匹配任意字符（不包括 /）
        - ** : 匹配任意字符（包括 /）

        Args:
            topic_name: 话题名称（如 /referee/common/game_status）
            pattern: glob模式（如 /referee/common/* 或 /referee/**）

        Returns:
            bool: 是否匹配
        """
        # 将 glob 模式转换为 fnmatch 兼容的模式
        # ** 匹配任意字符包括 /
        # * 匹配任意字符不包括 /

        # 先处理 ** 的情况
        if '**' in pattern:
            # 将 ** 替换为特殊标记，然后分割
            # /referee/** -> 分成前缀和后缀
            parts = pattern.split('**')
            if len(parts) == 2:
                prefix, suffix = parts
                # 检查前缀匹配
                if not topic_name.startswith(prefix):
                    return False
                # 检查后缀匹配（如果有）
                if suffix:
                    return topic_name.endswith(suffix)
                return True

        # 对于单 * 的情况，使用 fnmatch
        # 但要注意 * 不应该匹配 /
        # 将话题名中的 / 替换为特殊字符，模式中的 * 也做相应处理
        # 然后使用 fnmatch

        # 更简单的方法：使用分段匹配
        if '*' in pattern and '**' not in pattern:
            # 分割模式和话题
            pattern_parts = pattern.split('/')
            topic_parts = topic_name.split('/')

            if len(pattern_parts) != len(topic_parts):
                return False

            for p_part, t_part in zip(pattern_parts, topic_parts):
                if not fnmatch.fnmatch(t_part, p_part):
                    return False
            return True

        # 没有通配符，直接比较
        return topic_name == pattern

    def _is_topic_enabled_by_config(self, topic_path: str, config_key: str) -> Optional[bool]:
        """
        检查话题是否被配置启用

        优先级：
        1. 用户指定的完整路径配置（如 /referee/common/game_status）
        2. glob模式匹配（后定义的覆盖先定义的）
        3. 短名称默认配置（如 game_status）
        4. None（未配置）

        Args:
            topic_path: 完整话题路径（如 /referee/common/game_status）
            config_key: 配置键（如 game_status）

        Returns:
            Optional[bool]: True=启用, False=禁用, None=未配置
        """
        # 1. 检查用户指定的完整路径匹配
        if topic_path in self.topic_config:
            return self.topic_config[topic_path]

        # 2. 检查glob模式匹配
        # 后定义的模式覆盖先定义的（从前往后遍历，后面的会覆盖前面的结果）
        result = None
        for pattern, enabled in self.glob_patterns:
            if self._match_glob_pattern(topic_path, pattern):
                result = enabled

        if result is not None:
            return result

        # 3. 检查短名称默认配置
        if config_key in self.topic_config:
            return self.topic_config[config_key]

        return result

    def _load_topic_config(self) -> None:
        """加载话题配置文件，所有配置都在topics下，支持glob模式"""
        # 默认配置（短名称，用于内部匹配）
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

        self.topic_config = default_config.copy()
        self.glob_patterns = []  # 清空，重新从配置文件加载

        if self.config_file:
            try:
                import yaml
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    user_config = yaml.safe_load(f)
                    if user_config and 'topics' in user_config:
                        topics_config = user_config['topics']

                        # 分离glob模式和具体配置
                        for key, value in topics_config.items():
                            if '*' in key or '**' in key:
                                # Glob模式
                                self.glob_patterns.append((key, bool(value)))
                            else:
                                # 具体配置（支持短名称和完整路径）
                                self.topic_config[key] = bool(value)

                        self.get_logger().info(f'已加载配置文件: {self.config_file}')

                        # 打印配置摘要
                        if self.glob_patterns:
                            enabled_patterns = [p for p, e in self.glob_patterns if e]
                            disabled_patterns = [p for p, e in self.glob_patterns if not e]
                            if enabled_patterns:
                                self.get_logger().info(f'启用模式 ({len(enabled_patterns)}): {", ".join(enabled_patterns)}')
                            if disabled_patterns:
                                self.get_logger().info(f'禁用模式 ({len(disabled_patterns)}): {", ".join(disabled_patterns)}')

            except Exception as e:
                self.get_logger().warn(f'加载配置文件失败: {e}，使用默认配置')

    def _init_serial_port(self) -> None:
        """初始化串口"""
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_path,
                baudrate=int(self.serial_baud) if self.serial_baud is not None else SerialConfig.NORMAL_BAUDRATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SerialConfig.TIMEOUT
            )
            self.get_logger().info(f'已打开串口: {self.serial_port_path}')
        except serial.SerialException as e:
            self.get_logger().warn(f'无法打开串口 {self.serial_port_path}: {e}')

    def _create_publishers(self) -> None:
        """创建ROS 2话题发布器，支持glob模式匹配配置"""
        # 定义话题映射：命令码 -> (话题名, 消息类型, 配置键)
        topic_mappings = {
            CommandID.GAME_STATUS: ('game_status', GameStatus, 'game_status'),
            CommandID.GAME_RESULT: ('game_result', GameResult, 'game_result'),
            CommandID.ROBOT_HP: ('robot_hp', RobotHP, 'robot_hp'),
            CommandID.FIELD_EVENT: ('field_event', FieldEvent, 'field_event'),
            CommandID.REFEREE_WARNING: ('referee_warning', RefereeWarning, 'referee_warning'),
            CommandID.DART_LAUNCH_DATA: ('dart_launch_data', DartLaunchData, 'dart_launch_data'),
            CommandID.ROBOT_PERFORMANCE: ('robot_performance', RobotPerformance, 'robot_performance'),
            CommandID.ROBOT_HEAT: ('robot_heat', RobotHeat, 'robot_heat'),
            CommandID.ROBOT_POSITION: ('robot_position', RobotPosition, 'robot_position'),
            CommandID.ROBOT_BUFF: ('robot_buff', RobotBuff, 'robot_buff'),
            CommandID.DAMAGE_STATE: ('damage_state', DamageState, 'damage_state'),
            CommandID.SHOOT_DATA: ('shoot_data', ShootData, 'shoot_data'),
            CommandID.ALLOWED_SHOOT: ('allowed_shoot', AllowedShoot, 'allowed_shoot'),
            CommandID.RFID_STATUS: ('rfid_status', RFIDStatus, 'rfid_status'),
            CommandID.DART_OPERATOR_CMD: ('dart_operator_cmd', DartOperatorCmd, 'dart_operator_cmd'),
            CommandID.GROUND_ROBOT_POSITION: ('ground_robot_position', GroundRobotPosition, 'ground_robot_position'),
            CommandID.RADAR_MARK_PROGRESS: ('radar_mark_progress', RadarMarkProgress, 'radar_mark_progress'),
            CommandID.SENTRY_DECISION_SYNC: ('sentry_decision_sync', SentryDecisionSync, 'sentry_decision_sync'),
            CommandID.RADAR_DECISION_SYNC: ('radar_decision_sync', RadarDecisionSync, 'radar_decision_sync'),
            CommandID.MAP_CLICK_DATA: ('map_click_data', MapClickData, 'map_click_data'),
            CommandID.MAP_RADAR_DATA: ('map_radar_data', MapRadarData, 'map_radar_data'),
            CommandID.MAP_PATH_DATA: ('map_path_data', MapPathData, 'map_path_data'),
            CommandID.MAP_ROBOT_DATA: ('map_robot_data', MapRobotData, 'map_robot_data'),
            CommandID.ENEMY_POSITION: ('enemy_position', EnemyPosition, 'enemy_position'),
            CommandID.ENEMY_HP: ('enemy_hp', EnemyHP, 'enemy_hp'),
            CommandID.ENEMY_AMMO: ('enemy_ammo', EnemyAmmo, 'enemy_ammo'),
            CommandID.ENEMY_TEAM_STATUS: ('enemy_team_status', EnemyTeamStatus, 'enemy_team_status'),
            CommandID.ENEMY_BUFF: ('enemy_buff', EnemyBuff, 'enemy_buff'),
            CommandID.ENEMY_JAMMING_KEY: ('enemy_jamming_key', EnemyJammingKey, 'enemy_jamming_key'),
        }

        enabled_topics = []
        disabled_topics = []
        glob_matched_topics = []

        for cmd_id, (topic_name, msg_type, config_key) in topic_mappings.items():
            # 构建完整话题路径
            topic_path = f'/referee/common/{topic_name}'

            # 使用新的配置检查方法（支持glob模式）
            config_value = self._is_topic_enabled_by_config(topic_path, config_key)

            if config_value is False:
                # 检查是否被glob模式禁用
                if config_key in self.topic_config and self.topic_config[config_key] is False:
                    disabled_topics.append(topic_name)
                else:
                    # 被glob模式禁用
                    glob_matched_topics.append(f'{topic_name} (glob)')
                continue
            elif config_value is True:
                # 检查是否被glob模式启用
                if config_key not in self.topic_config:
                    glob_matched_topics.append(f'{topic_name} (glob)')
            elif self.publish_all_topics:
                pass
            else:
                disabled_topics.append(topic_name)
                continue

            self._publishers_dict[cmd_id] = self.create_publisher(
                msg_type,
                topic_path,
                10
            )
            enabled_topics.append(topic_name)

        # 打印话题状态
        if enabled_topics:
            self.get_logger().info(f'已启用话题 ({len(enabled_topics)}): {", ".join(enabled_topics)}')
        if disabled_topics:
            self.get_logger().info(f'已禁用话题 ({len(disabled_topics)}): {", ".join(disabled_topics)}')
        if glob_matched_topics:
            self.get_logger().info(f'由glob模式匹配 ({len(glob_matched_topics)}): {", ".join(glob_matched_topics)}')

    def _start_read_thread(self) -> None:
        """启动串口读取线程"""
        if self.serial_port:
            self.read_thread = threading.Thread(
                target=self._read_serial,
                daemon=True
            )
            self.read_thread.start()

    def _read_serial(self) -> None:
        """串口读取线程"""
        while self.running:
            try:
                if not self.serial_port or not self.serial_port.is_open:
                    time.sleep(0.05)
                    continue

                data = b''
                with self.serial_lock:
                    if self.serial_port and self.serial_port.is_open and self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting)

                if data:
                    self.last_byte_time = time.monotonic()
                    self.rx_bytes += len(data)
                    self.parser.feed_data(data)

                    while True:
                        result = self.parser.unpack()
                        if result is None:
                            break
                        cmd_id, parsed_data = result
                        self.last_packet_time = time.monotonic()
                        self.rx_packets += 1
                        self._publish_data(cmd_id, parsed_data)

                time.sleep(0.001)

            except serial.SerialException as e:
                self.get_logger().error(f'串口读取错误: {e}')
                time.sleep(0.1)
            except Exception as e:
                self.get_logger().error(f'解析错误: {e}')

    def _reopen_serial(self, baud: int) -> None:
        """重开串口并切换波特率"""
        try:
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
        except Exception:
            pass

        try:
            with self.serial_lock:
                self.serial_port = serial.Serial(
                    port=self.serial_port_path,
                    baudrate=baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=SerialConfig.TIMEOUT
                )
            self.serial_baud = baud
            self.last_byte_time = time.monotonic()
            self.last_packet_time = time.monotonic()
            self.get_logger().warn(f'串口无有效数据，切换波特率重连: {self.serial_port_path} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().warn(f'串口重连失败 {self.serial_port_path} @ {baud}: {e}')
            self.serial_port = None

    def _serial_watchdog_tick(self) -> None:
        """串口看门狗：长时间无有效帧时自动重连"""
        if not self.running or not self.serial_auto_baud_scan:
            return

        now = time.monotonic()
        if now - self.last_packet_time < self.serial_no_data_reopen_sec:
            return

        self.baud_index = (self.baud_index + 1) % len(self.baud_candidates)
        next_baud = self.baud_candidates[self.baud_index]
        self._reopen_serial(next_baud)

    def _publish_data(self, cmd_id: int, data: Any) -> None:
        """发布解析后的数据"""
        try:
            self._update_constraint_state(cmd_id, data)

            if cmd_id in self._publishers_dict:
                msg = self._create_ros_message(cmd_id, data)
                if msg:
                    self._publishers_dict[cmd_id].publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布数据错误 (cmd_id=0x{cmd_id:04X}): {e}')

    def _update_constraint_state(self, cmd_id: int, data: Any) -> None:
        """根据裁判帧更新约束状态"""
        updated = False

        if cmd_id == CommandID.ROBOT_PERFORMANCE:
            self.latest_robot_id = int(getattr(data, 'robot_id', 0))
            self.latest_heat_limit = float(getattr(data, 'shooter_barrel_heat_limit', 0.0))
            self.latest_chassis_power_limit = float(getattr(data, 'chassis_power_limit', 0.0))
            self._publish_self_color_from_robot_id(self.latest_robot_id)
            updated = True
        elif cmd_id == CommandID.ROBOT_HEAT:
            self.latest_shooter_heat = float(getattr(data, 'shooter_17mm_barrel_heat', 0.0))
            self.latest_chassis_power = float(getattr(data, 'chassis_current_power', 0.0))
            updated = True
        elif cmd_id == CommandID.ALLOWED_SHOOT:
            self.latest_projectile_allowance_17mm = int(getattr(data, 'projectile_allowance_17mm', 0))
            self.latest_projectile_allowance_42mm = int(getattr(data, 'projectile_allowance_42mm', 0))
            self.latest_remaining_gold_coin = int(getattr(data, 'remaining_gold_coin', 0))
            self.latest_fortress_reserve_17mm = int(getattr(data, 'fortress_reserve_17mm', 0))
            updated = True

        if not updated:
            return

        self._publish_constraints()

    def _calculate_constraint_values(self) -> Tuple[bool, float]:
        """计算当前约束状态"""
        fire_allowed = True
        if self.latest_heat_limit > 0.0:
            fire_allowed = self.latest_shooter_heat < max(0.0, self.latest_heat_limit - self.heat_lock_margin)
        if self.latest_chassis_power_limit > 0.0 and self.latest_chassis_power > self.latest_chassis_power_limit:
            fire_allowed = False

        speed_scale = 1.0
        if self.latest_chassis_power_limit > 0.0:
            ratio = self.latest_chassis_power / self.latest_chassis_power_limit
            if ratio >= self.power_hard_ratio:
                speed_scale = self.min_speed_scale
            elif ratio > self.power_high_ratio:
                denom = max(1e-6, self.power_hard_ratio - self.power_high_ratio)
                k = (ratio - self.power_high_ratio) / denom
                speed_scale = 1.0 - k * (1.0 - self.min_speed_scale)

        return fire_allowed, float(max(0.0, min(1.0, speed_scale)))

    def _publish_constraints(self) -> None:
        """发布约束消息"""
        fire_allowed, speed_scale = self._calculate_constraint_values()
        self.latest_fire_allowed = fire_allowed
        self.latest_speed_scale = speed_scale

        msg = Constraints()
        msg.shooter_heat = float(self.latest_shooter_heat)
        msg.heat_limit = float(self.latest_heat_limit)
        msg.chassis_power = float(self.latest_chassis_power)
        msg.chassis_power_limit = float(self.latest_chassis_power_limit)
        msg.fire_allowed = fire_allowed
        msg.speed_scale = float(speed_scale)
        self.constraint_pub.publish(msg)

    def _build_ui_line_texts(self) -> Tuple[str, str]:
        """生成 UI 两行显示文本"""
        line1 = (
            f"A17:{self.latest_projectile_allowance_17mm} "
            f"A42:{self.latest_projectile_allowance_42mm} "
            f"G:{self.latest_remaining_gold_coin} "
            f"R:{self.latest_fortress_reserve_17mm}"
        )
        line2 = (
            f"H:{int(self.latest_shooter_heat)}/{int(self.latest_heat_limit)} "
            f"P:{int(self.latest_chassis_power)}/{int(self.latest_chassis_power_limit)} "
            f"F:{1 if self.latest_fire_allowed else 0} "
            f"S:{self.latest_speed_scale:.2f}"
        )
        return line1[:30], line2[:30]

    def _resolve_ui_receiver_id(self) -> int:
        """解析 UI 接收者选手端 ID"""
        if self.ui_target_client_id > 0:
            return self.ui_target_client_id

        robot_id = int(self.latest_robot_id)
        mapping = {
            1: int(OperatorClientID.RED_HERO),
            2: int(OperatorClientID.RED_ENGINEER),
            3: int(OperatorClientID.RED_INFANTRY_3),
            4: int(OperatorClientID.RED_INFANTRY_4),
            5: int(OperatorClientID.RED_INFANTRY_5),
            6: int(OperatorClientID.RED_AERIAL),
            101: int(OperatorClientID.BLUE_HERO),
            102: int(OperatorClientID.BLUE_ENGINEER),
            103: int(OperatorClientID.BLUE_INFANTRY_3),
            104: int(OperatorClientID.BLUE_INFANTRY_4),
            105: int(OperatorClientID.BLUE_INFANTRY_5),
            106: int(OperatorClientID.BLUE_AERIAL),
        }
        return mapping.get(robot_id, 0)

    def _send_ui_frame(self, data_cmd_id: int, content_payload: bytes, receiver_id: int) -> bool:
        """发送单帧 UI 数据"""
        if not self.serial_port or not self.serial_port.is_open:
            return False
        if self.latest_robot_id <= 0 or receiver_id <= 0:
            return False

        frame = UIDrawingProtocol.build_robot_interaction_frame(
            seq=self.ui_tx_seq,
            data_cmd_id=data_cmd_id,
            sender_id=self.latest_robot_id,
            receiver_id=receiver_id,
            content_payload=content_payload,
        )
        self.ui_tx_seq = (self.ui_tx_seq + 1) & 0xFF

        try:
            with self.serial_lock:
                if not self.serial_port or not self.serial_port.is_open:
                    return False
                written = self.serial_port.write(frame)
            return written == len(frame)
        except serial.SerialException as e:
            self.get_logger().warn(f'UI帧发送失败: {e}')
            return False

    def _send_ui_delete_all(self, receiver_id: int) -> bool:
        """发送 UI 全删除命令（表 1-25）"""
        payload = UIDrawingProtocol.pack_delete_payload(delete_operation=2, layer=0)
        return self._send_ui_frame(int(UIDataCommandID.DELETE), payload, receiver_id)

    def _send_ui_char_line(self, receiver_id: int, name: str, text: str, x: int, y: int, add_mode: bool) -> bool:
        """发送 UI 字符绘制命令（表 1-31）"""
        graphic = UIGraphic(
            name=name,
            operation=int(UIGraphicOperation.ADD if add_mode else UIGraphicOperation.MODIFY),
            graphic_type=int(UIGraphicType.CHAR),
            layer=self.ui_layer,
            color=self.ui_color,
            details_a=self.ui_font_size,
            details_b=len(text.encode('utf-8', errors='ignore')[:30]),
            width=self.ui_line_width,
            start_x=x,
            start_y=y,
            details_c=0,
            details_d=0,
            details_e=0,
        )
        payload = UIDrawingProtocol.pack_char_payload(graphic, text)
        return self._send_ui_frame(
            data_cmd_id=int(UIDataCommandID.DRAW_CHAR),
            content_payload=payload,
            receiver_id=receiver_id,
        )

    def _ui_timer_tick(self) -> None:
        """周期发送 UI 绘制数据"""
        if not self.ui_enable_tx:
            return

        receiver_id = self._resolve_ui_receiver_id()
        if receiver_id <= 0 or self.latest_robot_id <= 0:
            return

        need_add = (not self.ui_initialized) or (receiver_id != self.ui_last_receiver_id)
        if need_add:
            self._send_ui_delete_all(receiver_id)
            self.ui_initialized = True
            self.ui_last_receiver_id = receiver_id

        line1, line2 = self._build_ui_line_texts()
        self._send_ui_char_line(receiver_id, 'AS1', line1, self.ui_anchor_x, self.ui_anchor_y, add_mode=need_add)
        self._send_ui_char_line(
            receiver_id,
            'CS1',
            line2,
            self.ui_anchor_x,
            self.ui_anchor_y - self.ui_line_gap,
            add_mode=need_add,
        )

    def _publish_self_color_from_robot_id(self, robot_id: int) -> None:
        """根据机器人ID发布自车颜色"""
        msg = SelfColor()
        if 1 <= robot_id < 100:
            msg.color = Constants.COLOR_RED
        elif 100 <= robot_id < 200:
            msg.color = Constants.COLOR_BLUE
        else:
            msg.color = Constants.COLOR_UNKNOWN

        self.latest_self_color = msg.color
        self.self_color_pub.publish(msg)

    def _publish_state_heartbeat(self) -> None:
        """周期性发布当前颜色与约束状态"""
        msg = SelfColor()
        msg.color = self.latest_self_color
        self.self_color_pub.publish(msg)
        self._publish_constraints()

    def _create_ros_message(self, cmd_id: int, data: Any) -> Optional[Any]:
        """创建自定义ROS消息"""
        try:
            if cmd_id == CommandID.GAME_STATUS:
                msg = GameStatus()
                msg.game_type = int(getattr(data, 'game_type', 0))
                msg.game_progress = int(getattr(data, 'game_progress', 0))
                msg.stage_remain_time = int(getattr(data, 'stage_remain_time', 0))
                msg.sync_timestamp = int(getattr(data, 'sync_timestamp', 0))
                return msg

            elif cmd_id == CommandID.GAME_RESULT:
                msg = GameResult()
                msg.winner = int(getattr(data, 'winner', 0))
                return msg

            elif cmd_id == CommandID.ROBOT_HP:
                msg = RobotHP()
                msg.red_1_robot_hp = int(getattr(data, 'red_1_robot_hp', 0))
                msg.red_2_robot_hp = int(getattr(data, 'red_2_robot_hp', 0))
                msg.red_3_robot_hp = int(getattr(data, 'red_3_robot_hp', 0))
                msg.red_4_robot_hp = int(getattr(data, 'red_4_robot_hp', 0))
                msg.red_7_robot_hp = int(getattr(data, 'red_7_robot_hp', 0))
                msg.red_outpost_hp = int(getattr(data, 'red_outpost_hp', 0))
                msg.red_base_hp = int(getattr(data, 'red_base_hp', 0))
                msg.blue_1_robot_hp = int(getattr(data, 'blue_1_robot_hp', 0))
                msg.blue_2_robot_hp = int(getattr(data, 'blue_2_robot_hp', 0))
                msg.blue_3_robot_hp = int(getattr(data, 'blue_3_robot_hp', 0))
                msg.blue_4_robot_hp = int(getattr(data, 'blue_4_robot_hp', 0))
                msg.blue_7_robot_hp = int(getattr(data, 'blue_7_robot_hp', 0))
                msg.blue_outpost_hp = int(getattr(data, 'blue_outpost_hp', 0))
                msg.blue_base_hp = int(getattr(data, 'blue_base_hp', 0))
                return msg

            elif cmd_id == CommandID.ROBOT_PERFORMANCE:
                msg = RobotPerformance()
                msg.robot_id = int(getattr(data, 'robot_id', 0))
                msg.robot_level = int(getattr(data, 'robot_level', 0))
                msg.current_hp = int(getattr(data, 'current_hp', 0))
                msg.maximum_hp = int(getattr(data, 'maximum_hp', 0))
                msg.shooter_barrel_cooling_value = int(getattr(data, 'shooter_barrel_cooling_value', 0))
                msg.shooter_barrel_heat_limit = int(getattr(data, 'shooter_barrel_heat_limit', 0))
                msg.chassis_power_limit = int(getattr(data, 'chassis_power_limit', 0))
                msg.power_management_gimbal_output = bool(getattr(data, 'power_management_gimbal_output', False))
                msg.power_management_chassis_output = bool(getattr(data, 'power_management_chassis_output', False))
                msg.power_management_shooter_output = bool(getattr(data, 'power_management_shooter_output', False))
                return msg

            elif cmd_id == CommandID.ROBOT_HEAT:
                msg = RobotHeat()
                msg.chassis_current_voltage = int(getattr(data, 'chassis_current_voltage', 0))
                msg.chassis_current_current = int(getattr(data, 'chassis_current_current', 0))
                msg.chassis_current_power = float(getattr(data, 'chassis_current_power', 0.0))
                msg.buffer_energy = int(getattr(data, 'buffer_energy', 0))
                msg.shooter_17mm_barrel_heat = int(getattr(data, 'shooter_17mm_barrel_heat', 0))
                msg.shooter_42mm_barrel_heat = int(getattr(data, 'shooter_42mm_barrel_heat', 0))
                return msg

            elif cmd_id == CommandID.ROBOT_POSITION:
                msg = RobotPosition()
                msg.x = float(getattr(data, 'x', 0.0))
                msg.y = float(getattr(data, 'y', 0.0))
                msg.angle = float(getattr(data, 'angle', 0.0))
                return msg

            elif cmd_id == CommandID.ROBOT_BUFF:
                msg = RobotBuff()
                msg.recovery_buff = int(getattr(data, 'recovery_buff', 0))
                msg.cooling_buff = int(getattr(data, 'cooling_buff', 0))
                msg.defence_buff = int(getattr(data, 'defence_buff', 0))
                msg.vulnerability_buff = int(getattr(data, 'vulnerability_buff', 0))
                msg.attack_buff = int(getattr(data, 'attack_buff', 0))
                msg.remaining_energy = int(getattr(data, 'remaining_energy', 0))
                return msg

            elif cmd_id == CommandID.DAMAGE_STATE:
                msg = DamageState()
                msg.armor_id = int(getattr(data, 'armor_id', 0))
                msg.damage_type = int(getattr(data, 'damage_type', 0))
                return msg

            elif cmd_id == CommandID.SHOOT_DATA:
                msg = ShootData()
                msg.bullet_type = int(getattr(data, 'bullet_type', 0))
                msg.shooter_id = int(getattr(data, 'shooter_id', 0))
                msg.launching_frequency = int(getattr(data, 'launching_frequency', 0))
                msg.initial_speed = float(getattr(data, 'initial_speed', 0.0))
                return msg

            elif cmd_id == CommandID.ALLOWED_SHOOT:
                msg = AllowedShoot()
                msg.projectile_allowance_17mm = int(getattr(data, 'projectile_allowance_17mm', 0))
                msg.projectile_allowance_42mm = int(getattr(data, 'projectile_allowance_42mm', 0))
                msg.remaining_gold_coin = int(getattr(data, 'remaining_gold_coin', 0))
                msg.fortress_reserve_17mm = int(getattr(data, 'fortress_reserve_17mm', 0))
                return msg

            elif cmd_id == CommandID.ENEMY_POSITION:
                msg = EnemyPosition()
                msg.hero_x = int(getattr(data, 'hero_x', 0))
                msg.hero_y = int(getattr(data, 'hero_y', 0))
                msg.engineer_x = int(getattr(data, 'engineer_x', 0))
                msg.engineer_y = int(getattr(data, 'engineer_y', 0))
                msg.infantry_3_x = int(getattr(data, 'infantry_3_x', 0))
                msg.infantry_3_y = int(getattr(data, 'infantry_3_y', 0))
                msg.infantry_4_x = int(getattr(data, 'infantry_4_x', 0))
                msg.infantry_4_y = int(getattr(data, 'infantry_4_y', 0))
                msg.aerial_x = int(getattr(data, 'aerial_x', 0))
                msg.aerial_y = int(getattr(data, 'aerial_y', 0))
                msg.sentry_x = int(getattr(data, 'sentry_x', 0))
                msg.sentry_y = int(getattr(data, 'sentry_y', 0))
                return msg

            elif cmd_id == CommandID.ENEMY_HP:
                msg = EnemyHP()
                msg.hero_hp = int(getattr(data, 'hero_hp', 0))
                msg.engineer_hp = int(getattr(data, 'engineer_hp', 0))
                msg.infantry_3_hp = int(getattr(data, 'infantry_3_hp', 0))
                msg.infantry_4_hp = int(getattr(data, 'infantry_4_hp', 0))
                msg.sentry_hp = int(getattr(data, 'sentry_hp', 0))
                return msg

            elif cmd_id == CommandID.ENEMY_AMMO:
                msg = EnemyAmmo()
                msg.hero_ammo = int(getattr(data, 'hero_ammo', 0))
                msg.infantry_3_ammo = int(getattr(data, 'infantry_3_ammo', 0))
                msg.infantry_4_ammo = int(getattr(data, 'infantry_4_ammo', 0))
                msg.aerial_ammo = int(getattr(data, 'aerial_ammo', 0))
                msg.sentry_ammo = int(getattr(data, 'sentry_ammo', 0))
                return msg

            else:
                return None

        except Exception as e:
            self.get_logger().error(f'创建消息错误: {e}')
            return None

    def destroy_node(self) -> None:
        """销毁节点"""
        self.running = False

        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)

        if self.serial_port and self.serial_port.is_open:
            with self.serial_lock:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()

        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """节点主入口函数"""
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
