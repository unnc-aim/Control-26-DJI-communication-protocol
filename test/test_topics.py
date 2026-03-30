#!/usr/bin/env python3
"""
裁判系统话题快速测试脚本

本脚本用于快速测试裁判系统 ROS 2 话题的发布情况。
通过订阅所有话题并打印接收到的数据来验证功能。

使用方法：
    # 在 ROS 2 环境中运行
    python3 test/test_topics.py

    # 或者添加执行权限后直接运行
    chmod +x test/test_topics.py
    ./test/test_topics.py

注意：
    - 需要先启动 referee_serial_node 节点
    - 需要安装 ROS 2 Humble
"""

import sys
import time
from datetime import datetime
from typing import Any, Dict

# ROS 2 导入
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("错误：无法导入 ROS 2 模块，请确保已安装 ROS 2 Humble 并 source 环境")
    sys.exit(1)

# 自定义消息类型导入
try:
    from dji_referee_protocol.msg import (
        GameStatus, GameResult, RobotHP, FieldEvent, RefereeWarning,
        DartLaunchData, RobotPerformance, RobotHeat, RobotPosition,
        RobotBuff, DamageState, ShootData, AllowedShoot, RFIDStatus,
        DartOperatorCmd, GroundRobotPosition, RadarMarkProgress,
        SentryDecisionSync, RadarDecisionSync,
        MapClickData, MapRadarData, MapPathData, MapRobotData,
        EnemyPosition, EnemyHP, EnemyAmmo, EnemyTeamStatus,
        EnemyBuff, EnemyJammingKey,
        Constraints, SelfColor,
    )
    CUSTOM_MSGS_AVAILABLE = True
except ImportError:
    CUSTOM_MSGS_AVAILABLE = False
    print("警告：无法导入自定义消息类型，将使用通用消息订阅")


# 颜色输出
class Colors:
    """终端颜色常量"""
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def color_print(text: str, color: str = Colors.ENDC, end: str = '\n') -> None:
    """彩色打印"""
    print(f"{color}{text}{Colors.ENDC}", end=end)


def format_timestamp() -> str:
    """格式化时间戳"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def format_message(msg: Any) -> str:
    """
    格式化 ROS 消息为可读字符串

    Args:
        msg: ROS 消息对象

    Returns:
        str: 格式化后的字符串
    """
    if msg is None:
        return "None"

    # 获取消息的所有字段
    lines = []
    if hasattr(msg, 'get_fields_and_field_types'):
        fields = msg.get_fields_and_field_types()
        for field_name in fields:
            value = getattr(msg, field_name, None)
            lines.append(f"  {field_name}: {value}")
    elif hasattr(msg, '__slots__'):
        for slot in msg.__slots__:
            value = getattr(msg, slot, None)
            lines.append(f"  {slot}: {value}")
    elif hasattr(msg, '__dict__'):
        for key, value in msg.__dict__.items():
            lines.append(f"  {key}: {value}")
    else:
        return str(msg)

    return '\n'.join(lines) if lines else str(msg)


# 话题定义：(话题短名, 描述, 消息类型, 话题前缀)
COMMON_TOPICS = [
    # 常规链路数据
    ('game_status', '比赛状态', GameStatus if CUSTOM_MSGS_AVAILABLE else None),
    ('game_result', '比赛结果', GameResult if CUSTOM_MSGS_AVAILABLE else None),
    ('robot_hp', '机器人血量', RobotHP if CUSTOM_MSGS_AVAILABLE else None),
    ('field_event', '场地事件', FieldEvent if CUSTOM_MSGS_AVAILABLE else None),
    ('referee_warning', '裁判警告', RefereeWarning if CUSTOM_MSGS_AVAILABLE else None),
    ('dart_launch_data', '飞镖发射数据', DartLaunchData if CUSTOM_MSGS_AVAILABLE else None),
    ('robot_performance', '机器人性能', RobotPerformance if CUSTOM_MSGS_AVAILABLE else None),
    ('robot_heat', '实时热量', RobotHeat if CUSTOM_MSGS_AVAILABLE else None),
    ('robot_position', '机器人位置', RobotPosition if CUSTOM_MSGS_AVAILABLE else None),
    ('robot_buff', '机器人增益', RobotBuff if CUSTOM_MSGS_AVAILABLE else None),
    ('damage_state', '伤害状态', DamageState if CUSTOM_MSGS_AVAILABLE else None),
    ('shoot_data', '射击数据', ShootData if CUSTOM_MSGS_AVAILABLE else None),
    ('allowed_shoot', '允许发弹量', AllowedShoot if CUSTOM_MSGS_AVAILABLE else None),
    ('rfid_status', 'RFID状态', RFIDStatus if CUSTOM_MSGS_AVAILABLE else None),
    ('dart_operator_cmd', '飞镖指令', DartOperatorCmd if CUSTOM_MSGS_AVAILABLE else None),
    ('ground_robot_position', '地面机器人位置', GroundRobotPosition if CUSTOM_MSGS_AVAILABLE else None),
    ('radar_mark_progress', '雷达标记进度', RadarMarkProgress if CUSTOM_MSGS_AVAILABLE else None),
    ('sentry_decision_sync', '哨兵决策同步', SentryDecisionSync if CUSTOM_MSGS_AVAILABLE else None),
    ('radar_decision_sync', '雷达决策同步', RadarDecisionSync if CUSTOM_MSGS_AVAILABLE else None),
    # 选手端小地图数据
    ('map_click_data', '小地图点击', MapClickData if CUSTOM_MSGS_AVAILABLE else None),
    ('map_radar_data', '小地图雷达数据', MapRadarData if CUSTOM_MSGS_AVAILABLE else None),
    ('map_path_data', '小地图路径数据', MapPathData if CUSTOM_MSGS_AVAILABLE else None),
    ('map_robot_data', '小地图机器人数据', MapRobotData if CUSTOM_MSGS_AVAILABLE else None),
    # 雷达无线链路数据
    ('enemy_position', '对方位置', EnemyPosition if CUSTOM_MSGS_AVAILABLE else None),
    ('enemy_hp', '对方血量', EnemyHP if CUSTOM_MSGS_AVAILABLE else None),
    ('enemy_ammo', '对方发弹量', EnemyAmmo if CUSTOM_MSGS_AVAILABLE else None),
    ('enemy_team_status', '对方队伍状态', EnemyTeamStatus if CUSTOM_MSGS_AVAILABLE else None),
    ('enemy_buff', '对方增益', EnemyBuff if CUSTOM_MSGS_AVAILABLE else None),
    ('enemy_jamming_key', '对方干扰密钥', EnemyJammingKey if CUSTOM_MSGS_AVAILABLE else None),
]

PARSED_TOPICS = [
    # 解析后数据
    ('constraints', '约束状态', Constraints if CUSTOM_MSGS_AVAILABLE else None),
    ('self_color', '自车颜色', SelfColor if CUSTOM_MSGS_AVAILABLE else None),
]


class TopicTestNode(Node):
    """
    话题测试节点

    订阅所有裁判系统话题并打印接收到的数据。
    """

    def __init__(self) -> None:
        """
        初始化测试节点

        创建订阅器并统计接收的消息。
        """
        super().__init__('referee_topic_tester')

        # 消息统计
        self.message_counts: Dict[str, int] = {}
        self.total_messages: int = 0
        self.start_time: float = time.time()

        # 订阅器列表
        self._subscriptions_list = []

        # QoS 配置
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 创建所有订阅器
        self._create_subscriptions()

        # 打印启动信息
        self._print_header()

    def _create_subscriptions(self) -> None:
        """创建所有话题的订阅器"""
        from rclpy.callback_groups import ReentrantCallbackGroup
        callback_group = ReentrantCallbackGroup()

        # 订阅 /referee/common/* 话题
        for topic_name, description, msg_type in COMMON_TOPICS:
            full_topic = f'/referee/common/{topic_name}'
            self.message_counts[full_topic] = 0

            try:
                if CUSTOM_MSGS_AVAILABLE and msg_type is not None:
                    sub = self.create_subscription(
                        msg_type,
                        full_topic,
                        lambda msg, tn=full_topic, desc=description: self._topic_callback(
                            msg, tn, desc),
                        self.qos_profile,
                        callback_group=callback_group
                    )
                else:
                    # 回退到通用订阅
                    from rosidl_runtime_py.utilities import get_message
                    from rclpy.serialization import deserialize_message
                    sub = self.create_subscription(
                        type(msg_type) if msg_type else object,
                        full_topic,
                        lambda msg, tn=full_topic, desc=description: self._topic_callback(
                            msg, tn, desc),
                        self.qos_profile,
                        callback_group=callback_group
                    )
                self._subscriptions_list.append(sub)
            except Exception as e:
                self.get_logger().warn(f'无法订阅话题 {full_topic}: {e}')

        # 订阅 /referee/parsed/common/* 话题
        for topic_name, description, msg_type in PARSED_TOPICS:
            full_topic = f'/referee/parsed/common/{topic_name}'
            self.message_counts[full_topic] = 0

            try:
                if CUSTOM_MSGS_AVAILABLE and msg_type is not None:
                    sub = self.create_subscription(
                        msg_type,
                        full_topic,
                        lambda msg, tn=full_topic, desc=description: self._topic_callback(
                            msg, tn, desc),
                        self.qos_profile,
                        callback_group=callback_group
                    )
                    self._subscriptions_list.append(sub)
            except Exception as e:
                self.get_logger().warn(f'无法订阅话题 {full_topic}: {e}')

    def _print_header(self) -> None:
        """打印启动信息"""
        total_topics = len(COMMON_TOPICS) + len(PARSED_TOPICS)
        print("\n" + "=" * 70)
        color_print("  DJI 裁判系统话题测试工具", Colors.BOLD + Colors.CYAN)
        print("=" * 70)
        print(f"  订阅话题数: {total_topics}")
        print(f"    - /referee/common/*: {len(COMMON_TOPICS)}")
        print(f"    - /referee/parsed/common/*: {len(PARSED_TOPICS)}")
        print(f"  启动时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 70 + "\n")

        color_print("  等待数据... (按 Ctrl+C 退出)", Colors.YELLOW)
        print("-" * 70 + "\n")

    def _topic_callback(self, msg: Any, topic_name: str, description: str) -> None:
        """
        话题回调函数

        Args:
            msg: 接收到的消息
            topic_name: 话题名称
            description: 话题描述
        """
        # 更新统计
        self.message_counts[topic_name] += 1
        self.total_messages += 1

        # 打印消息
        timestamp = format_timestamp()
        color_print(f"\n[{timestamp}]", Colors.CYAN, end=' ')
        color_print(f"{description}", Colors.GREEN + Colors.BOLD)
        color_print(f"  话题: {topic_name}", Colors.BLUE)
        color_print(f"  消息 #{self.message_counts[topic_name]}", Colors.YELLOW)
        print("-" * 50)
        print(format_message(msg))

    def print_statistics(self) -> None:
        """打印统计信息"""
        elapsed_time = time.time() - self.start_time

        print("\n" + "=" * 70)
        color_print("  统计信息", Colors.BOLD + Colors.CYAN)
        print("=" * 70)
        print(f"  运行时间: {elapsed_time:.2f} 秒")
        print(f"  总消息数: {self.total_messages}")
        if elapsed_time > 0:
            print(f"  平均频率: {self.total_messages / elapsed_time:.2f} msg/s")
        print("-" * 70)

        color_print("\n  各话题消息数:", Colors.BOLD)
        print("-" * 70)

        # 按消息数排序
        sorted_counts = sorted(
            self.message_counts.items(),
            key=lambda x: x[1],
            reverse=True
        )

        for topic, count in sorted_counts:
            if count > 0:
                status = Colors.GREEN + "●" + Colors.ENDC
            else:
                status = Colors.RED + "○" + Colors.ENDC
            print(f"  {status} {topic}: {count}")

        print("=" * 70 + "\n")


def main() -> None:
    """主函数"""
    rclpy.init()
    node = None

    try:
        node = TopicTestNode()

        # 运行节点
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n\n")
        color_print("  用户中断，正在退出...", Colors.YELLOW)

    finally:
        # 打印统计信息
        if node is not None:
            node.print_statistics()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
