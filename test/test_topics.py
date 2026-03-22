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
import json
from datetime import datetime
from typing import Any, Dict, Optional
from dataclasses import asdict, is_dataclass

# ROS 2 导入
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("错误：无法导入 ROS 2 模块，请确保已安装 ROS 2 Humble 并 source 环境")
    sys.exit(1)


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

    # 处理 String(JSON) 消息
    if isinstance(msg, String):
        try:
            parsed = json.loads(msg.data)
            return json.dumps(parsed, indent=2, ensure_ascii=False)
        except Exception:
            return msg.data

    # 尝试转换为字典
    try:
        if hasattr(msg, '__dict__'):
            return json.dumps(msg.__dict__, indent=2, default=str, ensure_ascii=False)
        return str(msg)
    except Exception:
        return str(msg)


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

        # 定义所有话题
        self.topics = [
            # 常规链路数据
            ('game_status', '比赛状态'),
            ('game_result', '比赛结果'),
            ('robot_hp', '机器人血量'),
            ('field_event', '场地事件'),
            ('referee_warning', '裁判警告'),
            ('dart_launch_data', '飞镖发射数据'),
            ('robot_performance', '机器人性能'),
            ('robot_heat', '实时热量'),
            ('robot_position', '机器人位置'),
            ('robot_buff', '机器人增益'),
            ('damage_state', '伤害状态'),
            ('shoot_data', '射击数据'),
            ('allowed_shoot', '允许发弹量'),
            ('rfid_status', 'RFID状态'),
            ('dart_operator_cmd', '飞镖指令'),
            ('ground_robot_position', '地面机器人位置'),
            ('radar_mark_progress', '雷达标记进度'),
            ('sentry_decision_sync', '哨兵决策同步'),
            ('radar_decision_sync', '雷达决策同步'),
            # 选手端小地图数据
            ('map_click_data', '小地图点击'),
            ('map_radar_data', '小地图雷达数据'),
            ('map_path_data', '小地图路径数据'),
            ('map_robot_data', '小地图机器人数据'),
            # 图传链路数据
            ('custom_controller_to_robot', '控制器到机器人'),
            ('robot_to_custom_controller', '机器人到控制器'),
            ('robot_to_custom_client', '机器人到客户端'),
            ('custom_client_to_robot', '客户端到机器人'),
            # 雷达无线链路数据
            ('enemy_position', '对方位置'),
            ('enemy_hp', '对方血量'),
            ('enemy_ammo', '对方发弹量'),
            ('enemy_team_status', '对方队伍状态'),
            ('enemy_buff', '对方增益'),
            ('enemy_jamming_key', '对方干扰密钥'),
        ]

        # 创建所有订阅器
        self._create_subscriptions()

        # 打印启动信息
        self._print_header()

    def _create_subscriptions(self) -> None:
        """创建所有话题的订阅器"""
        for topic_name, description in self.topics:
            full_topic = f'/referee/{topic_name}'
            self.message_counts[full_topic] = 0

            try:
                sub = self.create_subscription(
                    String,
                    full_topic,
                    lambda msg, tn=full_topic, desc=description: self._topic_callback(
                        msg, tn, desc),
                    self.qos_profile
                )
                self._subscriptions_list.append(sub)
            except Exception as e:
                self.get_logger().warn(f'无法订阅话题 {full_topic}: {e}')

    def _print_header(self) -> None:
        """打印启动信息"""
        print("\n" + "=" * 70)
        color_print("  DJI 裁判系统话题测试工具", Colors.BOLD + Colors.CYAN)
        print("=" * 70)
        print(f"  订阅话题数: {len(self.topics)}")
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
