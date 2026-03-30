#!/usr/bin/env python3
"""
测试话题配置的 glob 模式过滤逻辑

验证 issue #2 的修复：配置了 glob 禁用规则后，话题应被正确过滤。
无需 ROS 2 环境，直接测试 _is_topic_enabled_by_config 方法。
"""

import sys
import os
import tempfile
from typing import Dict, List, Tuple, Optional

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import fnmatch


# ============================================================
# 从 referee_serial_node.py 中提取需要测试的方法（避免依赖 ROS 2）
# ============================================================
class TopicConfigTester:
    """从 RefereeSerialNode 中提取的话题配置测试类"""

    def __init__(self):
        self.topic_config: Dict[str, bool] = {}
        self.glob_patterns: List[Tuple[str, bool]] = []

    def _match_glob_pattern(self, topic_name: str, pattern: str) -> bool:
        if '**' in pattern:
            parts = pattern.split('**')
            if len(parts) == 2:
                prefix, suffix = parts
                if not topic_name.startswith(prefix):
                    return False
                if suffix:
                    return topic_name.endswith(suffix)
                return True

        if '*' in pattern and '**' not in pattern:
            pattern_parts = pattern.split('/')
            topic_parts = topic_name.split('/')
            if len(pattern_parts) != len(topic_parts):
                return False
            for p_part, t_part in zip(pattern_parts, topic_parts):
                if not fnmatch.fnmatch(t_part, p_part):
                    return False
            return True

        return topic_name == pattern

    def _is_topic_enabled_by_config(self, topic_path: str, config_key: str) -> Optional[bool]:
        # 1. 检查用户指定的完整路径匹配
        if topic_path in self.topic_config:
            return self.topic_config[topic_path]

        # 2. 检查glob模式匹配
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

    def load_issue_config(self):
        """加载 issue #2 中用户提供的配置"""
        # 模拟 _load_topic_config 的行为：先加载默认短名称配置
        default_config = {
            'game_status': True, 'game_result': True, 'robot_hp': True,
            'field_event': True, 'referee_warning': True, 'dart_launch_data': True,
            'robot_performance': True, 'robot_heat': True, 'robot_position': True,
            'robot_buff': True, 'damage_state': True, 'shoot_data': True,
            'allowed_shoot': True, 'rfid_status': True, 'dart_operator_cmd': True,
            'ground_robot_position': True, 'radar_mark_progress': True,
            'sentry_decision_sync': True, 'radar_decision_sync': True,
            'map_click_data': True, 'map_radar_data': True,
            'map_path_data': True, 'map_robot_data': True,
            'enemy_position': True, 'enemy_hp': True,
            'enemy_ammo': True, 'enemy_team_status': True,
            'enemy_buff': True, 'enemy_jamming_key': True,
        }
        self.topic_config = default_config.copy()
        self.glob_patterns = []

        # 用户配置（来自 issue #2）
        user_yaml = {
            '/referee/common/*': False,
            '/referee/common/game_status': True,
            '/referee/common/robot_performance': True,
            '/referee/common/robot_heat': True,
            '/referee/common/shoot_data': True,
            '/referee/common/allowed_shoot': True,
            '/referee/common/enemy_*': True,
            '/referee/parsed/**': True,
        }

        for key, value in user_yaml.items():
            if '*' in key or '**' in key:
                self.glob_patterns.append((key, bool(value)))
            else:
                self.topic_config[key] = bool(value)


# ============================================================
# 测试用例
# ============================================================
def run_tests():
    tester = TopicConfigTester()
    tester.load_issue_config()

    # 定义所有话题：config_key -> topic_path
    topics = {
        'game_status': '/referee/common/game_status',
        'game_result': '/referee/common/game_result',
        'robot_hp': '/referee/common/robot_hp',
        'robot_performance': '/referee/common/robot_performance',
        'robot_heat': '/referee/common/robot_heat',
        'robot_buff': '/referee/common/robot_buff',
        'shoot_data': '/referee/common/shoot_data',
        'allowed_shoot': '/referee/common/allowed_shoot',
        'enemy_position': '/referee/common/enemy_position',
        'enemy_hp': '/referee/common/enemy_hp',
        'enemy_ammo': '/referee/common/enemy_ammo',
        'enemy_team_status': '/referee/common/enemy_team_status',
        'enemy_buff': '/referee/common/enemy_buff',
        'enemy_jamming_key': '/referee/common/enemy_jamming_key',
        'field_event': '/referee/common/field_event',
        'damage_state': '/referee/common/damage_state',
        'rfid_status': '/referee/common/rfid_status',
        'map_click_data': '/referee/common/map_click_data',
    }

    # 期望结果（基于 issue 配置）
    expected = {
        # 显式启用（完整路径覆盖 glob）
        'game_status': True,
        'robot_performance': True,
        'robot_heat': True,
        'shoot_data': True,
        'allowed_shoot': True,
        # enemy_* 被 glob 启用
        'enemy_position': True,
        'enemy_hp': True,
        'enemy_ammo': True,
        'enemy_team_status': True,
        'enemy_buff': True,
        'enemy_jamming_key': True,
        # 被 /referee/common/*: false 禁用（未单独启用）
        'game_result': False,
        'robot_hp': False,
        'robot_buff': False,
        'field_event': False,
        'damage_state': False,
        'rfid_status': False,
        'map_click_data': False,
    }

    passed = 0
    failed = 0

    print("=" * 60)
    print("  话题配置过滤测试（issue #2 配置）")
    print("=" * 60)

    for config_key, topic_path in topics.items():
        result = tester._is_topic_enabled_by_config(topic_path, config_key)
        exp = expected[config_key]
        status = "PASS" if result == exp else "FAIL"

        if result == exp:
            passed += 1
            icon = "✓"
        else:
            failed += 1
            icon = "✗"

        tag = "启用" if result else "禁用"
        reason = ""
        if config_key in ('game_status', 'robot_performance', 'robot_heat',
                          'shoot_data', 'allowed_shoot'):
            reason = "(完整路径覆盖)"
        elif config_key.startswith('enemy_'):
            reason = "(enemy_* glob 覆盖)"
        else:
            reason = "(common/* glob 禁用)"

        print(f"  {icon} {config_key:<25s} → {tag:>4s}  {reason}")

    print("-" * 60)
    print(f"  结果: {passed} 通过, {failed} 失败 / {passed + failed} 总计")
    print("=" * 60)

    return failed == 0


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
