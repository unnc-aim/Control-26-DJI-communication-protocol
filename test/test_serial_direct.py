#!/usr/bin/env python3
"""
裁判系统串口直接测试脚本（无需 ROS 2）

本脚本直接从串口读取数据并解析，无需 ROS 2 环境。
用于快速测试协议解析功能。

使用方法：
    python3 test/test_serial_direct.py [串口设备]

    示例：
    python3 test/test_serial_direct.py /dev/ttyUSB0
    python3 test/test_serial_direct.py /dev/ttyUSB0 -b 115200

参数：
    serial_port: 串口设备路径（默认：/dev/ttyUSB0）
    -b, --baud: 波特率（默认：115200）
    -v, --verbose: 显示详细信息
"""

import sys
import time
import argparse
import signal
from datetime import datetime
from typing import Optional

# 添加父目录到路径以导入本地模块
sys.path.insert(0, '.')

# 尝试导入本地模块
try:
    from dji_referee_protocol.crc_utils import CRCUtils
    from dji_referee_protocol.protocol_constants import (
        CommandID, FrameConstants, SerialConfig
    )
    from dji_referee_protocol.data_types import (
        GameStatus, GameResult, RobotHP, FieldEvent, RefereeWarning,
        DartLaunchData, RobotPerformance, RobotHeat, RobotPosition,
        RobotBuff, DamageState, ShootData, AllowedShoot, RFIDStatus,
        DartOperatorCmd, GroundRobotPosition, RadarMarkProgress,
        SentryDecisionSync, RadarDecisionSync, MapClickData,
        MapRadarData, MapPathData, MapRobotData, CustomControllerData,
        EnemyPosition, EnemyHP, EnemyAmmo, EnemyTeamStatus, EnemyBuff, EnemyJammingKey
    )
    LOCAL_MODULES_AVAILABLE = True
except ImportError as e:
    LOCAL_MODULES_AVAILABLE = False
    print(f"警告：无法导入本地模块: {e}")
    print("请确保在项目根目录下运行此脚本")

# 尝试导入串口库
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("错误：无法导入 pyserial，请安装：pip install pyserial")


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


def color_print(text: str, color: str = Colors.ENDC) -> None:
    """彩色打印"""
    print(f"{color}{text}{Colors.ENDC}")


def format_timestamp() -> str:
    """格式化时间戳"""
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


class SerialDirectTester:
    """
    串口直接测试器

    直接从串口读取数据并解析，无需 ROS 2。
    """

    # 命令码到描述的映射
    CMD_DESCRIPTIONS = {
        0x0001: ("比赛状态", "1Hz"),
        0x0002: ("比赛结果", "触发"),
        0x0003: ("机器人血量", "3Hz"),
        0x0101: ("场地事件", "1Hz"),
        0x0104: ("裁判警告", "1Hz"),
        0x0105: ("飞镖发射数据", "1Hz"),
        0x0201: ("机器人性能", "10Hz"),
        0x0202: ("实时热量", "10Hz"),
        0x0203: ("机器人位置", "1Hz"),
        0x0204: ("机器人增益", "3Hz"),
        0x0206: ("伤害状态", "触发"),
        0x0207: ("射击数据", "触发"),
        0x0208: ("允许发弹量", "10Hz"),
        0x0209: ("RFID状态", "3Hz"),
        0x020A: ("飞镖指令", "3Hz"),
        0x020B: ("地面机器人位置", "1Hz"),
        0x020C: ("雷达标记进度", "1Hz"),
        0x020D: ("哨兵决策同步", "1Hz"),
        0x020E: ("雷达决策同步", "1Hz"),
        0x0301: ("机器人交互", "30Hz"),
        0x0302: ("控制器到机器人", "30Hz"),
        0x0303: ("小地图点击", "触发"),
        0x0305: ("小地图雷达", "5Hz"),
        0x0306: ("控制器到选手端", "触发"),
        0x0307: ("小地图路径", "1Hz"),
        0x0308: ("小地图机器人", "3Hz"),
        0x0309: ("机器人到控制器", "10Hz"),
        0x0310: ("机器人到客户端", "50Hz"),
        0x0311: ("客户端到机器人", "75Hz"),
        0x0A01: ("对方位置", "10Hz"),
        0x0A02: ("对方血量", "10Hz"),
        0x0A03: ("对方发弹量", "10Hz"),
        0x0A04: ("对方队伍状态", "10Hz"),
        0x0A05: ("对方增益", "10Hz"),
        0x0A06: ("对方干扰密钥", "10Hz"),
    }

    def __init__(self, port: str, baudrate: int = 115200, verbose: bool = False) -> None:
        """
        初始化测试器

        Args:
            port: 串口设备路径
            baudrate: 波特率
            verbose: 是否显示详细信息
        """
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose

        # 统计信息
        self.total_frames = 0
        self.valid_frames = 0
        self.invalid_frames = 0
        self.cmd_counts: dict = {}
        self.start_time: float = 0

        # 解析状态
        self.unpack_step = 0  # 0-5 对应各步骤
        self.data_len = 0
        self.index = 0
        self.protocol_packet = bytearray(FrameConstants.MAX_FRAME_SIZE)

        # 串口对象
        self.serial: Optional[serial.Serial] = None

        # 运行标志
        self.running = True

    def start(self) -> None:
        """开始测试"""
        # 打印启动信息
        self._print_header()

        # 打开串口
        if not self._open_serial():
            return

        self.start_time = time.time()

        try:
            self._read_loop()
        except KeyboardInterrupt:
            print("\n\n用户中断...")
        finally:
            self._close_serial()
            self._print_statistics()

    def _print_header(self) -> None:
        """打印启动信息"""
        print("\n" + "=" * 70)
        color_print("  DJI 裁判系统串口直接测试工具", Colors.BOLD + Colors.CYAN)
        print("=" * 70)
        print(f"  串口设备: {self.port}")
        print(f"  波特率: {self.baudrate}")
        print(f"  启动时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 70 + "\n")

    def _open_serial(self) -> bool:
        """打开串口"""
        if not SERIAL_AVAILABLE:
            print("错误：pyserial 未安装")
            return False

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            color_print(f"  串口已打开: {self.port}", Colors.GREEN)
            return True
        except serial.SerialException as e:
            color_print(f"  错误：无法打开串口 {self.port}: {e}", Colors.RED)
            return False

    def _close_serial(self) -> None:
        """关闭串口"""
        if self.serial and self.serial.is_open:
            self.serial.close()

    def _read_loop(self) -> None:
        """读取数据循环"""
        color_print("  等待数据... (按 Ctrl+C 退出)", Colors.YELLOW)
        print("-" * 70 + "\n")

        while self.running and self.serial:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self._process_data(data)
                time.sleep(0.001)
            except serial.SerialException as e:
                color_print(f"  串口错误: {e}", Colors.RED)
                break

    def _process_data(self, data: bytes) -> None:
        """
        处理接收到的数据

        使用状态机解析数据流。

        Args:
            data: 接收到的字节数据
        """
        for byte in data:
            if self.unpack_step == 0:
                # 等待帧起始字节
                if byte == FrameConstants.SOF:
                    self.unpack_step = 1
                    self.protocol_packet[self.index] = byte
                    self.index += 1
                else:
                    self.index = 0

            elif self.unpack_step == 1:
                # 读取数据长度低字节
                self.data_len = byte
                self.protocol_packet[self.index] = byte
                self.index += 1
                self.unpack_step = 2

            elif self.unpack_step == 2:
                # 读取数据长度高字节
                self.data_len |= (byte << 8)
                self.protocol_packet[self.index] = byte
                self.index += 1

                max_data_len = (FrameConstants.MAX_FRAME_SIZE -
                               FrameConstants.FRAME_HEADER_LENGTH -
                               FrameConstants.CMD_ID_LENGTH -
                               FrameConstants.FRAME_TAIL_LENGTH)
                if self.data_len < max_data_len:
                    self.unpack_step = 3
                else:
                    self.unpack_step = 0
                    self.index = 0

            elif self.unpack_step == 3:
                # 读取帧序号
                self.protocol_packet[self.index] = byte
                self.index += 1
                self.unpack_step = 4

            elif self.unpack_step == 4:
                # 读取帧头CRC8
                self.protocol_packet[self.index] = byte
                self.index += 1

                if self.index == FrameConstants.FRAME_HEADER_LENGTH:
                    if CRCUtils.verify_crc8_check_sum(
                        bytes(self.protocol_packet[:FrameConstants.FRAME_HEADER_LENGTH]),
                        FrameConstants.FRAME_HEADER_LENGTH
                    ):
                        self.unpack_step = 5
                    else:
                        if self.verbose:
                            color_print("  CRC8校验失败", Colors.RED)
                        self.unpack_step = 0
                        self.index = 0

            elif self.unpack_step == 5:
                # 读取数据段和CRC16
                total_len = (FrameConstants.FRAME_HEADER_LENGTH +
                            FrameConstants.CMD_ID_LENGTH +
                            self.data_len +
                            FrameConstants.FRAME_TAIL_LENGTH)

                if self.index < total_len:
                    self.protocol_packet[self.index] = byte
                    self.index += 1

                if self.index >= total_len:
                    self.unpack_step = 0
                    self.index = 0
                    self.total_frames += 1

                    if CRCUtils.verify_crc16_check_sum(
                        bytes(self.protocol_packet[:total_len]),
                        total_len
                    ):
                        self.valid_frames += 1
                        self._handle_valid_frame()
                    else:
                        self.invalid_frames += 1
                        if self.verbose:
                            color_print("  CRC16校验失败", Colors.RED)

    def _handle_valid_frame(self) -> None:
        """处理有效的帧"""
        # 提取命令码
        cmd_id = (self.protocol_packet[FrameConstants.CMD_ID_OFFSET] |
                  (self.protocol_packet[FrameConstants.CMD_ID_OFFSET + 1] << 8))

        # 提取数据段
        data_start = FrameConstants.DATA_OFFSET
        data_end = data_start + self.data_len
        data = bytes(self.protocol_packet[data_start:data_end])

        # 更新统计
        self.cmd_counts[cmd_id] = self.cmd_counts.get(cmd_id, 0) + 1

        # 获取描述
        desc = self.CMD_DESCRIPTIONS.get(cmd_id, ("未知命令", "未知"))
        cmd_name, freq = desc

        # 打印信息
        timestamp = format_timestamp()
        color_print(f"\n[{timestamp}]", Colors.CYAN, end=' ')
        color_print(f"{cmd_name} (0x{cmd_id:04X})", Colors.GREEN + Colors.BOLD)
        color_print(f"  频率: {freq} | 长度: {self.data_len} 字节 | 累计: {self.cmd_counts[cmd_id]}", Colors.YELLOW)

        # 打印数据
        if self.verbose:
            print("  原始数据:", data.hex())
        self._print_parsed_data(cmd_id, data)

    def _print_parsed_data(self, cmd_id: int, data: bytes) -> None:
        """
        打印解析后的数据

        Args:
            cmd_id: 命令码
            data: 数据段
        """
        print("-" * 50)

        try:
            if cmd_id == 0x0001:
                # 比赛状态
                game_type = data[0] & 0x0F
                game_progress = (data[0] >> 4) & 0x0F
                import struct
                stage_remain_time = struct.unpack('<H', data[1:3])[0]
                sync_timestamp = struct.unpack('<Q', data[3:11])[0]

                game_types = {1: "超级对抗赛", 2: "高校单项赛", 3: "ICRA挑战赛", 4: "联盟赛3V3", 5: "联盟赛步兵"}
                stages = {0: "未开始", 1: "准备阶段", 2: "自检阶段", 3: "倒计时", 4: "比赛中", 5: "结算中"}

                print(f"  比赛类型: {game_types.get(game_type, game_type)}")
                print(f"  比赛阶段: {stages.get(game_progress, game_progress)}")
                print(f"  剩余时间: {stage_remain_time} 秒")
                print(f"  UNIX时间: {sync_timestamp}")

            elif cmd_id == 0x0002:
                # 比赛结果
                results = {0: "平局", 1: "红方胜利", 2: "蓝方胜利"}
                print(f"  结果: {results.get(data[0], data[0])}")

            elif cmd_id == 0x0003:
                # 机器人血量
                import struct
                hp_values = struct.unpack('<16H', data[:32])
                print(f"  红方英雄: {hp_values[0]} | 工程: {hp_values[1]} | 3号步兵: {hp_values[2]} | 4号步兵: {hp_values[3]}")
                print(f"  红方哨兵: {hp_values[5]} | 前哨站: {hp_values[6]} | 基地: {hp_values[7]}")
                print(f"  蓝方英雄: {hp_values[8]} | 工程: {hp_values[9]} | 3号步兵: {hp_values[10]} | 4号步兵: {hp_values[11]}")
                print(f"  蓝方哨兵: {hp_values[13]} | 前哨站: {hp_values[14]} | 基地: {hp_values[15]}")

            elif cmd_id == 0x0201:
                # 机器人性能
                import struct
                robot_id = data[0]
                robot_level = data[1]
                current_hp, maximum_hp = struct.unpack('<HH', data[2:6])
                shooter_cooling, shooter_limit, chassis_limit = struct.unpack('<HHH', data[6:12])

                print(f"  机器人ID: {robot_id} | 等级: {robot_level}")
                print(f"  血量: {current_hp}/{maximum_hp}")
                print(f"  枪口冷却: {shooter_cooling} | 热量上限: {shooter_limit} | 功率限制: {chassis_limit}W")

            elif cmd_id == 0x0202:
                # 实时热量
                import struct
                buffer_energy = struct.unpack('<H', data[8:10])[0]
                heat_17mm = struct.unpack('<H', data[10:12])[0]
                heat_42mm = struct.unpack('<H', data[12:14])[0]
                print(f"  缓冲能量: {buffer_energy}J")
                print(f"  17mm热量: {heat_17mm} | 42mm热量: {heat_42mm}")

            elif cmd_id == 0x0203:
                # 机器人位置
                import struct
                x, y, angle = struct.unpack('<fff', data[:12])
                print(f"  位置: ({x:.2f}, {y:.2f}) m")
                print(f"  朝向: {angle:.2f}°")

            elif cmd_id == 0x0206:
                # 伤害状态
                armor_id = data[0] & 0x0F
                damage_type = (data[0] >> 4) & 0x0F
                damage_types = {0: "弹丸攻击", 1: "模块离线", 5: "撞击"}
                print(f"  装甲ID: {armor_id} | 伤害类型: {damage_types.get(damage_type, damage_type)}")

            elif cmd_id == 0x0207:
                # 射击数据
                import struct
                bullet_type = data[0]
                shooter_id = data[1]
                frequency = data[2]
                initial_speed = struct.unpack('<f', data[3:7])[0]
                print(f"  弹丸: {'17mm' if bullet_type == 1 else '42mm'} | 发射器: {shooter_id}")
                print(f"  射速: {frequency}Hz | 初速: {initial_speed:.2f}m/s")

            elif cmd_id == 0x0208:
                # 允许发弹量
                import struct
                ammo_17mm, ammo_42mm, gold, fortress = struct.unpack('<HHHH', data[:8])
                print(f"  17mm发弹量: {ammo_17mm} | 42mm发弹量: {ammo_42mm}")
                print(f"  金币: {gold} | 堡垒储备: {fortress}")

            else:
                # 显示原始数据
                print(f"  原始数据: {data.hex()}")

        except Exception as e:
            print(f"  解析错误: {e}")
            print(f"  原始数据: {data.hex()}")

    def _print_statistics(self) -> None:
        """打印统计信息"""
        elapsed_time = time.time() - self.start_time

        print("\n" + "=" * 70)
        color_print("  统计信息", Colors.BOLD + Colors.CYAN)
        print("=" * 70)
        print(f"  运行时间: {elapsed_time:.2f} 秒")
        print(f"  总帧数: {self.total_frames}")
        print(f"  有效帧: {self.valid_frames}")
        print(f"  无效帧: {self.invalid_frames}")

        if self.total_frames > 0:
            print(f"  有效率: {self.valid_frames / self.total_frames * 100:.1f}%")
            print(f"  平均频率: {self.valid_frames / elapsed_time:.2f} fps")

        print("-" * 70)

        color_print("\n  各命令码统计:", Colors.BOLD)
        print("-" * 70)

        sorted_cmds = sorted(self.cmd_counts.items(), key=lambda x: x[1], reverse=True)
        for cmd_id, count in sorted_cmds:
            desc = self.CMD_DESCRIPTIONS.get(cmd_id, ("未知", ""))
            color_print(f"  0x{cmd_id:04X} {desc[0]}: {count}", Colors.GREEN if count > 0 else Colors.RED)

        print("=" * 70 + "\n")


def list_serial_ports() -> None:
    """列出所有可用串口"""
    if not SERIAL_AVAILABLE:
        print("错误：pyserial 未安装")
        return

    ports = serial.tools.list_ports.comports()
    if ports:
        print("\n可用串口:")
        for port in ports:
            print(f"  {port.device}: {port.description}")
    else:
        print("未找到可用串口")


def main() -> None:
    """主函数"""
    parser = argparse.ArgumentParser(
        description='DJI 裁判系统串口直接测试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  python3 test/test_serial_direct.py /dev/ttyUSB0
  python3 test/test_serial_direct.py /dev/ttyUSB0 -b 921600
  python3 test/test_serial_direct.py --list
        '''
    )

    parser.add_argument('serial_port', nargs='?', default='/dev/ttyUSB0',
                       help='串口设备路径（默认：/dev/ttyUSB0）')
    parser.add_argument('-b', '--baud', type=int, default=115200,
                       help='波特率（默认：115200）')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='显示详细信息')
    parser.add_argument('--list', action='store_true',
                       help='列出所有可用串口')

    args = parser.parse_args()

    # 列出串口
    if args.list:
        list_serial_ports()
        return

    # 检查模块
    if not LOCAL_MODULES_AVAILABLE:
        print("错误：无法导入本地模块，请在项目根目录下运行")
        sys.exit(1)

    if not SERIAL_AVAILABLE:
        print("错误：pyserial 未安装，请运行：pip install pyserial")
        sys.exit(1)

    # 启动测试
    tester = SerialDirectTester(
        port=args.serial_port,
        baudrate=args.baud,
        verbose=args.verbose
    )
    tester.start()


if __name__ == '__main__':
    main()
