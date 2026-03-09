"""
协议常量定义模块

本模块定义了DJI裁判系统通信协议中使用的所有常量，包括：
- 帧格式常量
- 命令码ID
- 数据长度
- 串口配置
- 枚举值定义

所有常量严格按照官方协议文档定义。

协议版本：V1.2.0
"""

from enum import IntEnum
from typing import Dict


class FrameConstants:
    """
    帧格式常量

    定义了通信协议帧的基本结构和格式。

    帧结构:
        | frame_header | cmd_id | data | frame_tail |
        |    5-byte    | 2-byte | n-byte |   2-byte   |

    帧头结构:
        | SOF | data_length | seq | CRC8 |
        | 1B  |     2B      | 1B  |  1B  |
    """

    # 帧起始字节（Start of Frame）
    SOF: int = 0xA5

    # 帧头长度（字节）
    FRAME_HEADER_LENGTH: int = 5

    # 命令码长度（字节）
    CMD_ID_LENGTH: int = 2

    # 帧尾CRC16长度（字节）
    FRAME_TAIL_LENGTH: int = 2

    # 帧头中各字段的偏移位置
    SOF_OFFSET: int = 0
    DATA_LENGTH_OFFSET: int = 1
    SEQ_OFFSET: int = 3
    CRC8_OFFSET: int = 4

    # 命令码在帧中的偏移位置
    CMD_ID_OFFSET: int = 5

    # 数据段在帧中的偏移位置
    DATA_OFFSET: int = 7

    # 最大帧大小
    MAX_FRAME_SIZE: int = 512

    # FIFO缓冲区大小
    FIFO_BUF_LENGTH: int = 1024

    @staticmethod
    def get_frame_size(data_length: int) -> int:
        """
        计算完整帧的大小

        Args:
            data_length: 数据段长度

        Returns:
            int: 完整帧的字节数
        """
        return (FrameConstants.FRAME_HEADER_LENGTH +
                FrameConstants.CMD_ID_LENGTH +
                data_length +
                FrameConstants.FRAME_TAIL_LENGTH)


class SerialConfig:
    """
    串口配置常量

    定义了裁判系统使用的串口配置参数。

    常规链路：
        - 波特率：115200
        - 数据位：8位
        - 停止位：1位
        - 校验位：无
        - 流控：无

    图传链路：
        - 波特率：921600
        - 其他参数同常规链路

    雷达无线链路：
        - 通过电磁波接收，非串口通信
    """

    # 常规链路波特率
    NORMAL_BAUDRATE: int = 115200

    # 图传链路波特率
    VIDEO_TRANSMISSION_BAUDRATE: int = 921600

    # 通用串口参数
    BYTESIZE: int = 8      # 数据位
    PARITY: str = 'N'      # 无校验
    STOPBITS: int = 1      # 停止位
    TIMEOUT: float = 0.1   # 读超时（秒）


class CommandID(IntEnum):
    """
    命令码ID枚举

    定义了所有命令码的ID值，按照官方协议文档表1-4。

    命名规则：
        - 常规链路命令以 0x0xxx 表示
        - 图传链路命令以 0x0xxx 表示（但通过图传链路传输）
        - 雷达无线链路命令以 0x0Axx 表示

    数据链路分类：
        - 常规链路：服务器→机器人，通过电源管理模块User串口
        - 图传链路：机器人↔自定义控制器/客户端，通过图传模块串口
        - 雷达无线链路：信号发射源→雷达，通过电磁波接收
    """

    # ==================== 常规链路数据（服务器→机器人）====================

    # 比赛状态数据（1Hz）
    GAME_STATUS: int = 0x0001

    # 比赛结果数据（比赛结束触发）
    GAME_RESULT: int = 0x0002

    # 机器人血量数据（3Hz）
    ROBOT_HP: int = 0x0003

    # 场地事件数据（1Hz）
    FIELD_EVENT: int = 0x0101

    # 裁判警告数据（1Hz或判罚触发）
    REFEREE_WARNING: int = 0x0104

    # 飞镖发射相关数据（1Hz）
    DART_LAUNCH_DATA: int = 0x0105

    # 机器人性能体系数据（10Hz）
    ROBOT_PERFORMANCE: int = 0x0201

    # 实时底盘缓冲能量和射击热量数据（10Hz）
    ROBOT_HEAT: int = 0x0202

    # 机器人位置数据（1Hz）
    ROBOT_POSITION: int = 0x0203

    # 机器人增益和底盘能量数据（3Hz）
    ROBOT_BUFF: int = 0x0204

    # 伤害状态数据（伤害发生触发）
    DAMAGE_STATE: int = 0x0206

    # 实时射击数据（弹丸发射触发）
    SHOOT_DATA: int = 0x0207

    # 允许发弹量（10Hz）
    ALLOWED_SHOOT: int = 0x0208

    # 机器人RFID模块状态（3Hz）
    RFID_STATUS: int = 0x0209

    # 飞镖选手端指令数据（3Hz）
    DART_OPERATOR_CMD: int = 0x020A

    # 地面机器人位置数据（1Hz，发送给哨兵）
    GROUND_ROBOT_POSITION: int = 0x020B

    # 雷达标记进度数据（1Hz，发送给雷达）
    RADAR_MARK_PROGRESS: int = 0x020C

    # 哨兵自主决策信息同步（1Hz）
    SENTRY_DECISION_SYNC: int = 0x020D

    # 雷达自主决策信息同步（1Hz）
    RADAR_DECISION_SYNC: int = 0x020E

    # 机器人交互数据（30Hz上限）
    ROBOT_INTERACTION: int = 0x0301

    # 选手端小地图交互数据（触发发送）
    MAP_CLICK_DATA: int = 0x0303

    # 选手端小地图接收雷达数据（5Hz上限）
    MAP_RADAR_DATA: int = 0x0305

    # 选手端小地图接收路径数据（1Hz上限）
    MAP_PATH_DATA: int = 0x0307

    # 选手端小地图接收机器人数据（3Hz上限）
    MAP_ROBOT_DATA: int = 0x0308

    # ==================== 图传链路数据 ====================

    # 自定义控制器与机器人交互数据（30Hz上限）
    CUSTOM_CONTROLLER_TO_ROBOT: int = 0x0302

    # 自定义控制器接收机器人数据（10Hz上限）
    ROBOT_TO_CUSTOM_CONTROLLER: int = 0x0309

    # 机器人发送给自定义客户端的数据（50Hz上限）
    ROBOT_TO_CUSTOM_CLIENT: int = 0x0310

    # 自定义客户端发送给机器人的自定义指令（75Hz上限）
    CUSTOM_CLIENT_TO_ROBOT: int = 0x0311

    # ==================== 非链路数据 ====================

    # 自定义控制器与选手端交互数据
    CUSTOM_CONTROLLER_TO_OPERATOR: int = 0x0306

    # ==================== 雷达无线链路数据（信号发射源→雷达）====================

    # 对方机器人的位置坐标（10Hz上限）
    ENEMY_POSITION: int = 0x0A01

    # 对方机器人的血量信息（10Hz上限）
    ENEMY_HP: int = 0x0A02

    # 对方机器人的剩余发弹量信息（10Hz上限）
    ENEMY_AMMO: int = 0x0A03

    # 对方队伍的宏观状态信息（10Hz上限）
    ENEMY_TEAM_STATUS: int = 0x0A04

    # 对方各机器人当前增益效果（10Hz上限）
    ENEMY_BUFF: int = 0x0A05

    # 对方干扰波密钥（10Hz上限）
    ENEMY_JAMMING_KEY: int = 0x0A06


class DataLength:
    """
    数据段长度常量

    定义了各命令码对应的数据段长度（字节数）。
    严格按照官方协议文档表1-4定义。
    """

    # 常规链路数据长度
    GAME_STATUS: int = 11
    GAME_RESULT: int = 1
    ROBOT_HP: int = 16
    FIELD_EVENT: int = 4
    REFEREE_WARNING: int = 3
    DART_LAUNCH_DATA: int = 3
    ROBOT_PERFORMANCE: int = 13
    ROBOT_HEAT: int = 14
    ROBOT_POSITION: int = 16
    ROBOT_BUFF: int = 8
    DAMAGE_STATE: int = 1
    SHOOT_DATA: int = 7
    ALLOWED_SHOOT: int = 6
    RFID_STATUS: int = 5
    DART_OPERATOR_CMD: int = 6
    GROUND_ROBOT_POSITION: int = 40
    RADAR_MARK_PROGRESS: int = 2
    SENTRY_DECISION_SYNC: int = 6
    RADAR_DECISION_SYNC: int = 1
    ROBOT_INTERACTION: int = 118
    MAP_CLICK_DATA: int = 15
    MAP_RADAR_DATA: int = 24
    MAP_PATH_DATA: int = 103
    MAP_ROBOT_DATA: int = 34

    # 图传链路数据长度
    CUSTOM_CONTROLLER_TO_ROBOT: int = 30
    ROBOT_TO_CUSTOM_CONTROLLER: int = 30
    ROBOT_TO_CUSTOM_CLIENT: int = 300
    CUSTOM_CLIENT_TO_ROBOT: int = 30

    # 非链路数据长度
    CUSTOM_CONTROLLER_TO_OPERATOR: int = 8

    # 雷达无线链路数据长度
    ENEMY_POSITION: int = 24
    ENEMY_HP: int = 12
    ENEMY_AMMO: int = 10
    ENEMY_TEAM_STATUS: int = 8
    ENEMY_BUFF: int = 36
    ENEMY_JAMMING_KEY: int = 6


class GameType(IntEnum):
    """
    比赛类型枚举

    定义了不同类型的比赛。
    用于解析0x0001命令码中的比赛类型字段。
    """

    ROBO_MASTER: int = 1          # RoboMaster机甲大师超级对抗赛
    ICRA_RM: int = 2              # RoboMaster机甲大师高校单项赛
    ICRA_ROBO_MASTER: int = 3     # ICRA RoboMaster高校人工智能挑战赛
    RMUL_3V3: int = 4             # RoboMaster机甲大师高校联盟赛3V3对抗
    RMUL_INFANTRY: int = 5        # RoboMaster机甲大师高校联盟赛步兵对抗


class GameStage(IntEnum):
    """
    比赛阶段枚举

    定义了比赛的不同阶段。
    用于解析0x0001命令码中的比赛阶段字段。
    """

    NOT_STARTED: int = 0          # 未开始比赛
    PREPARING: int = 1            # 准备阶段
    SELF_CHECK: int = 2           # 十五秒裁判系统自检阶段
    COUNTDOWN: int = 3            # 五秒倒计时
    IN_GAME: int = 4              # 比赛中
    SETTLING: int = 5             # 比赛结算中


class GameResultType(IntEnum):
    """
    比赛结果枚举

    定义了比赛的可能结果。
    用于解析0x0002命令码。
    """

    DRAW: int = 0          # 平局
    RED_WIN: int = 1       # 红方胜利
    BLUE_WIN: int = 2      # 蓝方胜利


class PenaltyLevel(IntEnum):
    """
    判罚等级枚举

    定义了不同的判罚等级。
    用于解析0x0104命令码中的判罚等级字段。
    """

    DOUBLE_YELLOW: int = 1     # 双方黄牌
    YELLOW: int = 2            # 黄牌
    RED: int = 3               # 红牌
    LOSE: int = 4              # 判负


class DamageType(IntEnum):
    """
    伤害类型枚举

    定义了机器人受到伤害的不同类型。
    用于解析0x0206命令码中的伤害类型字段。
    """

    PROJECTILE: int = 0        # 装甲模块被弹丸攻击导致扣血
    MODULE_OFFLINE: int = 1    # 装甲模块或超级电容管理模块离线导致扣血
    IMPACT: int = 5            # 装甲模块受到撞击导致扣血


class ProjectileType(IntEnum):
    """
    弹丸类型枚举

    定义了不同口径的弹丸类型。
    用于解析0x0207命令码中的弹丸类型字段。
    """

    AMMO_17MM: int = 1         # 17mm弹丸
    AMMO_42MM: int = 2         # 42mm弹丸


class ShooterID(IntEnum):
    """
    发射机构ID枚举

    定义了不同发射机构的ID。
    用于解析0x0207命令码中的发射机构ID字段。
    """

    SHOOTER_17MM: int = 1      # 17mm发射机构
    RESERVED: int = 2          # 保留位
    SHOOTER_42MM: int = 3      # 42mm发射机构


class DartTarget(IntEnum):
    """
    飞镖目标枚举

    定义了飞镖可以击中的目标类型。
    用于解析0x0105命令码中的目标字段。
    """

    NONE: int = 0                  # 未选定/选定前哨站
    OUTPOST: int = 1               # 击中前哨站
    BASE_FIXED: int = 2            # 击中基地固定目标
    BASE_RANDOM_FIXED: int = 3     # 击中基地随机固定目标
    BASE_RANDOM_MOVING: int = 4    # 击中基地随机移动目标
    BASE_END_MOVING: int = 5       # 击中基地末端移动目标


class SentryPosture(IntEnum):
    """
    哨兵姿态枚举

    定义了哨兵机器人的不同姿态。
    用于解析0x020D命令码中的姿态字段。
    """

    ATTACK: int = 1      # 进攻姿态
    DEFENSE: int = 2     # 防御姿态
    MOVE: int = 3        # 移动姿态


class RobotID(IntEnum):
    """
    机器人ID枚举

    定义了所有机器人的ID编号。
    用于识别数据来源或目标的机器人。

    红方机器人：1-11
    蓝方机器人：101-111
    """

    # 红方机器人
    RED_HERO: int = 1           # 红方英雄机器人
    RED_ENGINEER: int = 2       # 红方工程机器人
    RED_INFANTRY_3: int = 3     # 红方3号步兵机器人
    RED_INFANTRY_4: int = 4     # 红方4号步兵机器人
    RED_INFANTRY_5: int = 5     # 红方5号步兵机器人（保留）
    RED_AERIAL: int = 6         # 红方空中机器人
    RED_SENTRY: int = 7         # 红方哨兵机器人
    RED_DART: int = 8           # 红方飞镖
    RED_RADAR: int = 9          # 红方雷达
    RED_OUTPOST: int = 10       # 红方前哨站
    RED_BASE: int = 11          # 红方基地

    # 蓝方机器人
    BLUE_HERO: int = 101        # 蓝方英雄机器人
    BLUE_ENGINEER: int = 102    # 蓝方工程机器人
    BLUE_INFANTRY_3: int = 103  # 蓝方3号步兵机器人
    BLUE_INFANTRY_4: int = 104  # 蓝方4号步兵机器人
    BLUE_INFANTRY_5: int = 105  # 蓝方5号步兵机器人（保留）
    BLUE_AERIAL: int = 106      # 蓝方空中机器人
    BLUE_SENTRY: int = 107      # 蓝方哨兵机器人
    BLUE_DART: int = 108        # 蓝方飞镖
    BLUE_RADAR: int = 109       # 蓝方雷达
    BLUE_OUTPOST: int = 110     # 蓝方前哨站
    BLUE_BASE: int = 111        # 蓝方基地


class OperatorClientID(IntEnum):
    """
    选手端ID枚举

    定义了所有选手端的ID编号。
    用于识别选手端数据来源或目标。

    红方选手端：0x0101-0x0106
    蓝方选手端：0x0165-0x016A
    """

    # 红方选手端
    RED_HERO: int = 0x0101          # 红方英雄机器人选手端
    RED_ENGINEER: int = 0x0102      # 红方工程机器人选手端
    RED_INFANTRY_3: int = 0x0103    # 红方3号步兵机器人选手端
    RED_INFANTRY_4: int = 0x0104    # 红方4号步兵机器人选手端
    RED_INFANTRY_5: int = 0x0105    # 红方5号步兵机器人选手端
    RED_AERIAL: int = 0x0106        # 红方空中机器人选手端

    # 蓝方选手端
    BLUE_HERO: int = 0x0165         # 蓝方英雄机器人选手端
    BLUE_ENGINEER: int = 0x0166     # 蓝方工程机器人选手端
    BLUE_INFANTRY_3: int = 0x0167   # 蓝方3号步兵机器人选手端
    BLUE_INFANTRY_4: int = 0x0168   # 蓝方4号步兵机器人选手端
    BLUE_INFANTRY_5: int = 0x0169   # 蓝方5号步兵机器人选手端
    BLUE_AERIAL: int = 0x016A       # 蓝方空中机器人选手端

    # 裁判系统服务器（用于哨兵和雷达自主决策指令）
    SERVER: int = 0x8080


class UnpackStep(IntEnum):
    """
    解包步骤枚举

    定义了协议解包过程中的各个步骤。
    用于状态机解包算法。
    """

    STEP_HEADER_SOF = 0      # 等待帧起始字节
    STEP_LENGTH_LOW = 1      # 读取数据长度低字节
    STEP_LENGTH_HIGH = 2     # 读取数据长度高字节
    STEP_FRAME_SEQ = 3       # 读取帧序号
    STEP_HEADER_CRC8 = 4     # 读取帧头CRC8
    STEP_DATA_CRC16 = 5      # 读取数据和CRC16


# 命令码ID到数据长度的映射
CMD_ID_TO_DATA_LENGTH: Dict[int, int] = {
    CommandID.GAME_STATUS: DataLength.GAME_STATUS,
    CommandID.GAME_RESULT: DataLength.GAME_RESULT,
    CommandID.ROBOT_HP: DataLength.ROBOT_HP,
    CommandID.FIELD_EVENT: DataLength.FIELD_EVENT,
    CommandID.REFEREE_WARNING: DataLength.REFEREE_WARNING,
    CommandID.DART_LAUNCH_DATA: DataLength.DART_LAUNCH_DATA,
    CommandID.ROBOT_PERFORMANCE: DataLength.ROBOT_PERFORMANCE,
    CommandID.ROBOT_HEAT: DataLength.ROBOT_HEAT,
    CommandID.ROBOT_POSITION: DataLength.ROBOT_POSITION,
    CommandID.ROBOT_BUFF: DataLength.ROBOT_BUFF,
    CommandID.DAMAGE_STATE: DataLength.DAMAGE_STATE,
    CommandID.SHOOT_DATA: DataLength.SHOOT_DATA,
    CommandID.ALLOWED_SHOOT: DataLength.ALLOWED_SHOOT,
    CommandID.RFID_STATUS: DataLength.RFID_STATUS,
    CommandID.DART_OPERATOR_CMD: DataLength.DART_OPERATOR_CMD,
    CommandID.GROUND_ROBOT_POSITION: DataLength.GROUND_ROBOT_POSITION,
    CommandID.RADAR_MARK_PROGRESS: DataLength.RADAR_MARK_PROGRESS,
    CommandID.SENTRY_DECISION_SYNC: DataLength.SENTRY_DECISION_SYNC,
    CommandID.RADAR_DECISION_SYNC: DataLength.RADAR_DECISION_SYNC,
    CommandID.ROBOT_INTERACTION: DataLength.ROBOT_INTERACTION,
    CommandID.MAP_CLICK_DATA: DataLength.MAP_CLICK_DATA,
    CommandID.MAP_RADAR_DATA: DataLength.MAP_RADAR_DATA,
    CommandID.MAP_PATH_DATA: DataLength.MAP_PATH_DATA,
    CommandID.MAP_ROBOT_DATA: DataLength.MAP_ROBOT_DATA,
    CommandID.CUSTOM_CONTROLLER_TO_ROBOT: DataLength.CUSTOM_CONTROLLER_TO_ROBOT,
    CommandID.ROBOT_TO_CUSTOM_CONTROLLER: DataLength.ROBOT_TO_CUSTOM_CONTROLLER,
    CommandID.ROBOT_TO_CUSTOM_CLIENT: DataLength.ROBOT_TO_CUSTOM_CLIENT,
    CommandID.CUSTOM_CLIENT_TO_ROBOT: DataLength.CUSTOM_CLIENT_TO_ROBOT,
    CommandID.CUSTOM_CONTROLLER_TO_OPERATOR: DataLength.CUSTOM_CONTROLLER_TO_OPERATOR,
    CommandID.ENEMY_POSITION: DataLength.ENEMY_POSITION,
    CommandID.ENEMY_HP: DataLength.ENEMY_HP,
    CommandID.ENEMY_AMMO: DataLength.ENEMY_AMMO,
    CommandID.ENEMY_TEAM_STATUS: DataLength.ENEMY_TEAM_STATUS,
    CommandID.ENEMY_BUFF: DataLength.ENEMY_BUFF,
    CommandID.ENEMY_JAMMING_KEY: DataLength.ENEMY_JAMMING_KEY,
}
