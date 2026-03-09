"""
数据类型定义模块

本模块定义了C语言协议数据结构的Python等价类。
每个数据类对应协议文档中的一个命令码数据格式。

使用Python dataclass实现，提供：
- 自动生成__init__方法
- 自动生成__repr__方法
- 类型注解支持

协议版本：V1.2.0
"""

from dataclasses import dataclass
from typing import List, Optional
import struct


@dataclass
class GameStatus:
    """
    比赛状态数据 (命令码: 0x0001)

    包含比赛的当前状态信息，以1Hz频率发送。

    Attributes:
        game_type: 比赛类型 (1-5)
            1: RoboMaster机甲大师超级对抗赛
            2: RoboMaster机甲大师高校单项赛
            3: ICRA RoboMaster高校人工智能挑战赛
            4: RoboMaster机甲大师高校联盟赛3V3对抗
            5: RoboMaster机甲大师高校联盟赛步兵对抗
        game_progress: 当前比赛阶段 (0-5)
            0: 未开始比赛
            1: 准备阶段
            2: 十五秒裁判系统自检阶段
            3: 五秒倒计时
            4: 比赛中
            5: 比赛结算中
        stage_remain_time: 当前阶段剩余时间（秒）
        sync_timestamp: UNIX时间戳
    """
    game_type: int = 0
    game_progress: int = 0
    stage_remain_time: int = 0
    sync_timestamp: int = 0


@dataclass
class GameResult:
    """
    比赛结果数据 (命令码: 0x0002)

    比赛结束时触发发送。

    Attributes:
        winner: 比赛结果
            0: 平局
            1: 红方胜利
            2: 蓝方胜利
    """
    winner: int = 0


@dataclass
class RobotHP:
    """
    机器人血量数据 (命令码: 0x0003)

    包含双方所有机器人的血量信息，以3Hz频率发送。

    Attributes:
        red_1_robot_hp: 己方1号英雄机器人血量
        red_2_robot_hp: 己方2号工程机器人血量
        red_3_robot_hp: 己方3号步兵机器人血量
        red_4_robot_hp: 己方4号步兵机器人血量
        red_7_robot_hp: 己方7号哨兵机器人血量
        red_outpost_hp: 己方前哨站血量
        red_base_hp: 己方基地血量
        blue_1_robot_hp: 对方1号英雄机器人血量
        blue_2_robot_hp: 对方2号工程机器人血量
        blue_3_robot_hp: 对方3号步兵机器人血量
        blue_4_robot_hp: 对方4号步兵机器人血量
        blue_7_robot_hp: 对方7号哨兵机器人血量
        blue_outpost_hp: 对方前哨站血量
        blue_base_hp: 对方基地血量
    """
    red_1_robot_hp: int = 0
    red_2_robot_hp: int = 0
    red_3_robot_hp: int = 0
    red_4_robot_hp: int = 0
    red_7_robot_hp: int = 0
    red_outpost_hp: int = 0
    red_base_hp: int = 0
    blue_1_robot_hp: int = 0
    blue_2_robot_hp: int = 0
    blue_3_robot_hp: int = 0
    blue_4_robot_hp: int = 0
    blue_7_robot_hp: int = 0
    blue_outpost_hp: int = 0
    blue_base_hp: int = 0


@dataclass
class FieldEvent:
    """
    场地事件数据 (命令码: 0x0101)

    包含场地相关的事件和状态信息，以1Hz频率发送。

    Attributes:
        supply_area_1: 己方与资源区不重叠的补给区占领状态
        supply_area_2: 己方与资源区重叠的补给区占领状态
        rmul_supply_area: 己方补给区的占领状态（仅RMUL适用）
        small_energy_mech: 己方小能量机关的激活状态 (0-2)
        big_energy_mech: 己方大能量机关的激活状态 (0-2)
        central_highland: 己方中央高地的占领状态
        trapezoid_highland: 己方梯形高地的占领状态
        dart_last_hit_time: 对方飞镖最后一次击中己方前哨站或基地的时间
        dart_last_hit_target: 对方飞镖最后一次击中的目标
        center_buff_point: 中心增益点的占领状态（仅RMUL适用）
        fortress_buff_point: 己方堡垒增益点的占领状态
        outpost_buff_point: 己方前哨站增益点的占领状态
        base_buff_point: 己方基地增益点的占领状态
    """
    supply_area_1: bool = False
    supply_area_2: bool = False
    rmul_supply_area: bool = False
    small_energy_mech: int = 0
    big_energy_mech: int = 0
    central_highland: int = 0
    trapezoid_highland: bool = False
    dart_last_hit_time: int = 0
    dart_last_hit_target: int = 0
    center_buff_point: int = 0
    fortress_buff_point: int = 0
    outpost_buff_point: int = 0
    base_buff_point: bool = False


@dataclass
class RefereeWarning:
    """
    裁判警告数据 (命令码: 0x0104)

    己方判罚/判负时触发发送，其余时间以1Hz频率发送。

    Attributes:
        penalty_level: 己方最后一次受到判罚的等级
            1: 双方黄牌
            2: 黄牌
            3: 红牌
            4: 判负
       犯规_robot_id: 己方最后一次受到判罚的违规机器人ID
        foul_count: 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数
    """
    penalty_level: int = 0
    foul_robot_id: int = 0
    foul_count: int = 0


@dataclass
class DartLaunchData:
    """
    飞镖发射相关数据 (命令码: 0x0105)

    以1Hz频率发送。

    Attributes:
        dart_remaining_time: 己方飞镖发射剩余时间（秒）
        last_hit_target: 最近一次己方飞镖击中的目标
        target_hit_count: 对方最近被击中的目标累计被击中次数
        selected_target: 飞镖此时选定的击打目标
    """
    dart_remaining_time: int = 0
    last_hit_target: int = 0
    target_hit_count: int = 0
    selected_target: int = 0


@dataclass
class RobotPerformance:
    """
    机器人性能体系数据 (命令码: 0x0201)

    以10Hz频率发送，包含机器人当前性能信息。

    Attributes:
        robot_id: 本机器人ID
        robot_level: 机器人等级
        current_hp: 机器人当前血量
        maximum_hp: 机器人血量上限
        shooter_barrel_cooling_value: 机器人射击热量每秒冷却值
        shooter_barrel_heat_limit: 机器人射击热量上限
        chassis_power_limit: 机器人底盘功率上限
        power_management_output: 电源管理模块的输出情况
    """
    robot_id: int = 0
    robot_level: int = 0
    current_hp: int = 0
    maximum_hp: int = 0
    shooter_barrel_cooling_value: int = 0
    shooter_barrel_heat_limit: int = 0
    chassis_power_limit: int = 0
    power_management_gimbal_output: bool = False
    power_management_chassis_output: bool = False
    power_management_shooter_output: bool = False


@dataclass
class RobotHeat:
    """
    实时底盘缓冲能量和射击热量数据 (命令码: 0x0202)

    以10Hz频率发送。

    Attributes:
        chassis_current_voltage: 底盘当前电压（保留）
        chassis_current_current: 底盘当前电流（保留）
        chassis_current_power: 底盘当前功率（保留）
        buffer_energy: 缓冲能量（单位：J）
        shooter_17mm_barrel_heat: 17mm发射机构的射击热量
        shooter_42mm_barrel_heat: 42mm发射机构的射击热量
    """
    chassis_current_voltage: int = 0
    chassis_current_current: int = 0
    chassis_current_power: float = 0.0
    buffer_energy: int = 0
    shooter_17mm_barrel_heat: int = 0
    shooter_42mm_barrel_heat: int = 0


@dataclass
class RobotPosition:
    """
    机器人位置数据 (命令码: 0x0203)

    以1Hz频率发送。

    Attributes:
        x: 本机器人位置x坐标（单位：m）
        y: 本机器人位置y坐标（单位：m）
        angle: 本机器人测速模块的朝向（单位：度，正北为0度）
    """
    x: float = 0.0
    y: float = 0.0
    angle: float = 0.0


@dataclass
class RobotBuff:
    """
    机器人增益和底盘能量数据 (命令码: 0x0204)

    以3Hz频率发送。

    Attributes:
        recovery_buff: 机器人回血增益（百分比）
        cooling_buff: 机器人射击热量冷却倍率
        defence_buff: 机器人防御增益（百分比）
        vulnerability_buff: 机器人负防御增益（百分比）
        attack_buff: 机器人攻击增益（百分比）
        remaining_energy: 机器人剩余能量值反馈
    """
    recovery_buff: int = 0
    cooling_buff: int = 0
    defence_buff: int = 0
    vulnerability_buff: int = 0
    attack_buff: int = 0
    remaining_energy: int = 0


@dataclass
class DamageState:
    """
    伤害状态数据 (命令码: 0x0206)

    伤害发生后发送。

    Attributes:
        armor_id: 装甲模块或测速模块的ID编号
        damage_type: 血量变化类型
            0: 装甲模块被弹丸攻击导致扣血
            1: 装甲模块或超级电容管理模块离线导致扣血
            5: 装甲模块受到撞击导致扣血
    """
    armor_id: int = 0
    damage_type: int = 0


@dataclass
class ShootData:
    """
    实时射击数据 (命令码: 0x0207)

    弹丸发射后发送。

    Attributes:
        bullet_type: 弹丸类型 (1: 17mm, 2: 42mm)
        shooter_id: 发射机构ID (1: 17mm发射机构, 3: 42mm发射机构)
        launching_frequency: 弹丸射速（单位：Hz）
        initial_speed: 弹丸初速度（单位：m/s）
    """
    bullet_type: int = 0
    shooter_id: int = 0
    launching_frequency: int = 0
    initial_speed: float = 0.0


@dataclass
class AllowedShoot:
    """
    允许发弹量 (命令码: 0x0208)

    以10Hz频率发送。

    Attributes:
        projectile_allowance_17mm: 17mm弹丸允许发弹量
        projectile_allowance_42mm: 42mm弹丸允许发弹量
        remaining_gold_coin: 剩余金币数量
        fortress_reserve_17mm: 堡垒增益点提供的储备17mm弹丸允许发弹量
    """
    projectile_allowance_17mm: int = 0
    projectile_allowance_42mm: int = 0
    remaining_gold_coin: int = 0
    fortress_reserve_17mm: int = 0


@dataclass
class RFIDStatus:
    """
    机器人RFID模块状态 (命令码: 0x0209)

    以3Hz频率发送。

    Attributes:
        detected_rfid_bits: 检测到的RFID卡位掩码（32位 + 8位）
    """
    detected_rfid_bits_low: int = 0  # 低32位
    detected_rfid_bits_high: int = 0  # 高8位


@dataclass
class DartOperatorCmd:
    """
    飞镖选手端指令数据 (命令码: 0x020A)

    以3Hz频率发送。

    Attributes:
        dart_station_status: 当前飞镖发射站的状态
            1: 关闭
            2: 正在开启或者关闭中
            0: 已经开启
        target_switch_time: 切换击打目标时的比赛剩余时间（秒）
        last_launch_time: 最后一次操作手确定发射指令时的比赛剩余时间（秒）
    """
    dart_station_status: int = 0
    target_switch_time: int = 0
    last_launch_time: int = 0


@dataclass
class GroundRobotPosition:
    """
    地面机器人位置数据 (命令码: 0x020B)

    以1Hz频率发送给哨兵机器人。

    Attributes:
        hero_x: 己方英雄机器人位置x轴坐标（单位：m）
        hero_y: 己方英雄机器人位置y轴坐标（单位：m）
        engineer_x: 己方工程机器人位置x轴坐标（单位：m）
        engineer_y: 己方工程机器人位置y轴坐标（单位：m）
        infantry_3_x: 己方3号步兵机器人位置x轴坐标（单位：m）
        infantry_3_y: 己方3号步兵机器人位置y轴坐标（单位：m）
        infantry_4_x: 己方4号步兵机器人位置x轴坐标（单位：m）
        infantry_4_y: 己方4号步兵机器人位置y轴坐标（单位：m）
    """
    hero_x: float = 0.0
    hero_y: float = 0.0
    engineer_x: float = 0.0
    engineer_y: float = 0.0
    infantry_3_x: float = 0.0
    infantry_3_y: float = 0.0
    infantry_4_x: float = 0.0
    infantry_4_y: float = 0.0


@dataclass
class RadarMarkProgress:
    """
    雷达标记进度数据 (命令码: 0x020C)

    以1Hz频率发送给雷达机器人。

    Attributes:
        enemy_hero_marked: 对方1号英雄机器人易伤情况
        enemy_engineer_marked: 对方2号工程机器人易伤情况
        enemy_infantry_3_marked: 对方3号步兵机器人易伤情况
        enemy_infantry_4_marked: 对方4号步兵机器人易伤情况
        enemy_aerial_marked: 对方空中机器人特殊标识情况
        enemy_sentry_marked: 对方哨兵机器人易伤情况
        ally_hero_marked: 己方1号英雄机器人特殊标识情况
        ally_engineer_marked: 己方2号工程机器人特殊标识情况
        ally_infantry_3_marked: 己方3号步兵机器人特殊标识情况
        ally_infantry_4_marked: 己方4号步兵机器人特殊标识情况
        ally_aerial_marked: 己方空中机器人特殊标识情况
        ally_sentry_marked: 己方哨兵机器人特殊标识情况
    """
    enemy_hero_marked: bool = False
    enemy_engineer_marked: bool = False
    enemy_infantry_3_marked: bool = False
    enemy_infantry_4_marked: bool = False
    enemy_aerial_marked: bool = False
    enemy_sentry_marked: bool = False
    ally_hero_marked: bool = False
    ally_engineer_marked: bool = False
    ally_infantry_3_marked: bool = False
    ally_infantry_4_marked: bool = False
    ally_aerial_marked: bool = False
    ally_sentry_marked: bool = False


@dataclass
class SentryDecisionSync:
    """
    哨兵自主决策信息同步 (命令码: 0x020D)

    以1Hz频率发送给哨兵机器人。

    Attributes:
        exchanged_ammo: 哨兵机器人成功兑换的允许发弹量
        remote_ammo_count: 哨兵机器人成功远程兑换允许发弹量的次数
        remote_hp_count: 哨兵机器人成功远程兑换血量的次数
        can_free_revive: 哨兵机器人当前是否可以确认免费复活
        can_buy_revive: 哨兵机器人当前是否可以兑换立即复活
        revive_cost: 哨兵机器人当前若兑换立即复活需要花费的金币数
        in_disengage: 哨兵当前是否处于脱战状态
        remaining_ammo_exchange: 队伍17mm允许发弹量的剩余可兑换数
        sentry_posture: 哨兵当前姿态 (1: 进攻, 2: 防御, 3: 移动)
        can_activate_rune: 己方能量机关是否能够进入正在激活状态
    """
    exchanged_ammo: int = 0
    remote_ammo_count: int = 0
    remote_hp_count: int = 0
    can_free_revive: bool = False
    can_buy_revive: bool = False
    revive_cost: int = 0
    in_disengage: bool = False
    remaining_ammo_exchange: int = 0
    sentry_posture: int = 3
    can_activate_rune: bool = False


@dataclass
class RadarDecisionSync:
    """
    雷达自主决策信息同步 (命令码: 0x020E)

    以1Hz频率发送给雷达机器人。

    Attributes:
        double_vulnerability_chance: 雷达是否拥有触发双倍易伤的机会
        enemy_double_vulnerability: 对方是否正在被触发双倍易伤
        encryption_level: 己方加密等级（即对方干扰波难度等级）
        can_modify_key: 当前是否可以修改密钥
    """
    double_vulnerability_chance: int = 0
    enemy_double_vulnerability: bool = False
    encryption_level: int = 1
    can_modify_key: bool = False


@dataclass
class MapClickData:
    """
    选手端小地图交互数据 (命令码: 0x0303)

    选手端触发发送。

    Attributes:
        target_x: 目标位置x轴坐标（单位：m）
        target_y: 目标位置y轴坐标（单位：m）
        keyboard_key: 云台手按下的键盘按键通用键值
        target_robot_id: 对方机器人ID
        source_id: 信息来源ID
    """
    target_x: float = 0.0
    target_y: float = 0.0
    keyboard_key: int = 0
    target_robot_id: int = 0
    source_id: int = 0


@dataclass
class MapRadarData:
    """
    选手端小地图接收雷达数据 (命令码: 0x0305)

    频率上限为5Hz。

    Attributes:
        target_robot_id: 目标机器人ID
        target_x: 目标位置x坐标（单位：cm）
        target_y: 目标位置y坐标（单位：cm）
        特殊标识状态: 是否特殊标识
    """
    # 注意：实际数据包含多个机器人的位置信息
    # 这里简化为单个机器人
    hero_x: int = 0
    hero_y: int = 0
    engineer_x: int = 0
    engineer_y: int = 0
    infantry_3_x: int = 0
    infantry_3_y: int = 0
    infantry_4_x: int = 0
    infantry_4_y: int = 0
    sentry_x: int = 0
    sentry_y: int = 0


@dataclass
class MapPathData:
    """
    选手端小地图接收路径数据 (命令码: 0x0307)

    频率上限为1Hz。

    Attributes:
        intention: 哨兵意图 (1: 攻击, 2: 防守, 3: 移动)
        start_x: 路径起点x轴坐标（单位：dm）
        start_y: 路径起点y轴坐标（单位：dm）
        delta_x: 路径点x轴增量数组（49个元素）
        delta_y: 路径点y轴增量数组（49个元素）
        sender_id: 发送者ID
    """
    intention: int = 0
    start_x: int = 0
    start_y: int = 0
    delta_x: List[int] = None  # type: ignore
    delta_y: List[int] = None  # type: ignore
    sender_id: int = 0

    def __post_init__(self):
        if self.delta_x is None:
            self.delta_x = [0] * 49
        if self.delta_y is None:
            self.delta_y = [0] * 49


@dataclass
class MapRobotData:
    """
    选手端小地图接收机器人数据 (命令码: 0x0308)

    频率上限为3Hz。

    Attributes:
        sender_id: 发送者的ID
        receiver_id: 接收者的ID
        message: 字符消息（UTF-16编码）
    """
    sender_id: int = 0
    receiver_id: int = 0
    message: str = ""


@dataclass
class CustomControllerData:
    """
    自定义控制器数据 (命令码: 0x0302, 0x0306, 0x0309, 0x0310, 0x0311)

    用于机器人与自定义控制器/客户端之间的数据交互。

    Attributes:
        data: 原始数据字节
    """
    data: bytes = b""


@dataclass
class EnemyPosition:
    """
    对方机器人的位置坐标 (命令码: 0x0A01)

    雷达无线链路数据，频率上限为10Hz。

    Attributes:
        hero_x: 对方英雄机器人位置x轴坐标（单位：cm）
        hero_y: 对方英雄机器人位置y轴坐标（单位：cm）
        engineer_x: 对方工程机器人位置x轴坐标（单位：cm）
        engineer_y: 对方工程机器人位置y轴坐标（单位：cm）
        infantry_3_x: 对方3号步兵机器人位置x轴坐标（单位：cm）
        infantry_3_y: 对方3号步兵机器人位置y轴坐标（单位：cm）
        infantry_4_x: 对方4号步兵机器人位置x轴坐标（单位：cm）
        infantry_4_y: 对方4号步兵机器人位置y轴坐标（单位：cm）
        aerial_x: 对方空中机器人位置x轴坐标（单位：cm）
        aerial_y: 对方空中机器人位置y轴坐标（单位：cm）
        sentry_x: 对方哨兵机器人位置x轴坐标（单位：cm）
        sentry_y: 对方哨兵机器人位置y轴坐标（单位：cm）
    """
    hero_x: int = 0
    hero_y: int = 0
    engineer_x: int = 0
    engineer_y: int = 0
    infantry_3_x: int = 0
    infantry_3_y: int = 0
    infantry_4_x: int = 0
    infantry_4_y: int = 0
    aerial_x: int = 0
    aerial_y: int = 0
    sentry_x: int = 0
    sentry_y: int = 0


@dataclass
class EnemyHP:
    """
    对方机器人的血量信息 (命令码: 0x0A02)

    雷达无线链路数据，频率上限为10Hz。

    Attributes:
        hero_hp: 对方1号英雄机器人血量
        engineer_hp: 对方2号工程机器人血量
        infantry_3_hp: 对方3号步兵机器人血量
        infantry_4_hp: 对方4号步兵机器人血量
        sentry_hp: 对方7号哨兵机器人血量
    """
    hero_hp: int = 0
    engineer_hp: int = 0
    infantry_3_hp: int = 0
    infantry_4_hp: int = 0
    sentry_hp: int = 0


@dataclass
class EnemyAmmo:
    """
    对方机器人的剩余发弹量信息 (命令码: 0x0A03)

    雷达无线链路数据，频率上限为10Hz。

    Attributes:
        hero_ammo: 对方1号英雄机器人允许发弹量
        infantry_3_ammo: 对方3号步兵机器人允许发弹量
        infantry_4_ammo: 对方4号步兵机器人允许发弹量
        aerial_ammo: 对方6号空中机器人允许发弹量
        sentry_ammo: 对方7号哨兵机器人允许发弹量
    """
    hero_ammo: int = 0
    infantry_3_ammo: int = 0
    infantry_4_ammo: int = 0
    aerial_ammo: int = 0
    sentry_ammo: int = 0


@dataclass
class EnemyTeamStatus:
    """
    对方队伍的宏观状态信息 (命令码: 0x0A04)

    雷达无线链路数据，频率上限为10Hz。

    Attributes:
        remaining_gold: 对方剩余金币数
        total_gold: 对方累计总金币数
        supply_area_occupied: 对方补给区占领状态
        central_highland_status: 对方中央高地的占领状态
        trapezoid_highland_status: 对方梯形高地的占领状态
        fortress_status: 对方堡垒增益点的占领状态
        outpost_status: 对方前哨站增益点的占领状态
        base_status: 对方基地增益点的占领状态
        terrain_cross_status: 各地形跨越增益点状态
    """
    remaining_gold: int = 0
    total_gold: int = 0
    supply_area_occupied: bool = False
    central_highland_status: int = 0
    trapezoid_highland_status: bool = False
    fortress_status: int = 0
    outpost_status: int = 0
    base_status: bool = False
    terrain_cross_status: int = 0


@dataclass
class EnemyBuff:
    """
    对方各机器人当前增益效果 (命令码: 0x0A05)

    雷达无线链路数据，频率上限为10Hz。

    每个机器人包含：
    - recovery_buff: 回血增益
    - cooling_buff: 射击热量冷却增益
    - defence_buff: 防御增益
    - vulnerability_buff: 负防御增益
    - attack_buff: 攻击增益

    Attributes:
        hero: 英雄机器人增益
        engineer: 工程机器人增益
        infantry_3: 3号步兵机器人增益
        infantry_4: 4号步兵机器人增益
        sentry: 哨兵机器人增益及姿态
    """
    # 英雄
    hero_recovery: int = 0
    hero_cooling: int = 0
    hero_defence: int = 0
    hero_vulnerability: int = 0
    hero_attack: int = 0
    # 工程
    engineer_recovery: int = 0
    engineer_cooling: int = 0
    engineer_defence: int = 0
    engineer_vulnerability: int = 0
    engineer_attack: int = 0
    # 3号步兵
    infantry_3_recovery: int = 0
    infantry_3_cooling: int = 0
    infantry_3_defence: int = 0
    infantry_3_vulnerability: int = 0
    infantry_3_attack: int = 0
    # 4号步兵
    infantry_4_recovery: int = 0
    infantry_4_cooling: int = 0
    infantry_4_defence: int = 0
    infantry_4_vulnerability: int = 0
    infantry_4_attack: int = 0
    # 哨兵
    sentry_recovery: int = 0
    sentry_cooling: int = 0
    sentry_defence: int = 0
    sentry_vulnerability: int = 0
    sentry_attack: int = 0
    sentry_posture: int = 0


@dataclass
class EnemyJammingKey:
    """
    对方干扰波密钥 (命令码: 0x0A06)

    雷达无线链路数据，频率上限为10Hz。

    Attributes:
        key: 6字节ASCII码密钥
    """
    key: str = ""
