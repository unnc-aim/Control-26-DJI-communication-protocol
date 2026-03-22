"""
协议解析器模块

本模块实现裁判系统通信协议的数据解析功能。
使用状态机方式解包串口数据流，并将解析后的数据转换为Python数据类。

解析流程：
1. 从FIFO缓冲区读取字节流
2. 使用状态机识别帧边界
3. 验证CRC8和CRC16校验
4. 根据命令码解析数据段
5. 返回对应的数据类实例

协议版本：V1.2.0
"""

import struct
from typing import Optional, Tuple, Any
from collections import deque

from .protocol_constants import FrameConstants, CommandID, UnpackStep
from .crc_utils import CRCUtils
from .data_types import (
    GameStatus, GameResult, RobotHP, FieldEvent, RefereeWarning,
    DartLaunchData, RobotPerformance, RobotHeat, RobotPosition,
    RobotBuff, DamageState, ShootData, AllowedShoot, RFIDStatus,
    DartOperatorCmd, GroundRobotPosition, RadarMarkProgress,
    SentryDecisionSync, RadarDecisionSync, MapClickData,
    MapRadarData, MapPathData, MapRobotData, CustomControllerData,
    EnemyPosition, EnemyHP, EnemyAmmo, EnemyTeamStatus, EnemyBuff, EnemyJammingKey
)


class ProtocolParser:
    """
    协议解析器类

    实现裁判系统通信协议的解包和解析功能。

    使用状态机解析帧结构：
    1. STEP_HEADER_SOF: 等待帧起始字节(0xA5)
    2. STEP_LENGTH_LOW: 读取数据长度低字节
    3. STEP_LENGTH_HIGH: 读取数据长度高字节
    4. STEP_FRAME_SEQ: 读取帧序号
    5. STEP_HEADER_CRC8: 读取帧头CRC8校验
    6. STEP_DATA_CRC16: 读取数据段和CRC16校验

    Attributes:
        fifo_buffer: FIFO缓冲区，存储待解析的字节流
        unpack_step: 当前解包步骤
        data_len: 当前帧的数据长度
        index: 当前帧的写入索引
        protocol_packet: 当前帧的缓冲区
    """

    def __init__(self) -> None:
        """
        初始化协议解析器

        创建空的FIFO缓冲区和初始化状态机。
        """
        # FIFO缓冲区
        self.fifo_buffer: deque = deque(maxlen=FrameConstants.FIFO_BUF_LENGTH)

        # 解包状态机
        self.unpack_step: UnpackStep = UnpackStep.STEP_HEADER_SOF
        self.data_len: int = 0
        self.index: int = 0
        self.protocol_packet: bytearray = bytearray(
            FrameConstants.MAX_FRAME_SIZE)

    def feed_data(self, data: bytes) -> None:
        """
        向FIFO缓冲区添加数据

        Args:
            data: 要添加的字节流数据
        """
        for byte in data:
            self.fifo_buffer.append(byte)

    def unpack(self) -> Optional[Tuple[int, Any]]:
        """
        从FIFO缓冲区解包一帧数据

        使用状态机解析帧边界，验证校验和，并返回解析后的数据。

        Returns:
            Optional[Tuple[int, Any]]: 如果成功解析一帧，返回(命令码, 数据对象)元组；
                                       如果没有完整帧或校验失败，返回None
        """
        sof = FrameConstants.SOF

        while self.fifo_buffer:
            byte = self.fifo_buffer.popleft()

            if self.unpack_step == UnpackStep.STEP_HEADER_SOF:
                # 步骤1：等待帧起始字节
                if byte == sof:
                    self.unpack_step = UnpackStep.STEP_LENGTH_LOW
                    self.protocol_packet[self.index] = byte
                    self.index += 1
                else:
                    self.index = 0

            elif self.unpack_step == UnpackStep.STEP_LENGTH_LOW:
                # 步骤2：读取数据长度低字节
                self.data_len = byte
                self.protocol_packet[self.index] = byte
                self.index += 1
                self.unpack_step = UnpackStep.STEP_LENGTH_HIGH

            elif self.unpack_step == UnpackStep.STEP_LENGTH_HIGH:
                # 步骤3：读取数据长度高字节
                self.data_len |= (byte << 8)
                self.protocol_packet[self.index] = byte
                self.index += 1

                # 检查数据长度是否合法
                max_data_len = (FrameConstants.MAX_FRAME_SIZE -
                                FrameConstants.FRAME_HEADER_LENGTH -
                                FrameConstants.CMD_ID_LENGTH -
                                FrameConstants.FRAME_TAIL_LENGTH)
                if self.data_len < max_data_len:
                    self.unpack_step = UnpackStep.STEP_FRAME_SEQ
                else:
                    # 数据长度非法，重置状态机
                    self.unpack_step = UnpackStep.STEP_HEADER_SOF
                    self.index = 0

            elif self.unpack_step == UnpackStep.STEP_FRAME_SEQ:
                # 步骤4：读取帧序号
                self.protocol_packet[self.index] = byte
                self.index += 1
                self.unpack_step = UnpackStep.STEP_HEADER_CRC8

            elif self.unpack_step == UnpackStep.STEP_HEADER_CRC8:
                # 步骤5：读取帧头CRC8校验
                self.protocol_packet[self.index] = byte
                self.index += 1

                # 验证帧头CRC8
                if self.index == FrameConstants.FRAME_HEADER_LENGTH:
                    if CRCUtils.verify_crc8_check_sum(
                        bytes(
                            self.protocol_packet[:FrameConstants.FRAME_HEADER_LENGTH]),
                        FrameConstants.FRAME_HEADER_LENGTH
                    ):
                        self.unpack_step = UnpackStep.STEP_DATA_CRC16
                    else:
                        # CRC8校验失败，重置状态机
                        self.unpack_step = UnpackStep.STEP_HEADER_SOF
                        self.index = 0

            elif self.unpack_step == UnpackStep.STEP_DATA_CRC16:
                # 步骤6：读取数据段和CRC16校验
                total_len = (FrameConstants.FRAME_HEADER_LENGTH +
                             FrameConstants.CMD_ID_LENGTH +
                             self.data_len +
                             FrameConstants.FRAME_TAIL_LENGTH)

                if self.index < total_len:
                    self.protocol_packet[self.index] = byte
                    self.index += 1

                if self.index >= total_len:
                    # 完整帧已接收，验证CRC16
                    self.unpack_step = UnpackStep.STEP_HEADER_SOF
                    self.index = 0

                    if CRCUtils.verify_crc16_check_sum(
                        bytes(self.protocol_packet[:total_len]),
                        total_len
                    ):
                        # CRC16校验通过，解析数据
                        return self._parse_frame()

        return None

    def _parse_frame(self) -> Optional[Tuple[int, Any]]:
        """
        解析已验证的帧数据

        从当前帧中提取命令码和数据段，并根据命令码解析为对应的数据类。

        Returns:
            Optional[Tuple[int, Any]]: (命令码, 数据对象)元组，解析失败返回None
        """
        # 提取命令码（小端序）
        cmd_id = (self.protocol_packet[FrameConstants.CMD_ID_OFFSET] |
                  (self.protocol_packet[FrameConstants.CMD_ID_OFFSET + 1] << 8))

        # 提取数据段
        data_start = FrameConstants.DATA_OFFSET
        data_end = data_start + self.data_len
        data = bytes(self.protocol_packet[data_start:data_end])

        # 裁判系统在链路抖动时可能出现有效CRC但字段长度不足的帧。
        # 这里先做长度校验，避免后续struct.unpack抛异常。
        if not self._has_min_payload_length(cmd_id, len(data)):
            return None

        # 根据命令码解析数据
        parsed_data = None

        if cmd_id == CommandID.GAME_STATUS:
            parsed_data = self._parse_game_status(data)
        elif cmd_id == CommandID.GAME_RESULT:
            parsed_data = self._parse_game_result(data)
        elif cmd_id == CommandID.ROBOT_HP:
            parsed_data = self._parse_robot_hp(data)
        elif cmd_id == CommandID.FIELD_EVENT:
            parsed_data = self._parse_field_event(data)
        elif cmd_id == CommandID.REFEREE_WARNING:
            parsed_data = self._parse_referee_warning(data)
        elif cmd_id == CommandID.DART_LAUNCH_DATA:
            parsed_data = self._parse_dart_launch_data(data)
        elif cmd_id == CommandID.ROBOT_PERFORMANCE:
            parsed_data = self._parse_robot_performance(data)
        elif cmd_id == CommandID.ROBOT_HEAT:
            parsed_data = self._parse_robot_heat(data)
        elif cmd_id == CommandID.ROBOT_POSITION:
            parsed_data = self._parse_robot_position(data)
        elif cmd_id == CommandID.ROBOT_BUFF:
            parsed_data = self._parse_robot_buff(data)
        elif cmd_id == CommandID.DAMAGE_STATE:
            parsed_data = self._parse_damage_state(data)
        elif cmd_id == CommandID.SHOOT_DATA:
            parsed_data = self._parse_shoot_data(data)
        elif cmd_id == CommandID.ALLOWED_SHOOT:
            parsed_data = self._parse_allowed_shoot(data)
        elif cmd_id == CommandID.RFID_STATUS:
            parsed_data = self._parse_rfid_status(data)
        elif cmd_id == CommandID.DART_OPERATOR_CMD:
            parsed_data = self._parse_dart_operator_cmd(data)
        elif cmd_id == CommandID.GROUND_ROBOT_POSITION:
            parsed_data = self._parse_ground_robot_position(data)
        elif cmd_id == CommandID.RADAR_MARK_PROGRESS:
            parsed_data = self._parse_radar_mark_progress(data)
        elif cmd_id == CommandID.SENTRY_DECISION_SYNC:
            parsed_data = self._parse_sentry_decision_sync(data)
        elif cmd_id == CommandID.RADAR_DECISION_SYNC:
            parsed_data = self._parse_radar_decision_sync(data)
        elif cmd_id == CommandID.MAP_CLICK_DATA:
            parsed_data = self._parse_map_click_data(data)
        elif cmd_id == CommandID.MAP_RADAR_DATA:
            parsed_data = self._parse_map_radar_data(data)
        elif cmd_id == CommandID.MAP_PATH_DATA:
            parsed_data = self._parse_map_path_data(data)
        elif cmd_id == CommandID.MAP_ROBOT_DATA:
            parsed_data = self._parse_map_robot_data(data)
        elif cmd_id == CommandID.ENEMY_POSITION:
            parsed_data = self._parse_enemy_position(data)
        elif cmd_id == CommandID.ENEMY_HP:
            parsed_data = self._parse_enemy_hp(data)
        elif cmd_id == CommandID.ENEMY_AMMO:
            parsed_data = self._parse_enemy_ammo(data)
        elif cmd_id == CommandID.ENEMY_TEAM_STATUS:
            parsed_data = self._parse_enemy_team_status(data)
        elif cmd_id == CommandID.ENEMY_BUFF:
            parsed_data = self._parse_enemy_buff(data)
        elif cmd_id == CommandID.ENEMY_JAMMING_KEY:
            parsed_data = self._parse_enemy_jamming_key(data)
        else:
            # 未知命令码，返回原始数据
            parsed_data = CustomControllerData(data=data)

        return (cmd_id, parsed_data) if parsed_data else None

    def _has_min_payload_length(self, cmd_id: int, payload_len: int) -> bool:
        """检查指定命令是否满足最小载荷长度要求。"""
        min_len_by_cmd = {
            int(CommandID.GAME_STATUS): 11,
            int(CommandID.GAME_RESULT): 1,
            int(CommandID.ROBOT_HP): 32,
            int(CommandID.FIELD_EVENT): 4,
            int(CommandID.REFEREE_WARNING): 3,
            int(CommandID.DART_LAUNCH_DATA): 3,
            int(CommandID.ROBOT_PERFORMANCE): 13,
            int(CommandID.ROBOT_HEAT): 14,
            int(CommandID.ROBOT_POSITION): 12,
            int(CommandID.ROBOT_BUFF): 7,
            int(CommandID.DAMAGE_STATE): 1,
            int(CommandID.SHOOT_DATA): 7,
            int(CommandID.ALLOWED_SHOOT): 8,
            int(CommandID.RFID_STATUS): 5,
            int(CommandID.DART_OPERATOR_CMD): 6,
            int(CommandID.GROUND_ROBOT_POSITION): 40,
            int(CommandID.RADAR_MARK_PROGRESS): 2,
            int(CommandID.SENTRY_DECISION_SYNC): 6,
            int(CommandID.RADAR_DECISION_SYNC): 1,
            int(CommandID.MAP_CLICK_DATA): 13,
            int(CommandID.MAP_RADAR_DATA): 32,
            int(CommandID.MAP_PATH_DATA): 105,
            int(CommandID.MAP_ROBOT_DATA): 34,
            int(CommandID.ENEMY_POSITION): 24,
            int(CommandID.ENEMY_HP): 12,
            int(CommandID.ENEMY_AMMO): 10,
            int(CommandID.ENEMY_TEAM_STATUS): 8,
            int(CommandID.ENEMY_BUFF): 36,
            int(CommandID.ENEMY_JAMMING_KEY): 6,
        }
        required = min_len_by_cmd.get(cmd_id)
        if required is None:
            return True
        return payload_len >= required

    # ==================== 数据解析方法 ====================

    def _parse_game_status(self, data: bytes) -> GameStatus:
        """
        解析比赛状态数据 (0x0001)

        数据格式：
        - 字节0: bit0-3比赛类型, bit4-7比赛阶段
        - 字节1-2: 当前阶段剩余时间（小端序）
        - 字节3-10: UNIX时间戳（小端序）
        """
        game_type = data[0] & 0x0F
        game_progress = (data[0] >> 4) & 0x0F
        stage_remain_time = struct.unpack('<H', data[1:3])[0]
        sync_timestamp = struct.unpack('<Q', data[3:11])[0]

        return GameStatus(
            game_type=game_type,
            game_progress=game_progress,
            stage_remain_time=stage_remain_time,
            sync_timestamp=sync_timestamp
        )

    def _parse_game_result(self, data: bytes) -> GameResult:
        """解析比赛结果数据 (0x0002)"""
        return GameResult(winner=data[0])

    def _parse_robot_hp(self, data: bytes) -> RobotHP:
        """
        解析机器人血量数据 (0x0003)

        数据格式：16个uint16，分别表示红蓝双方各机器人血量
        """
        hp_values = struct.unpack('<16H', data[:32])
        return RobotHP(
            red_1_robot_hp=hp_values[0],
            red_2_robot_hp=hp_values[1],
            red_3_robot_hp=hp_values[2],
            red_4_robot_hp=hp_values[3],
            # hp_values[4] 保留
            red_7_robot_hp=hp_values[5],
            red_outpost_hp=hp_values[6],
            red_base_hp=hp_values[7],
            blue_1_robot_hp=hp_values[8],
            blue_2_robot_hp=hp_values[9],
            blue_3_robot_hp=hp_values[10],
            blue_4_robot_hp=hp_values[11],
            # hp_values[12] 保留
            blue_7_robot_hp=hp_values[13],
            blue_outpost_hp=hp_values[14],
            blue_base_hp=hp_values[15]
        )

    def _parse_field_event(self, data: bytes) -> FieldEvent:
        """解析场地事件数据 (0x0101)"""
        # 解析32位数据
        value = struct.unpack('<I', data[:4])[0]

        return FieldEvent(
            supply_area_1=bool(value & (1 << 0)),
            supply_area_2=bool(value & (1 << 1)),
            rmul_supply_area=bool(value & (1 << 2)),
            small_energy_mech=(value >> 3) & 0x03,
            big_energy_mech=(value >> 5) & 0x03,
            central_highland=(value >> 7) & 0x03,
            trapezoid_highland=bool(value & (1 << 9)),
            dart_last_hit_time=(value >> 11) & 0x1FF,
            dart_last_hit_target=(value >> 20) & 0x07,
            center_buff_point=(value >> 23) & 0x03,
            fortress_buff_point=(value >> 25) & 0x03,
            outpost_buff_point=(value >> 27) & 0x03,
            base_buff_point=bool(value & (1 << 29))
        )

    def _parse_referee_warning(self, data: bytes) -> RefereeWarning:
        """解析裁判警告数据 (0x0104)"""
        return RefereeWarning(
            penalty_level=data[0],
            foul_robot_id=data[1],
            foul_count=data[2]
        )

    def _parse_dart_launch_data(self, data: bytes) -> DartLaunchData:
        """解析飞镖发射相关数据 (0x0105)"""
        dart_remaining_time = data[0]
        value = struct.unpack('<H', data[1:3])[0]

        return DartLaunchData(
            dart_remaining_time=dart_remaining_time,
            last_hit_target=value & 0x07,
            target_hit_count=(value >> 3) & 0x07,
            selected_target=(value >> 6) & 0x07
        )

    def _parse_robot_performance(self, data: bytes) -> RobotPerformance:
        """解析机器人性能体系数据 (0x0201)"""
        robot_id = data[0]
        robot_level = data[1]
        current_hp, maximum_hp = struct.unpack('<HH', data[2:6])
        shooter_cooling, shooter_limit, chassis_limit = struct.unpack(
            '<HHH', data[6:12])
        power_output = data[12]

        return RobotPerformance(
            robot_id=robot_id,
            robot_level=robot_level,
            current_hp=current_hp,
            maximum_hp=maximum_hp,
            shooter_barrel_cooling_value=shooter_cooling,
            shooter_barrel_heat_limit=shooter_limit,
            chassis_power_limit=chassis_limit,
            power_management_gimbal_output=bool(power_output & 0x01),
            power_management_chassis_output=bool(power_output & 0x02),
            power_management_shooter_output=bool(power_output & 0x04)
        )

    def _parse_robot_heat(self, data: bytes) -> RobotHeat:
        """解析实时底盘缓冲能量和射击热量数据 (0x0202)"""
        chassis_voltage, chassis_current = struct.unpack('<HH', data[0:4])
        chassis_power = struct.unpack('<f', data[4:8])[0]
        buffer_energy, heat_17mm, heat_42mm = struct.unpack('<HHH', data[8:14])

        return RobotHeat(
            chassis_current_voltage=chassis_voltage,
            chassis_current_current=chassis_current,
            chassis_current_power=chassis_power,
            buffer_energy=buffer_energy,
            shooter_17mm_barrel_heat=heat_17mm,
            shooter_42mm_barrel_heat=heat_42mm
        )

    def _parse_robot_position(self, data: bytes) -> RobotPosition:
        """解析机器人位置数据 (0x0203)"""
        x, y, angle = struct.unpack('<fff', data[:12])
        return RobotPosition(x=x, y=y, angle=angle)

    def _parse_robot_buff(self, data: bytes) -> RobotBuff:
        """解析机器人增益和底盘能量数据 (0x0204)"""
        recovery = data[0]
        cooling = data[1]
        defence = data[2]
        vulnerability = data[3]
        attack = struct.unpack('<H', data[4:6])[0]
        remaining = data[6]
        # energy = data[7]  # 保留

        return RobotBuff(
            recovery_buff=recovery,
            cooling_buff=cooling,
            defence_buff=defence,
            vulnerability_buff=vulnerability,
            attack_buff=attack,
            remaining_energy=remaining
        )

    def _parse_damage_state(self, data: bytes) -> DamageState:
        """解析伤害状态数据 (0x0206)"""
        armor_id = data[0] & 0x0F
        damage_type = (data[0] >> 4) & 0x0F
        return DamageState(armor_id=armor_id, damage_type=damage_type)

    def _parse_shoot_data(self, data: bytes) -> ShootData:
        """解析实时射击数据 (0x0207)"""
        bullet_type = data[0]
        shooter_id = data[1]
        frequency = data[2]
        initial_speed = struct.unpack('<f', data[3:7])[0]

        return ShootData(
            bullet_type=bullet_type,
            shooter_id=shooter_id,
            launching_frequency=frequency,
            initial_speed=initial_speed
        )

    def _parse_allowed_shoot(self, data: bytes) -> AllowedShoot:
        """解析允许发弹量 (0x0208)"""
        ammo_17mm, ammo_42mm, gold, fortress = struct.unpack('<HHHH', data[:8])
        return AllowedShoot(
            projectile_allowance_17mm=ammo_17mm,
            projectile_allowance_42mm=ammo_42mm,
            remaining_gold_coin=gold,
            fortress_reserve_17mm=fortress
        )

    def _parse_rfid_status(self, data: bytes) -> RFIDStatus:
        """解析机器人RFID模块状态 (0x0209)"""
        bits_low = struct.unpack('<I', data[0:4])[0]
        bits_high = data[4]
        return RFIDStatus(
            detected_rfid_bits_low=bits_low,
            detected_rfid_bits_high=bits_high
        )

    def _parse_dart_operator_cmd(self, data: bytes) -> DartOperatorCmd:
        """解析飞镖选手端指令数据 (0x020A)"""
        status = data[0]
        # data[1] 保留
        switch_time, launch_time = struct.unpack('<HH', data[2:6])
        return DartOperatorCmd(
            dart_station_status=status,
            target_switch_time=switch_time,
            last_launch_time=launch_time
        )

    def _parse_ground_robot_position(self, data: bytes) -> GroundRobotPosition:
        """解析地面机器人位置数据 (0x020B)"""
        positions = struct.unpack('<10f', data[:40])
        return GroundRobotPosition(
            hero_x=positions[0], hero_y=positions[1],
            engineer_x=positions[2], engineer_y=positions[3],
            infantry_3_x=positions[4], infantry_3_y=positions[5],
            infantry_4_x=positions[6], infantry_4_y=positions[7]
            # positions[8], positions[9] 保留
        )

    def _parse_radar_mark_progress(self, data: bytes) -> RadarMarkProgress:
        """解析雷达标记进度数据 (0x020C)"""
        value = struct.unpack('<H', data[:2])[0]
        return RadarMarkProgress(
            enemy_hero_marked=bool(value & (1 << 0)),
            enemy_engineer_marked=bool(value & (1 << 1)),
            enemy_infantry_3_marked=bool(value & (1 << 2)),
            enemy_infantry_4_marked=bool(value & (1 << 3)),
            enemy_aerial_marked=bool(value & (1 << 4)),
            enemy_sentry_marked=bool(value & (1 << 5)),
            ally_hero_marked=bool(value & (1 << 6)),
            ally_engineer_marked=bool(value & (1 << 7)),
            ally_infantry_3_marked=bool(value & (1 << 8)),
            ally_infantry_4_marked=bool(value & (1 << 9)),
            ally_aerial_marked=bool(value & (1 << 10)),
            ally_sentry_marked=bool(value & (1 << 11))
        )

    def _parse_sentry_decision_sync(self, data: bytes) -> SentryDecisionSync:
        """解析哨兵自主决策信息同步 (0x020D)"""
        value1 = struct.unpack('<I', data[0:4])[0]
        value2 = struct.unpack('<H', data[4:6])[0]

        return SentryDecisionSync(
            exchanged_ammo=value1 & 0x7FF,
            remote_ammo_count=(value1 >> 11) & 0x0F,
            remote_hp_count=(value1 >> 15) & 0x0F,
            can_free_revive=bool(value1 & (1 << 19)),
            can_buy_revive=bool(value1 & (1 << 20)),
            revive_cost=(value1 >> 21) & 0x3FF,
            in_disengage=bool(value2 & 0x01),
            remaining_ammo_exchange=(value2 >> 1) & 0x7FF,
            sentry_posture=(value2 >> 12) & 0x03,
            can_activate_rune=bool(value2 & (1 << 14))
        )

    def _parse_radar_decision_sync(self, data: bytes) -> RadarDecisionSync:
        """解析雷达自主决策信息同步 (0x020E)"""
        value = data[0]
        return RadarDecisionSync(
            double_vulnerability_chance=value & 0x03,
            enemy_double_vulnerability=bool(value & (1 << 2)),
            encryption_level=(value >> 3) & 0x03,
            can_modify_key=bool(value & (1 << 5))
        )

    def _parse_map_click_data(self, data: bytes) -> MapClickData:
        """解析选手端小地图交互数据 (0x0303)"""
        x, y = struct.unpack('<ff', data[0:8])
        key = data[8]
        target_id = struct.unpack('<H', data[9:11])[0]
        source_id = struct.unpack('<H', data[11:13])[0]
        # 注意：文档显示为15字节，但只用到13字节

        return MapClickData(
            target_x=x,
            target_y=y,
            keyboard_key=key,
            target_robot_id=target_id,
            source_id=source_id
        )

    def _parse_map_radar_data(self, data: bytes) -> MapRadarData:
        """解析选手端小地图接收雷达数据 (0x0305)"""
        # 偏移量8开始的数据
        positions = struct.unpack('<12H', data[8:32])
        return MapRadarData(
            infantry_3_x=positions[0], infantry_3_y=positions[1],
            infantry_4_x=positions[2], infantry_4_y=positions[3],
            # positions[4], positions[5] 保留
            sentry_x=positions[6], sentry_y=positions[7]
        )

    def _parse_map_path_data(self, data: bytes) -> MapPathData:
        """解析选手端小地图接收路径数据 (0x0307)"""
        intention = data[0]
        start_x, start_y = struct.unpack('<HH', data[1:5])
        delta_x = [int(b) for b in data[5:54]]
        delta_y = [int(b) for b in data[54:103]]
        sender_id = struct.unpack('<H', data[103:105])[0]

        return MapPathData(
            intention=intention,
            start_x=start_x,
            start_y=start_y,
            delta_x=delta_x,
            delta_y=delta_y,
            sender_id=sender_id
        )

    def _parse_map_robot_data(self, data: bytes) -> MapRobotData:
        """解析选手端小地图接收机器人数据 (0x0308)"""
        sender_id, receiver_id = struct.unpack('<HH', data[0:4])
        message = data[4:34].decode('utf-16', errors='ignore').rstrip('\x00')

        return MapRobotData(
            sender_id=sender_id,
            receiver_id=receiver_id,
            message=message
        )

    def _parse_enemy_position(self, data: bytes) -> EnemyPosition:
        """解析对方机器人的位置坐标 (0x0A01)"""
        positions = struct.unpack('<12H', data[:24])
        return EnemyPosition(
            hero_x=positions[0], hero_y=positions[1],
            engineer_x=positions[2], engineer_y=positions[3],
            infantry_3_x=positions[4], infantry_3_y=positions[5],
            infantry_4_x=positions[6], infantry_4_y=positions[7],
            aerial_x=positions[8], aerial_y=positions[9],
            sentry_x=positions[10], sentry_y=positions[11]
        )

    def _parse_enemy_hp(self, data: bytes) -> EnemyHP:
        """解析对方机器人的血量信息 (0x0A02)"""
        hp_values = struct.unpack('<6H', data[:12])
        return EnemyHP(
            hero_hp=hp_values[0],
            engineer_hp=hp_values[1],
            infantry_3_hp=hp_values[2],
            infantry_4_hp=hp_values[3],
            # hp_values[4] 保留
            sentry_hp=hp_values[5]
        )

    def _parse_enemy_ammo(self, data: bytes) -> EnemyAmmo:
        """解析对方机器人的剩余发弹量信息 (0x0A03)"""
        ammo_values = struct.unpack('<5H', data[:10])
        return EnemyAmmo(
            hero_ammo=ammo_values[0],
            infantry_3_ammo=ammo_values[1],
            infantry_4_ammo=ammo_values[2],
            aerial_ammo=ammo_values[3],
            sentry_ammo=ammo_values[4]
        )

    def _parse_enemy_team_status(self, data: bytes) -> EnemyTeamStatus:
        """解析对方队伍的宏观状态信息 (0x0A04)"""
        gold, total_gold = struct.unpack('<HH', data[0:4])
        status = struct.unpack('<I', data[4:8])[0]

        return EnemyTeamStatus(
            remaining_gold=gold,
            total_gold=total_gold,
            supply_area_occupied=bool(status & (1 << 0)),
            central_highland_status=(status >> 1) & 0x03,
            trapezoid_highland_status=bool(status & (1 << 3)),
            fortress_status=(status >> 4) & 0x03,
            outpost_status=(status >> 6) & 0x03,
            base_status=bool(status & (1 << 8)),
            terrain_cross_status=(status >> 9)
        )

    def _parse_enemy_buff(self, data: bytes) -> EnemyBuff:
        """解析对方各机器人当前增益效果 (0x0A05)"""
        # 解析每个机器人的增益数据
        # 英雄 (0-6)
        hero_recovery = data[0]
        hero_cooling = struct.unpack('<H', data[1:3])[0]
        hero_defence = data[3]
        hero_vulnerability = data[4]
        hero_attack = struct.unpack('<H', data[5:7])[0]

        # 工程 (7-13)
        engineer_recovery = data[7]
        engineer_cooling = struct.unpack('<H', data[8:10])[0]
        engineer_defence = data[10]
        engineer_vulnerability = data[11]
        engineer_attack = struct.unpack('<H', data[12:14])[0]

        # 3号步兵 (14-20)
        inf3_recovery = data[14]
        inf3_cooling = struct.unpack('<H', data[15:17])[0]
        inf3_defence = data[17]
        inf3_vulnerability = data[18]
        inf3_attack = struct.unpack('<H', data[19:21])[0]

        # 4号步兵 (21-27)
        inf4_recovery = data[21]
        inf4_cooling = struct.unpack('<H', data[22:24])[0]
        inf4_defence = data[24]
        inf4_vulnerability = data[25]
        inf4_attack = struct.unpack('<H', data[26:28])[0]

        # 哨兵 (28-35)
        sentry_recovery = data[28]
        sentry_cooling = struct.unpack('<H', data[29:31])[0]
        sentry_defence = data[31]
        sentry_vulnerability = data[32]
        sentry_attack = struct.unpack('<H', data[33:35])[0]
        sentry_posture = data[35]

        return EnemyBuff(
            hero_recovery=hero_recovery, hero_cooling=hero_cooling,
            hero_defence=hero_defence, hero_vulnerability=hero_vulnerability,
            hero_attack=hero_attack,
            engineer_recovery=engineer_recovery, engineer_cooling=engineer_cooling,
            engineer_defence=engineer_defence, engineer_vulnerability=engineer_vulnerability,
            engineer_attack=engineer_attack,
            infantry_3_recovery=inf3_recovery, infantry_3_cooling=inf3_cooling,
            infantry_3_defence=inf3_defence, infantry_3_vulnerability=inf3_vulnerability,
            infantry_3_attack=inf3_attack,
            infantry_4_recovery=inf4_recovery, infantry_4_cooling=inf4_cooling,
            infantry_4_defence=inf4_defence, infantry_4_vulnerability=inf4_vulnerability,
            infantry_4_attack=inf4_attack,
            sentry_recovery=sentry_recovery, sentry_cooling=sentry_cooling,
            sentry_defence=sentry_defence, sentry_vulnerability=sentry_vulnerability,
            sentry_attack=sentry_attack, sentry_posture=sentry_posture
        )

    def _parse_enemy_jamming_key(self, data: bytes) -> EnemyJammingKey:
        """解析对方干扰波密钥 (0x0A06)"""
        key = data[:6].decode('ascii', errors='ignore')
        return EnemyJammingKey(key=key)
