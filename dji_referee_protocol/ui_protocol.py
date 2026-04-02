"""
UI 绘制协议封装模块

本模块封装裁判系统 0x0301 机器人交互数据中 UI 绘制相关子内容的打包逻辑，
对应协议表 1-25 至 1-31。
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import List
import struct

from .crc_utils import CRCUtils
from .protocol_constants import FrameConstants, CommandID


class UIDataCommandID(IntEnum):
    """
    UI 子内容 ID 枚举
    """

    DELETE = 0x0100
    DRAW_1 = 0x0101
    DRAW_2 = 0x0102
    DRAW_5 = 0x0103
    DRAW_7 = 0x0104
    DRAW_CHAR = 0x0110


class UIGraphicOperation(IntEnum):
    """
    图形操作类型
    """

    NO_OP = 0
    ADD = 1
    MODIFY = 2
    DELETE = 3


class UIGraphicType(IntEnum):
    """
    图形类型
    """

    LINE = 0
    RECTANGLE = 1
    CIRCLE = 2
    OVAL = 3
    ARC = 4
    FLOAT = 5
    INT = 6
    CHAR = 7


class UIColor(IntEnum):
    """
    UI 颜色枚举
    """

    SELF = 0
    YELLOW = 1
    GREEN = 2
    ORANGE = 3
    PURPLE_RED = 4
    PINK = 5
    CYAN = 6
    BLACK = 7
    WHITE = 8


@dataclass
class UIGraphic:
    """
    单个图形结构（15 字节）

    对应协议表 1-27 图形结构。
    """

    name: str
    operation: int
    graphic_type: int
    layer: int
    color: int
    details_a: int
    details_b: int
    width: int
    start_x: int
    start_y: int
    details_c: int
    details_d: int
    details_e: int


class UIDrawingProtocol:
    """
    UI 绘制协议打包工具
    """

    MAX_INTERACTION_CONTENT_LEN: int = 112

    @staticmethod
    def _clip(value: int, min_value: int, max_value: int) -> int:
        """
        限幅整型值
        """
        return max(min_value, min(max_value, int(value)))

    @staticmethod
    def _pack_name(name: str) -> bytes:
        """
        打包 3 字节图形名
        """
        raw = name.encode("ascii", errors="ignore")[:3]
        return raw.ljust(3, b"\x00")

    @staticmethod
    def pack_graphic(graphic: UIGraphic) -> bytes:
        """
        打包单个 15 字节图形
        """
        name_bytes = UIDrawingProtocol._pack_name(graphic.name)

        cfg1 = (
            (UIDrawingProtocol._clip(graphic.operation, 0, 0x7) << 0)
            | (UIDrawingProtocol._clip(graphic.graphic_type, 0, 0x7) << 3)
            | (UIDrawingProtocol._clip(graphic.layer, 0, 0xF) << 6)
            | (UIDrawingProtocol._clip(graphic.color, 0, 0xF) << 10)
            | (UIDrawingProtocol._clip(graphic.details_a, 0, 0x1FF) << 14)
            | (UIDrawingProtocol._clip(graphic.details_b, 0, 0x1FF) << 23)
        )
        cfg2 = (
            (UIDrawingProtocol._clip(graphic.width, 0, 0x3FF) << 0)
            | (UIDrawingProtocol._clip(graphic.start_x, 0, 0x7FF) << 10)
            | (UIDrawingProtocol._clip(graphic.start_y, 0, 0x7FF) << 21)
        )
        cfg3 = (
            (UIDrawingProtocol._clip(graphic.details_c, 0, 0x3FF) << 0)
            | (UIDrawingProtocol._clip(graphic.details_d, 0, 0x7FF) << 10)
            | (UIDrawingProtocol._clip(graphic.details_e, 0, 0x7FF) << 21)
        )

        return name_bytes + struct.pack("<III", cfg1, cfg2, cfg3)

    @staticmethod
    def pack_delete_payload(delete_operation: int, layer: int) -> bytes:
        """
        打包删除图层负载（表 1-25，子内容 ID 0x0100）
        """
        return bytes(
            [
                UIDrawingProtocol._clip(delete_operation, 0, 2),
                UIDrawingProtocol._clip(layer, 0, 9),
            ]
        )

    @staticmethod
    def choose_graphics_data_cmd_id(graphic_count: int) -> int:
        """
        根据图形数量选择子内容 ID
        """
        if graphic_count == 1:
            return int(UIDataCommandID.DRAW_1)
        if graphic_count == 2:
            return int(UIDataCommandID.DRAW_2)
        if graphic_count == 5:
            return int(UIDataCommandID.DRAW_5)
        if graphic_count == 7:
            return int(UIDataCommandID.DRAW_7)
        raise ValueError("图形数量仅支持 1/2/5/7")

    @staticmethod
    def pack_graphics_payload(graphics: List[UIGraphic]) -> bytes:
        """
        打包图形组负载（表 1-26/1-28/1-29/1-30）
        """
        return b"".join(UIDrawingProtocol.pack_graphic(g) for g in graphics)

    @staticmethod
    def pack_char_payload(graphic: UIGraphic, text: str) -> bytes:
        """
        打包字符负载（表 1-31，子内容 ID 0x0110）
        """
        text_bytes = text.encode("utf-8", errors="ignore")[:30]
        return UIDrawingProtocol.pack_graphic(graphic) + text_bytes.ljust(30, b"\x00")

    @staticmethod
    def build_robot_interaction_frame(
        seq: int,
        data_cmd_id: int,
        sender_id: int,
        receiver_id: int,
        content_payload: bytes,
    ) -> bytes:
        """
        构建 0x0301 完整帧并附加 CRC
        """
        interaction_data = struct.pack(
            "<HHH", int(data_cmd_id), int(sender_id), int(receiver_id)
        ) + content_payload
        if len(content_payload) > UIDrawingProtocol.MAX_INTERACTION_CONTENT_LEN:
            raise ValueError("机器人交互内容数据超过 112 字节上限")

        data_len = len(interaction_data)
        total_len = (
            FrameConstants.FRAME_HEADER_LENGTH
            + FrameConstants.CMD_ID_LENGTH
            + data_len
            + FrameConstants.FRAME_TAIL_LENGTH
        )
        frame = bytearray(total_len)

        frame[0] = FrameConstants.SOF
        frame[1] = data_len & 0xFF
        frame[2] = (data_len >> 8) & 0xFF
        frame[3] = int(seq) & 0xFF
        CRCUtils.append_crc8_check_sum(frame, FrameConstants.FRAME_HEADER_LENGTH)

        frame[FrameConstants.CMD_ID_OFFSET] = int(CommandID.ROBOT_INTERACTION) & 0xFF
        frame[FrameConstants.CMD_ID_OFFSET + 1] = (
            int(CommandID.ROBOT_INTERACTION) >> 8
        ) & 0xFF
        frame[
            FrameConstants.DATA_OFFSET : FrameConstants.DATA_OFFSET + data_len
        ] = interaction_data
        CRCUtils.append_crc16_check_sum(frame, len(frame))

        return bytes(frame)
