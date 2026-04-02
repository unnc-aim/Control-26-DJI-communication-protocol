#!/usr/bin/env python3
"""
UI 协议打包测试脚本（无需 ROS 2）

验证 0x0301 下 UI 子内容（表 1-25~1-31）对应的关键打包逻辑：
1. 删除图层负载长度
2. 单图形打包长度
3. 字符绘制负载长度
4. 机器人交互完整帧长度与命令码
"""

import sys
import os

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dji_referee_protocol.protocol_constants import CommandID, FrameConstants
from dji_referee_protocol.ui_protocol import (
    UIDrawingProtocol,
    UIDataCommandID,
    UIGraphic,
    UIGraphicOperation,
    UIGraphicType,
    UIColor,
)


def run_tests() -> bool:
    """
    运行 UI 协议打包测试
    """
    # 表 1-25：删除图层/全部
    delete_payload = UIDrawingProtocol.pack_delete_payload(delete_operation=2, layer=0)
    assert len(delete_payload) == 2, "删除负载长度应为2字节"

    # 表 1-26：单图形（15字节）
    graphic = UIGraphic(
        name='T01',
        operation=int(UIGraphicOperation.ADD),
        graphic_type=int(UIGraphicType.LINE),
        layer=8,
        color=int(UIColor.SELF),
        details_a=0,
        details_b=0,
        width=2,
        start_x=100,
        start_y=200,
        details_c=0,
        details_d=300,
        details_e=400,
    )
    packed_graphic = UIDrawingProtocol.pack_graphic(graphic)
    assert len(packed_graphic) == 15, "单图形打包长度应为15字节"

    # 表 1-31：字符绘制（15字节配置 + 30字节字符）
    char_graphic = UIGraphic(
        name='C01',
        operation=int(UIGraphicOperation.ADD),
        graphic_type=int(UIGraphicType.CHAR),
        layer=8,
        color=int(UIColor.SELF),
        details_a=20,    # 字体大小
        details_b=5,     # 字符长度
        width=2,
        start_x=120,
        start_y=300,
        details_c=0,
        details_d=0,
        details_e=0,
    )
    char_payload = UIDrawingProtocol.pack_char_payload(char_graphic, 'HELLO')
    assert len(char_payload) == 45, "字符负载长度应为45字节"

    # 0x0301 完整帧（frame_header + cmd_id + data + frame_tail）
    frame = UIDrawingProtocol.build_robot_interaction_frame(
        seq=1,
        data_cmd_id=int(UIDataCommandID.DRAW_CHAR),
        sender_id=103,
        receiver_id=0x0167,
        content_payload=char_payload,
    )
    assert frame[0] == FrameConstants.SOF, "帧起始字节错误"
    cmd_id = frame[FrameConstants.CMD_ID_OFFSET] | (frame[FrameConstants.CMD_ID_OFFSET + 1] << 8)
    assert cmd_id == int(CommandID.ROBOT_INTERACTION), "命令码应为0x0301"
    assert len(frame) == 60, "字符绘制完整帧长度应为60字节"

    print("UI 协议打包测试通过")
    return True


if __name__ == '__main__':
    ok = run_tests()
    sys.exit(0 if ok else 1)
