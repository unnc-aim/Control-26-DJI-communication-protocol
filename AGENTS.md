# AGENTS.md

本文档为 AI 助手（如 Claude、GPT 等）提供项目上下文，帮助 AI 更好地理解和维护此代码库。

## 项目概述

这是一个 ROS 2 Python 功能包，用于解析大疆机甲大师（RoboMaster）裁判系统通信协议。主要功能是从串口读取裁判系统数据，解析后发布为 ROS 2 话题。

## 技术栈

- **ROS 2**: Humble 版本
- **Python**: 3.10+
- **串口通信**: pyserial
- **协议**: DJI 裁判系统通信协议 V1.2.0

## 代码架构

```
dji_referee_protocol/
├── __init__.py              # 包初始化
├── crc_utils.py             # CRC 校验工具（CRC8/CRC16）
├── constant_constants.py     # 协议常量（命令码、数据长度等）
├── data_types.py            # 数据类定义（使用 dataclass）
├── protocol_parser.py       # 协议解析器（状态机）
└── referee_serial_node.py   # ROS 2 主节点
```

## 核心模块说明

### 1. crc_utils.py

CRC 校验工具模块，提供：
- `CRCUtils.get_crc8_check_sum()` - 计算 CRC8 校验和
- `CRCUtils.verify_crc8_check_sum()` - 验证 CRC8 校验
- `CRCUtils.get_crc16_check_sum()` - 计算 CRC16 校验和
- `CRCUtils.verify_crc16_check_sum()` - 验证 CRC16 校验

关键常量：
- `CRC8_INIT = 0xFF`
- `CRC16_INIT = 0xFFFF`

### 2. constant_constants.py

协议常量定义，包括：
- `FrameConstants` - 帧格式常量
- `SerialConfig` - 串口配置
- `CommandID` - 命令码枚举
- `DataLength` - 数据长度常量
- `UnpackStep` - 解包步骤枚举

### 3. data_types.py

使用 Python dataclass 定义的数据类型，每个类对应一个命令码：
- `GameStatus` - 0x0001
- `GameResult` - 0x0002
- `RobotHP` - 0x0003
- 等等...

### 4. protocol_parser.py

协议解析器，使用状态机解析串口数据：

```
状态机流程:
STEP_HEADER_SOF → STEP_LENGTH_LOW → STEP_LENGTH_HIGH
    → STEP_FRAME_SEQ → STEP_HEADER_CRC8 → STEP_DATA_CRC16
```

关键方法：
- `feed_data(data)` - 添加数据到 FIFO 缓冲区
- `unpack()` - 解包一帧数据

### 5. referee_serial_node.py

ROS 2 节点，功能：
- 从串口读取数据
- 调用协议解析器解析
- 发布 ROS 2 话题

## 协议格式

### 帧结构

```
| frame_header | cmd_id | data | frame_tail |
|    5-byte    | 2-byte | n-byte |   2-byte   |
```

### 帧头结构

```
| SOF | data_length | seq | CRC8 |
| 1B  |     2B      | 1B  |  1B  |
```

- SOF = 0xA5 (帧起始)
- data_length: 数据段长度（小端序）
- seq: 包序号
- CRC8: 帧头校验

### 命令码分类

1. **常规链路** (0x0001-0x0308): 服务器 → 机器人
2. **图传链路** (0x0302, 0x0309-0x0311): 自定义控制器 ↔ 机器人
3. **雷达无线链路** (0x0A01-0x0A06): 信号发射源 → 雷达

## 串口配置

| 链路类型 | 波特率 | 数据位 | 停止位 | 校验 |
|----------|--------|--------|--------|------|
| 常规链路 | 115200 | 8 | 1 | 无 |
| 图传链路 | 921600 | 8 | 1 | 无 |

## 编码规范

### 注释要求

- 所有代码必须包含完整的中文注释
- 每个类、函数、方法都需要 docstring
- 变量声明需要有行内注释

### 类型注解

- 使用 Python 类型注解
- 兼容 Pylance 类型检查

示例：
```python
def unpack(self) -> Optional[Tuple[int, Any]]:
    """
    从FIFO缓冲区解包一帧数据

    Returns:
        Optional[Tuple[int, Any]]: (命令码, 数据对象)元组
    """
```

### 命名规范

- 类名：PascalCase（如 `GameStatus`）
- 函数名：snake_case（如 `get_crc8_check_sum`）
- 常量：UPPER_SNAKE_CASE（如 `CRC8_INIT`）
- 私有方法：以 `_` 开头（如 `_parse_frame`）

## 常见问题

### 1. CRC 校验失败

检查数据是否完整，帧头和帧尾的 CRC 是否正确。

### 2. 串口打开失败

检查设备权限：
```bash
sudo chmod 666 /dev/ttyUSB0
```

### 3. 话题不发布

检查配置文件 `config/topic_config.yaml` 中对应话题是否启用。

## 测试方法

```bash
# 语法检查
python3 -m py_compile dji_referee_protocol/*.py

# 类型检查（如果安装了 mypy）
mypy dji_referee_protocol/
```

## 相关文件

- `protocol.md` - 官方协议文档
- `reference/` - 参考代码（C语言实现）
- `config/topic_config.yaml` - 话题配置文件

## 更新日志

### v1.0.0 (2026-03-10)

- 初始版本
- 实现所有命令码的解析
- 支持 ROS 2 Humble
