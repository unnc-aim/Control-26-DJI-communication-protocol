# DJI机甲大师裁判系统通信协议 ROS 2 功能包

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow)](https://opensource.org/licenses/MIT)

## 📖 项目简介

本功能包实现了大疆机甲大师（RoboMaster）裁判系统通信协议的 ROS 2 接口。从裁判系统串口读取数据，严格按照官方协议文档解析，并将解析后的数据发布为独立的 ROS 2 话题。

### 主要功能

- ✅ 支持多个串口读取（常规链路 + 图传链路）
- ✅ 严格按照官方协议文档解析数据
- ✅ 将每类数据发布为独立的 ROS 2 话题
- ✅ 支持 YAML 配置文件控制话题发布
- ✅ 完整的中文注释和文档
- ✅ 兼容 ROS 2 Humble 版本

### 协议版本

- 版本：V1.2.0
- 更新日期：2026.02.09

## 📦 目录结构

```
dji-communication-protocol/
├── dji_referee_protocol/          # Python 包
│   ├── __init__.py                # 包初始化文件
│   ├── crc_utils.py               # CRC8/CRC16 校验工具
│   ├── constant_constants.py       # 协议常量定义
│   ├── data_types.py              # 数据类型定义
│   ├── protocol_parser.py         # 协议解析器
│   └── referee_serial_node.py     # ROS 2 主节点
├── config/
│   └── topic_config.yaml          # 话题配置文件
├── launch/
│   └── referee_serial_launch.py   # 启动文件
├── resource/
│   └── dji_referee_protocol       # 资源标记文件
├── test/                          # 测试目录
├── reference/                     # 参考代码（老版本）
├── protocol.md                    # 官方协议文档
├── package.xml                    # ROS 2 包描述
├── setup.py                       # Python 安装配置
├── setup.cfg                      # Python 安装配置
├── README.md                      # 本文件
└── AGENTS.md                      # AI 代理文档
```

## 🚀 快速开始

### 依赖安装

```bash
# 安装 ROS 2 Humble（如果尚未安装）
# 参考：https://docs.ros.org/en/humble/Installation.html

# 安装 Python 依赖
pip install pyserial pyyaml
```

### 编译

```bash
# 创建工作空间（如果尚未创建）
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆或复制本功能包到 src 目录
# cp -r /path/to/dji-communication-protocol .

# 编译
cd ~/ros2_ws
colcon build --packages-select dji_referee_protocol

# 加载环境
source install/setup.bash
```

### 运行

```bash
# 方式一：直接运行节点
ros2 run dji_referee_protocol referee_serial_node

# 方式二：使用启动文件（推荐）
ros2 launch dji_referee_protocol referee_serial_launch.py

# 方式三：指定串口设备
ros2 launch dji_referee_protocol referee_serial_launch.py \
    serial_port_normal:=/dev/ttyUSB0 \
    serial_port_video:=/dev/ttyUSB1

# 方式四：使用配置文件
ros2 launch dji_referee_protocol referee_serial_launch.py \
    config_file:=/path/to/topic_config.yaml
```

## 📋 话题列表

### 常规链路数据

| 话题名称 | 命令码 | 频率 | 描述 |
|---------|--------|------|------|
| `/referee/game_status` | 0x0001 | 1Hz | 比赛状态数据 |
| `/referee/game_result` | 0x0002 | 触发 | 比赛结果数据 |
| `/referee/robot_hp` | 0x0003 | 3Hz | 机器人血量数据 |
| `/referee/field_event` | 0x0101 | 1Hz | 场地事件数据 |
| `/referee/referee_warning` | 0x0104 | 1Hz | 裁判警告数据 |
| `/referee/dart_launch_data` | 0x0105 | 1Hz | 飞镖发射数据 |
| `/referee/robot_performance` | 0x0201 | 10Hz | 机器人性能数据 |
| `/referee/robot_heat` | 0x0202 | 10Hz | 实时热量数据 |
| `/referee/robot_position` | 0x0203 | 1Hz | 机器人位置数据 |
| `/referee/robot_buff` | 0x0204 | 3Hz | 机器人增益数据 |
| `/referee/damage_state` | 0x0206 | 触发 | 伤害状态数据 |
| `/referee/shoot_data` | 0x0207 | 触发 | 射击数据 |
| `/referee/allowed_shoot` | 0x0208 | 10Hz | 允许发弹量 |

### 图传链路数据

| 话题名称 | 命令码 | 频率 | 描述 |
|---------|--------|------|------|
| `/referee/custom_controller_to_robot` | 0x0302 | 30Hz | 自定义控制器数据 |
| `/referee/robot_to_custom_controller` | 0x0309 | 10Hz | 机器人到控制器数据 |
| `/referee/robot_to_custom_client` | 0x0310 | 50Hz | 机器人到客户端数据 |
| `/referee/custom_client_to_robot` | 0x0311 | 75Hz | 客户端到机器人数据 |

### 雷达无线链路数据

| 话题名称 | 命令码 | 频率 | 描述 |
|---------|--------|------|------|
| `/referee/enemy_position` | 0x0A01 | 10Hz | 对方机器人位置 |
| `/referee/enemy_hp` | 0x0A02 | 10Hz | 对方机器人血量 |
| `/referee/enemy_ammo` | 0x0A03 | 10Hz | 对方机器人发弹量 |
| `/referee/enemy_team_status` | 0x0A04 | 10Hz | 对方队伍状态 |
| `/referee/enemy_buff` | 0x0A05 | 10Hz | 对方增益效果 |
| `/referee/enemy_jamming_key` | 0x0A06 | 10Hz | 对方干扰波密钥 |

## ⚙️ 配置文件

配置文件 `config/topic_config.yaml` 用于控制每个话题是否发布：

```yaml
topics:
  # 设置为 false 禁用不需要的话题
  game_status: true      # 比赛状态
  robot_hp: true         # 机器人血量
  enemy_position: false  # 禁用对方位置数据
  # ...
```

## 🔧 参数说明

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `serial_port_normal` | string | /dev/ttyUSB0 | 常规链路串口设备 |
| `serial_port_video` | string | /dev/ttyUSB1 | 图传链路串口设备 |
| `serial_baud_normal` | int | 115200 | 常规链路波特率 |
| `serial_baud_video` | int | 921600 | 图传链路波特率 |
| `config_file` | string | "" | 配置文件路径 |
| `publish_all_topics` | bool | true | 是否发布所有话题 |

## 📚 协议说明

### 帧格式

```
| frame_header | cmd_id | data | frame_tail |
|    5-byte    | 2-byte | n-byte |   2-byte   |
```

### 帧头结构

```
| SOF | data_length | seq | CRC8 |
| 0xA5 |    2-byte   | 1B  |  1B  |
```

### CRC 校验

- CRC8：帧头校验
- CRC16：整包校验

详细协议说明请参考 `protocol.md` 文件。

## 🧪 测试

```bash
# 运行 Python 语法检查
python3 -m py_compile dji_referee_protocol/*.py

# 运行单元测试（如果可用）
pytest test/
```

## 📝 代码风格

- 遵循 PEP 8 Python 代码风格
- 使用完整的中文注释
- 包含类型注解（兼容 Pylance）

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

MIT License

## 📧 联系方式

如有问题，请提交 Issue 或联系维护者。
