# UnifiedInput.msg
# 统一输入消息格式 - 所有输入源（RC/键盘/导航）都转换为此格式
# 由 RC Interpreter 节点发布，Hub 节点订阅

std_msgs/Header header

# ===== 控制源 =====
string control_source      # 当前控制源

# ===== 连接状态 =====
bool connected         # 遥控器连接状态

# ===== 模式标志 =====
bool emergency_stop       # 急停标志（左拨杆下档）
bool autoaim_enabled      # 自瞄使能（鼠标右键）
bool navigation_enabled   # 导航模式（右拨杆中/上档）

# ===== 底盘指令 (云台坐标系) =====
float64 vx                # X方向速度 (前进)
float64 vy                # Y方向速度 (横移)
float64 wz                # 旋转角速度
bool spin_mode            # 小陀螺模式
float64 spin_speed        # 小陀螺速度

# ===== 云台指令 =====
float64 pitch_delta       # Pitch 增量 (度) - 遥控模式用
float64 yaw_delta         # Yaw 增量 (弧度) - 遥控模式用

# ===== 火控指令 =====
bool fire_trigger         # 发射触发
bool burst_mode           # 连发模式
bool friction_on          # 摩擦轮开关
float64 friction_speed    # 摩擦轮速度

# ===== 速度分档 =====
float64 chassis_speed_scale  # 底盘速度比例 (0-1)
