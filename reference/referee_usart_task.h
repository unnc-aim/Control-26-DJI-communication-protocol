//
// Created by DJI in 2019.
//

#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "main.h"
#include "CRC8_CRC16.h"
#include "stm32h7xx_hal.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

/*
 * referee task running in rtos
 */
void referee_task(void const *argument);

#pragma pack(push, 1)

typedef struct {
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
} frame_header_struct_t;

typedef enum {
    STEP_HEADER_SOF = 0,
    STEP_LENGTH_LOW = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16 = 5,
} unpack_step_e;

typedef struct {
    frame_header_struct_t *p_header;
    uint16_t data_len;
    uint8_t protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    unpack_step_e unpack_step;
    uint16_t index;
} unpack_data_t;

typedef enum {
    // 比赛状态数据，固定3Hz 频率发送 服务器→全体机器人
    ID_GAME_STATUS = 0x0001,
    // 机器人血量数据，固定3Hz 频率发送 服务器→全体机器人
    ID_BOT_HP = 0x0003,
    // 机器人性能体系数据，固定10Hz 频率发送 主控模块→对应机器人
    ID_BOT_PERFORMANCE = 0x0201,
    // 实时功率热量数据，固定50Hz 频率发送 主控模块→对应机器人
    ID_REALTIME_POWER = 0x0202,
    // 机器人位置数据，固定10Hz 频率发送 主控模块→对应机器人
    ID_BOT_LOCATION = 0X0203,
    // 机器人增益数据，固定3Hz 频率发送 服务器→对应机器人
    ID_BOT_BOOST = 0X0204,
    // 伤害状态数据，伤害发生后发送 主控模块→对应机器人
    ID_DAMAGE_STATUS = 0X0206,
    // 实时射击数据，弹丸发射后发送 主控模块→对应机器人
    ID_REALTIME_SHOT = 0X0207,
    // 允许发弹量，固定10Hz 频率发送 服务器→己方英雄、步兵、哨兵、空中机器人
    ID_ALLOWED_BULLET_AMOUNT = 0x0208,
    // 雷达标记进度数据，固定1Hz 频率发送 服务器→己方雷达机器人
    ID_DART_MARKING_PROGRESS = 0X020C,
    // 机器人交互数据，发送方触发发送，频率上限为10Hz
    ID_BOT_INTERACTION = 0X0301
} REFEREE_COMMAND_ID;

#pragma pack(pop)

#pragma pack(1)

/**
    0 1
    0-3 bit：比赛类型
    • 1：RoboMaster 机甲大师超级对抗赛
    • 2：RoboMaster 机甲大师高校单项赛
    • 3：ICRA RoboMaster 高校人工智能挑战赛
    • 4：RoboMaster 机甲大师高校联盟赛3V3 对抗
    • 5：RoboMaster 机甲大师高校联盟赛步兵对抗
    4-7 bit：当前比赛阶段
    • 0：未开始比赛
    • 1：准备阶段
    • 2：自检阶段
    • 3：5 秒倒计时
    • 4：比赛中
    • 5：比赛结算中

    1 2 当前阶段剩余时间，单位：秒
    3 8 UNIX 时间，当机器人正确连接到裁判系统的NTP 服务器后生效
 */
typedef struct {
    uint8_t game_type: 4;
    uint8_t game_progress: 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

/**
    0 2 红1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为0
    2 2 红2 工程机器人血量
    4 2 红3 步兵机器人血量
    6 2 红4 步兵机器人血量
    8 2 红5 步兵机器人血量
    10 2 红7 哨兵机器人血量
    12 2 红方前哨站血量
    14 2 红方基地血量
    16 2 蓝1 英雄机器人血量
    18 2 蓝2 工程机器人血量
    20 2 蓝3 步兵机器人血量
    22 2 蓝4 步兵机器人血量
    24 2 蓝5 步兵机器人血量
    26 2 蓝7 哨兵机器人血量
    28 2 蓝方前哨站血量
    30 2 蓝方基地血量
 */
typedef struct {
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t reserved1;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t reserved2;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} game_robot_HP_t;

/**
* 0 1 本机器人 ID
1 1 机器人等级
2 2 机器人当前血量
4 2 机器人血量上限
6 2 机器人射击热量每秒冷却值
8 2 机器人射击热量上限
10 2 机器人底盘功率上限
12 1
电源管理模块的输出情况：
bit 0：gimbal 口输出，0 为无输出，1 为 24V 输出
bit 1：chassis 口输出，0 为无输出，1 为 24V 输出
bit 2：shooter 口输出，0 为无输出，1 为 24V 输出
*/
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} robot_status_t;

/**
* 0 2 保留位
2 2 保留位
4 4 保留位
8 2 缓冲能量（单位：J）
10 2 第 1 个 17mm 发射机构的射击热量
12 2 第 2 个 17mm 发射机构的射击热量
14 2 42mm 发射机构的射击热量
*/
typedef struct {
    uint16_t reserved1;
    uint16_t reserved2;
    float reserved3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/**
0 4 本机器人位置x 坐标，单位：m
4 4 本机器人位置y 坐标，单位：m
8 4 本机器人位置z 坐标，单位：m
12 4 本机器人测速模块朝向，单位：度。正北为0 度
 */
typedef struct {
    float x;
    float y;
    float angle;
} robot_pos_t;

/**
*  1 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
1 1 机器人射击热量冷却倍率（直接值，值为 5 表示 5 倍冷却）
2 1 机器人防御增益（百分比，值为 50 表示 50%防御增益）
3 1 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
4 2 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
5 1
bit 0-4：机器人剩余能量值反馈，以 16 进制标识机器人剩余能量值比例，仅
在机器人剩余能量小于 50%时反馈，其余默认反馈 0x32。
bit 0：在剩余能量≥50%时为 1，其余情况为 0
bit 1：在剩余能量≥30%时为 1，其余情况为 0
bit 2：在剩余能量≥15%时为 1，其余情况为 0
bit 3：在剩余能量≥5%时为 1，其余情况为 0
Bit 4：在剩余能量≥1%时为 1，其余情况为 0
*/
typedef struct {
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} buff_t;

/**
    0 1
    bit0-3：当扣血原因为装甲模块或测速模块时，该4bit 组成的数值为装甲模
    块或测速模块的ID 编号；其他原因扣血时，该数值为0
    bit4-7：血量变化类型
     0 装甲被弹丸攻击扣血
     1 裁判系统重要模块离线扣血
     2 射击初速度超限扣血
     3 枪口热量超限扣血
     4 底盘功率超限扣血
     5 装甲模块受到撞击扣血
 */
typedef struct {
    uint8_t armor_id: 4;
    uint8_t HP_deduction_reason: 4;
} hurt_data_t;

/**
    0 1
    弹丸类型：
     1：17mm 弹丸
     2：42mm 弹丸
    1 1
    发射机构ID：
     1：第1 个17mm 发射机构
     2：第2 个17mm 发射机构
     3：42mm 发射机构
    2 1 弹丸射速（单位：Hz）
    3 4 弹丸初速度（单位：m/s）
 */
typedef struct {
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

/**
    0 2 17mm 弹丸允许发弹量
    2 2 42mm 弹丸允许发弹量
    4 2 剩余金币数量
 */
typedef struct {
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
} projectile_allowance_t;

/**
bit 0：对方 1 号英雄机器人易伤情况
bit 1：对方 2 号工程机器人易伤情况
bit 2：对方 3 号步兵机器人易伤情况
bit 3：对方 4 号步兵机器人易伤情况
bit 4：对方哨兵机器人易伤情况
 */
typedef struct {
    uint8_t hero_debuff: 1;
    uint8_t engineer_debuff: 1;
    uint8_t inf3_debuff: 1;
    uint8_t inf4_debuff: 1;
    uint8_t sentinel_debuff: 1;
    uint8_t reserved: 3;
} radar_mark_data_t;

/**
    0 2 子内容ID 需为开放的子内容ID
        0x0200~0x02FF x≤113 机器人之间通信
        0x0100 2 选手端删除图层
        0x0101 15 选手端绘制一个图形
        0x0102 30 选手端绘制两个图形
        0x0103 75 选手端绘制五个图形
        0x0104 105 选手端绘制七个图形
        0x0110 45 选手端绘制字符图形
    2 2 发送者ID 需与自身ID 匹配，ID 编号详见附录
    4 2 接收者ID
         仅限己方通信
         需为规则允许的车间通信接收者
         若接收者为选手端，则仅可发送至发送者对应的选
        手端
         1：红方英雄机器人
         2：红方工程机器人
         3/4/5：红方步兵机器人（与机器人ID 3~5 对应）
         6：红方空中机器人
         7：红方哨兵机器人
         8：红方飞镖
         9：红方雷达
         10：红方前哨站
         11：红方基地
         101：蓝方英雄机器人
         102：蓝方工程机器人
         103/104/105：蓝方步兵机器人（与机器人ID 3~5 对应）
         106：蓝方空中机器人
         107：蓝方哨兵机器人
         108：蓝方飞镖
         109：蓝方雷达
         110：蓝方前哨站
         111：蓝方基地
        选手端ID 如下所示：
         0x0101：红方英雄机器人选手端
         0x0102：红方工程机器人选手端
         0x0103/0x0104/0x0105：红方步兵机器人选手端（与机器人ID3~5 对应）
         0x0106：红方空中机器人选手端
         0x0165：蓝方英雄机器人选手端
         0x0166：蓝方工程机器人选手端
         0x0167/0x0168/0x0169：蓝方步兵机器人选手端（与机器人ID3~5 对应）
    6 x 内容数据段 x 最大为113
 */
typedef struct {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113]; // TODO length TBD
} robot_interaction_data_t;

/** 子内容ID：0x0100 删除图层
    0 1 删除操作
     0：空操作
     1：删除图层
     2：删除所有
    1 1 图层数 图层数：0~9
 */
typedef struct {
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

/** 表 2-23 子内容ID：0x0101 绘制一个图形
    0 3 图形名 在图形删除，修改等操作中，作为索引
    3 4 图形配置1
        bit 0-2：图形操作：
         0：空操作
         1：增加
         2：修改
         3：删除
        bit 3-5：图形类型：
         0：直线
         1：矩形
         2：正圆
         3：椭圆
         4：圆弧
         5：浮点数
         6：整型数
         7：字符
        bit 6-9：图层数（0~9）
        bit 10-13：颜色：
         0：红/蓝（己方颜色）
         1：黄色
         2：绿色
         3：橙色
         4：紫红色
         5：粉色
         6：青色
         7：黑色
         8：白色
        bit 14-31：依绘制的不同图形含义不同，详见下表
            * 角度值含义为：0°指12 点钟方向，顺时针绘制；
            类型  details_a   details_b
            直线  -           -
            矩形  -           -
            正圆  -           -
            椭圆  -           -
            圆弧  起始角度      终止角度
            浮点数 字体大小     无作用
            整型数 字体大小     -
            字符  字体大小      字符长度
    7 4 图形配置2
        bit 0-9：线宽，建议字体大小与线宽比例为10：1
        bit 10-20：起点/圆心x 坐标
        bit 21-31：起点/圆心y 坐标
    11 4 图形配置3 依绘制的不同图形含义不同，详见下表
        * 屏幕位置：（0,0）为屏幕左下角（1920，1080）为屏幕右上角；
        * 浮点数：整型数均为32 位， 对于浮点数，实际显示的值为输入的值/1000，如在 details_c,details_d,details_e 对应的字节输入1234，选手端实际显示的值将为1.234。
        类型  details_c   details_d   details_e
        直线  -           终点x坐标     终点y 坐标
        矩形  -           对角顶点x坐标 对角顶点y 坐标
        正圆  半径         -           -
        椭圆  -           x半轴长度     y半轴长度
        圆弧  -           x半轴长度     y半轴长度
        浮点数 (      该值除以1000 即实际显示值     )
        整型数 (       32 位整型数，int32_t       )
        字符  -            -           -
 */
typedef struct {
    uint8_t figure_name[3];
    uint32_t operate_type: 3;
    uint32_t figure_type: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t details_a: 9;
    uint32_t details_b: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t details_c: 10;
    uint32_t details_d: 11;
    uint32_t details_e: 11;
} interaction_figure_t;

/** 子内容ID：0x0102 绘制2个图形
    0 15 图形1 与0x0101 的数据段相同
    15 15 图形2 与0x0101 的数据段相同
 */
typedef struct {
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

/** 子内容ID：0x0103 绘制5个图形
    0 15 图形1 与0x0101 的数据段相同
    15 15 图形2 与0x0101 的数据段相同
    30 15 图形3 与0x0101 的数据段相同
    45 15 图形4 与0x0101 的数据段相同
    60 15 图形5 与0x0101 的数据段相同
 */
typedef struct {
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

/** 子内容ID：0x0104 绘制7个图形
    0 15 图形1 与0x0101 的数据段相同
    15 15 图形2 与0x0101 的数据段相同
    30 15 图形3 与0x0101 的数据段相同
    45 15 图形4 与0x0101 的数据段相同
    60 15 图形5 与0x0101 的数据段相同
    75 15 图形6 与0x0101 的数据段相同
    90 15 图形7 与0x0101 的数据段相同
 */
typedef struct {
    interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

/** 子内容ID：0x0110 绘制字符图形
    0 15 字符配置 详见图形数据介绍
    15 30 字符 -
 */
typedef struct {
    interaction_figure_t graphic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

/**
    云台手可通过选手端小地图向机器人发送固定数据。
    触发时发送，两次发送间隔不得低于3秒。

    发送方式一：
        ①点击己方机器人头像；
        ②（可选）按下一个键盘按键或点击对方机器人头像；
        ③点击小地图任意位置。该方式向己方选定的机器人发送地图坐标数据，若点击了对方机器人头像，则以目标机器人ID 代替坐标数据。
    发送方式二：
        ①（可选）按下一个键盘按键或点击对方机器人头像；
        ②点击小地图任意位置。该方式向己方所有机器人发送地图坐标数据，若点击了对方机器人头像，则以目标机器人ID 代替坐标数据。

    0 4 目标位置x 轴坐标，单位m 当发送目标机器人ID 时，该值为0
    4 4 目标位置y 轴坐标，单位m 当发送目标机器人ID 时，该值为0
    8 4 目标位置z 轴坐标，单位m 目前该值始终 为0
    12 1 云台手按下的键盘按键通用键值 无按键按下则为0
    13 2 目标机器人ID 当发送坐标数据时，该值为0
 */
typedef struct {
    float target_position_x;
    float target_position_y;
    float target_position_z;
    uint8_t command_keyboard;
    uint16_t target_robot_id;
} map_command_t;

/**
    选手端小地图可接收机器人数据。
    雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据，该位置会在己方选手端小地图显示。

    0 2 目标机器人ID
    2 4 目标x 位置坐标，单位：m 当x、y 超出边界时则不显示
    6 4 目标y 位置坐标，单位：m 当x、y 超出边界时则不显示
 */
typedef struct {
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} map_robot_data_t;

/**
    哨兵机器人可通过常规链路向己方空中机器人选手端发送路径坐标数据，该路径会在小地图上显示。
    0 1
        1：到目标点攻击；
        2：到目标点防守；
        3：移动到目标点
    1 2 路径起点x 轴坐标，单位：dm
        小地图左下角为坐标原点，水平向右为X轴正方向，竖直向上为Y 轴正方向。显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界
    3 2 路径起点 y轴坐标，单位：dm 的位置将在边界处显示
    5 49 路径点x 轴增量数组，单位：dm
        增量相较于上一个点位进行计算，共49个新点位，X与Y轴增量对应组成点位
    54 49 路径点y 轴增量数组，单位：dm
        增量相较于上一个点位进行计算，共49个新点位，X与Y轴增量对应组成点位
 */
typedef struct {
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
} map_sentry_data_t;

#pragma pack()

#endif
