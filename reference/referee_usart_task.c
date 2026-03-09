#include "referee_usart_task.h"
#include "fifo.h"

static void referee_unpack_fifo_data(void);
void referee_data_solve(uint8_t *frame);

extern UART_HandleTypeDef huart1;
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
extern uint8_t referee_rx_buf[256];

robot_status_t robotStatus;
power_heat_data_t powerHeatData;
robot_pos_t robotPos;
buff_t buff;
hurt_data_t hurtData;
shoot_data_t shootData;
projectile_allowance_t projectileAllowance;
radar_mark_data_t radarMarkData;
robot_interaction_data_t robotInteractionData;
game_status_t game_status;
game_robot_HP_t game_robot_HP;

frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

void init_referee_struct_data() {
    memset(&robotStatus, 0, sizeof(robotStatus));
    memset(&powerHeatData, 0, sizeof(powerHeatData));
    memset(&robotPos, 0, sizeof(robotPos));
    memset(&buff, 0, sizeof(buff));
    memset(&hurtData, 0, sizeof(hurtData));
    memset(&shootData, 0, sizeof(shootData));
    memset(&projectileAllowance, 0, sizeof(projectileAllowance));
    memset(&radarMarkData, 0, sizeof(radarMarkData));
    memset(&robotInteractionData, 0, sizeof(robotInteractionData));
    memset(&game_status, 0, sizeof(game_status));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP));
}

void referee_task(void const *args) {
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, referee_rx_buf, 256);

    while (1) {
        referee_unpack_fifo_data();
        HAL_Delay(2);
    }
}

void referee_unpack_fifo_data(void) {
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while (fifo_s_used(&referee_fifo)) {
        byte = fifo_s_get(&referee_fifo);
        switch (p_obj->unpack_step) {
            case STEP_HEADER_SOF: {
                if (byte == sof) {
                    p_obj->unpack_step = STEP_LENGTH_LOW;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                } else {
                    p_obj->index = 0;
                }
            }
                break;

            case STEP_LENGTH_LOW: {
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_LENGTH_HIGH;
            }
                break;

            case STEP_LENGTH_HIGH: {
                p_obj->data_len |= (byte << 8);
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)) {
                    p_obj->unpack_step = STEP_FRAME_SEQ;
                } else {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
                break;
            case STEP_FRAME_SEQ: {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_HEADER_CRC8;
            }
                break;

            case STEP_HEADER_CRC8: {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                        p_obj->unpack_step = STEP_DATA_CRC16;
                    } else {
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                    p_obj->index = STEP_DATA_CRC16;
                }
            }
                break;

            case STEP_DATA_CRC16: {
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                    if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len)) {
                        referee_data_solve(p_obj->protocol_packet);
                    }
                    referee_data_solve(p_obj->protocol_packet);
                }
            }
                break;

            default: {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
                break;
        }
    }
}

void referee_data_solve(uint8_t *frame) {
    uint16_t cmd_id = 0;
    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case ID_GAME_STATUS: {
            memcpy(&game_status, frame + index, sizeof(game_status));
            break;
        }
        case ID_BOT_HP: {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP));
            break;
        }
        case ID_BOT_PERFORMANCE: {
            memcpy(&robotStatus, frame + index, sizeof(robotStatus));
            break;
        }
        case ID_REALTIME_POWER: {
            memcpy(&powerHeatData, frame + index, sizeof(powerHeatData));
            break;
        }
        case ID_BOT_LOCATION: {
            memcpy(&robotPos, frame + index, sizeof(robotPos));
            break;
        }
        case ID_BOT_BOOST: {
            memcpy(&buff, frame + index, sizeof(buff));
            break;
        }
        case ID_DAMAGE_STATUS: {
            memcpy(&hurtData, frame + index, sizeof(hurtData));
            break;
        }
        case ID_REALTIME_SHOT: {
            memcpy(&shootData, frame + index, sizeof(shootData));
            break;
        }
        case ID_ALLOWED_BULLET_AMOUNT: {
            memcpy(&projectileAllowance, frame + index, sizeof(projectileAllowance));
            break;
        }
        case ID_DART_MARKING_PROGRESS: {
            memcpy(&radarMarkData, frame + index, sizeof(radarMarkData));
            break;
        }
        case ID_BOT_INTERACTION: {
            memcpy(&robotInteractionData, frame + index, sizeof(robotInteractionData));
            break;
        }
        default: {
            break;
        }
    }
}
