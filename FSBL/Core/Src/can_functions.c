/*
 * can_functions.c
 *
 *  Created on: 1 Mar 2026
 *      Author: tomas
 */

#include "can_functions.h"
// #include "main.h"

const float32_t angle_multiplier[12] = {
        -2 * PI / 6.0,
        -2 * PI / 6.0,
        -2 * PI / 6.0025,
        2 * PI / 6.0,
        -2 * PI / 6.0,
        -2 * PI / 6.0025,
        -2 * PI / 6.0,
        2 * PI / 6.0,
        2 * PI / 6.0025,
        2 * PI / 6.0,
        2 * PI / 6.0,
        2 * PI / 6.0025};

const float32_t angle_multiplier2[12] = {-1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1};

const int32_t calibration_values[12] = {14502, 378, 16320, 15500, 3959, 8364, 7521, 10868, 11109, 14839, 10770, 5026};

void reboot_odrives(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3, FDCAN_TxHeaderTypeDef *TxHeader, void *dataToSend){

    uint8_t command = 0x16;
    uint8_t data_size = 0;

    send_can_data_frame_buffer(can_handle1, TxHeader, dataToSend + 0 *data_size, 0 , command,  data_size, 1 << 0);
    send_can_data_frame_buffer(can_handle1, TxHeader, dataToSend + 3 *data_size, 3 , command,  data_size, 1 << 1);

    send_can_data_frame_buffer(can_handle2, TxHeader, dataToSend + 1 *data_size, 1 , command,  data_size, 1 << 0);
    send_can_data_frame_buffer(can_handle2, TxHeader, dataToSend + 7 *data_size, 7 , command,  data_size, 1 << 1);

    send_can_data_frame_buffer(can_handle3, TxHeader, dataToSend + 4 *data_size, 4 , command,  data_size, 1 << 0);
    send_can_data_frame_buffer(can_handle3, TxHeader, dataToSend + 10*data_size, 10, command,  data_size, 1 << 1);

    uint8_t cnt = 0;
    TIM4->CNT = 0;
    while (cnt < 10) {
        cnt = TIM4->CNT;
    }

    can_handle1->Instance->TXBAR = 0x00000003;
    can_handle2->Instance->TXBAR = 0x00000003;
    can_handle3->Instance->TXBAR = 0x00000003;

    uint32_t pending1 = can_handle1->Instance->TXBRP;
    uint32_t pending2 = can_handle2->Instance->TXBRP;
    uint32_t pending3 = can_handle3->Instance->TXBRP;

    while(pending1 || pending2 || pending3){
        pending1 = can_handle1->Instance->TXBRP;
        pending2 = can_handle2->Instance->TXBRP;
        pending3 = can_handle3->Instance->TXBRP;
    }

    HAL_Delay(3000);
}

uint8_t run_leg_angle_calibration(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
                               CAN_Packet *can_packet, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, FDCAN_RxHeaderTypeDef *RxHeader,
                               uint8_t CAN_RxData[], uint32_t CAN_Received_Datas[], uint32_t *error_codes, TIM_HandleTypeDef *htim3, volatile uint8_t* estimates_flag) {

    can_packet->RTR = 1;
    can_packet->id = 0xa;
    can_packet->data_size = 8;

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

    uint8_t timed_out = 0;
    int32_t angles[10][12];
    float32_t average_angle_float;
    int32_t average_angle_int;

    for(uint8_t i=0; i<10; i++){
        for(uint8_t j=0; j<12; j++){
            angles[i][j] = 0;
        }
    }

//    uint8_t count_messages1 = HAL_FDCAN_GetRxFifoFillLevel(can_handle1, FDCAN_RX_FIFO0);
//    uint8_t count_messages2 = HAL_FDCAN_GetRxFifoFillLevel(can_handle2, FDCAN_RX_FIFO0);
//    uint8_t count_messages3 = HAL_FDCAN_GetRxFifoFillLevel(can_handle3, FDCAN_RX_FIFO0);
//
//    // Wait for heartbeat messages before beginning
//    while(count_messages1 == 0 || count_messages2 == 0 || count_messages3 == 0){
//        count_messages1 = HAL_FDCAN_GetRxFifoFillLevel(can_handle1, FDCAN_RX_FIFO0);
//        count_messages2 = HAL_FDCAN_GetRxFifoFillLevel(can_handle2, FDCAN_RX_FIFO0);
//        count_messages3 = HAL_FDCAN_GetRxFifoFillLevel(can_handle3, FDCAN_RX_FIFO0);
//    }

    int32_t current_count = 0;
    int32_t diff = 0;

    // Flush input FIFOs
    receive_can_data_frame(can_handle1, RxHeader, CAN_RxData, CAN_Received_Datas, 0xff, error_codes);
    receive_can_data_frame(can_handle2, RxHeader, CAN_RxData, CAN_Received_Datas, 0xff, error_codes);
    receive_can_data_frame(can_handle3, RxHeader, CAN_RxData, CAN_Received_Datas, 0xff, error_codes);

    uint8_t cnt=0;

    while(cnt<10){
        *estimates_flag = 0;
        timed_out = send_can_command_to_all_controllers(can_handle1, can_handle2, can_handle3, can_packet, TxHeader, TxData, RxHeader, CAN_RxData,
                CAN_Received_Datas, error_codes, htim3);
        can_handle1->Instance->TXBAR = 0x00000003;
        can_handle2->Instance->TXBAR = 0x00000003;
        can_handle3->Instance->TXBAR = 0x00000003;

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

        while (*estimates_flag != 1) {
            continue;
        }

        for (uint8_t i = 0; i < 12; i++) {
            angles[cnt][i] = CAN_Received_Datas[i + 12];
        }
        cnt++;
    }

    for (uint8_t i = 0; i < 12; i++) {
        average_angle_float = 0;
        for(uint8_t j=0; j<10; j++){
            average_angle_float += angles[j][i]/10.0;
        }

        average_angle_int = (int32_t)roundf(average_angle_float);

        current_count = average_angle_int;
        diff = current_count - calibration_values[i];

        if (diff > 8192) {
            current_count -= 16384;
        } else if (diff < -8192) {
            current_count += 16384;
        }

        diff = (current_count - calibration_values[i]);
//        current_angle = diff / 16384.0 * angle_multiplier[i] * 0;
//        angles[i] = current_angle;

        memcpy(TxData, &diff, 4);

        if (i == 0 || i == 3 || i == 6 || i == 9) {
            send_can_data_frame(can_handle1, TxHeader, TxData, i, 0x19, 4);
        }

        if (i == 1 || i == 2 || i == 7 || i == 8) {
            send_can_data_frame(can_handle2, TxHeader, TxData, i, 0x19, 4);
        }

        if (i == 4 || i == 5 || i == 10 || i == 11) {
            send_can_data_frame(can_handle3, TxHeader, TxData, i, 0x19, 4);
        }
        HAL_Delay(2);
    }
    *estimates_flag = 0;
    return timed_out;
}

void send_can_remote_frame(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node, uint8_t cmd) {
    TxHeader->Identifier = (node << 5) | cmd;
    TxHeader->TxFrameType = FDCAN_REMOTE_FRAME;
    HAL_FDCAN_AddMessageToTxFifoQ(can_handle, TxHeader, TxData);
}

HAL_StatusTypeDef send_can_remote_frame_buffer(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node,
                                               uint8_t cmd, uint32_t buffer_index) {
    TxHeader->Identifier = (node << 5) | cmd;
    TxHeader->TxFrameType = FDCAN_REMOTE_FRAME;
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxBuffer(can_handle, TxHeader, TxData, buffer_index);
    return status;
}

HAL_StatusTypeDef send_can_data_frame(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node, uint8_t cmd,
                                      uint8_t data_length) {
    TxHeader->Identifier = (node << 5) | cmd;
    TxHeader->TxFrameType = FDCAN_DATA_FRAME;
    TxHeader->DataLength = data_length;
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(can_handle, TxHeader, TxData);
    return status;
}

HAL_StatusTypeDef send_can_data_frame_buffer(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node, uint8_t cmd,
                                      uint8_t data_length, uint32_t buffer_index) {
    TxHeader->Identifier = (node << 5) | cmd;
    TxHeader->TxFrameType = FDCAN_DATA_FRAME;
    TxHeader->DataLength = data_length;
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxBuffer(can_handle, TxHeader, TxData, buffer_index);
    return status;
}

void receive_can_data_frame(FDCAN_HandleTypeDef *can_handle, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[], uint32_t CAN_Received_Datas[],
                            uint8_t command, uint32_t *error_codes) {
    uint8_t count_messages = HAL_FDCAN_GetRxFifoFillLevel(can_handle, FDCAN_RX_FIFO0);
    for (int i = 0; i < count_messages; i++) {
        HAL_FDCAN_GetRxMessage(can_handle, FDCAN_RX_FIFO0, RxHeader, RxData);
        uint32_t id = RxHeader->Identifier;
        uint8_t node = id >> 5;
        uint8_t cmd = id & 0x1f;
        if (cmd != command) {
            if (cmd == 0x1) {
                uint32_t data_0 = (RxData[0] << 0) | (RxData[1] << 8) | (RxData[2] << 16) | (RxData[3] << 24);
                //                uint32_t data_1 = (RxData[4] << 0) | (RxData[5] << 8) | (RxData[6] << 16) | (RxData[7] << 24);
                error_codes[node] = data_0;
            }
        } else {
            uint32_t data_0 = (RxData[0] << 0) | (RxData[1] << 8) | (RxData[2] << 16) | (RxData[3] << 24);
            uint32_t data_1 = (RxData[4] << 0) | (RxData[5] << 8) | (RxData[6] << 16) | (RxData[7] << 24);
            CAN_Received_Datas[node] = data_0;
            CAN_Received_Datas[node + 12] = data_1;
        }
    }
}


uint8_t send_can_command_to_all_controllers(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
                                            CAN_Packet *can_packet, FDCAN_TxHeaderTypeDef *TxHeader, void *TxData, FDCAN_RxHeaderTypeDef *RxHeader,
                                            uint8_t CAN_RxData[], uint32_t *CAN_Received_Datas, uint32_t *error_codes, TIM_HandleTypeDef *htim3) {
    uint8_t command = can_packet->id;
    uint8_t data_size = can_packet->data_size;
    uint8_t timed_out = 0;

    // 0, 6 -> bit 0
    // 1, 2 -> bit 1
    // 3, 9 -> bit 2
    // 4, 5 -> bit 3
    // 7, 8 -> bit 4
    // 10, 11 -> bit 5

    if (can_packet->RTR) {

        receive_can_data_frame(can_handle1, RxHeader, CAN_RxData, CAN_Received_Datas, 0xff, error_codes);
        receive_can_data_frame(can_handle2, RxHeader, CAN_RxData, CAN_Received_Datas, 0xff, error_codes);
        receive_can_data_frame(can_handle3, RxHeader, CAN_RxData, CAN_Received_Datas, 0xff, error_codes);

        send_can_remote_frame_buffer(can_handle1, TxHeader, TxData, 9, command,  1 << 1);
        send_can_remote_frame_buffer(can_handle1, TxHeader, TxData, 6, command,  1 << 3);
        send_can_remote_frame_buffer(can_handle1, TxHeader, TxData, 3, command,  1 << 0);
        send_can_remote_frame_buffer(can_handle1, TxHeader, TxData, 0, command,  1 << 2);

        send_can_remote_frame_buffer(can_handle2, TxHeader, TxData, 8, command,  1 << 3);
        send_can_remote_frame_buffer(can_handle2, TxHeader, TxData, 7, command,  1 << 2);
        send_can_remote_frame_buffer(can_handle2, TxHeader, TxData, 2, command,  1 << 1);
        send_can_remote_frame_buffer(can_handle2, TxHeader, TxData, 1, command,  1 << 0);

        send_can_remote_frame_buffer(can_handle3, TxHeader, TxData, 11, command, 1 << 1);
        send_can_remote_frame_buffer(can_handle3, TxHeader, TxData, 10, command, 1 << 0);
        send_can_remote_frame_buffer(can_handle3, TxHeader, TxData, 5, command,  1 << 3);
        send_can_remote_frame_buffer(can_handle3, TxHeader, TxData, 4, command,  1 << 2);
        TIM3->CNT = 0;
        TIM3->SR &= ~TIM_SR_UIF;
        HAL_TIM_Base_Start_IT(htim3);
        return timed_out;
    } else {
        send_can_data_frame(can_handle1, TxHeader, can_packet->data_buffer, 3, command, data_size);
        send_can_data_frame(can_handle1, TxHeader, can_packet->data_buffer, 9, command, data_size);
        send_can_data_frame(can_handle2, TxHeader, can_packet->data_buffer, 1, command, data_size);
        send_can_data_frame(can_handle2, TxHeader, can_packet->data_buffer, 2, command, data_size);
        send_can_data_frame(can_handle3, TxHeader, can_packet->data_buffer, 10, command, data_size);
        send_can_data_frame(can_handle3, TxHeader, can_packet->data_buffer, 11, command, data_size);
        send_can_data_frame(can_handle1, TxHeader, can_packet->data_buffer, 0, command, data_size);
        send_can_data_frame(can_handle1, TxHeader, can_packet->data_buffer, 6, command, data_size);
        send_can_data_frame(can_handle2, TxHeader, can_packet->data_buffer, 7, command, data_size);
        send_can_data_frame(can_handle2, TxHeader, can_packet->data_buffer, 8, command, data_size);
        send_can_data_frame(can_handle3, TxHeader, can_packet->data_buffer, 4, command, data_size);
        send_can_data_frame(can_handle3, TxHeader, can_packet->data_buffer, 5, command, data_size);
        return 0;
    }
}

void send_command_data(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        CAN_Packet *can_packet, FDCAN_TxHeaderTypeDef *TxHeader, void *dataToSend)
{
    uint8_t command = can_packet->id;
    uint8_t data_size = can_packet->data_size;
//    uint8_t num_words = data_size/4;

    send_can_data_frame_buffer(can_handle1, TxHeader, dataToSend + 0 *data_size, 0 , command,  data_size, 1 << 0);
    send_can_data_frame_buffer(can_handle1, TxHeader, dataToSend + 3 *data_size, 3 , command,  data_size, 1 << 1);
    send_can_data_frame_buffer(can_handle1, TxHeader, dataToSend + 6 *data_size, 6 , command,  data_size, 1 << 2);
    send_can_data_frame_buffer(can_handle1, TxHeader, dataToSend + 9 *data_size, 9 , command,  data_size, 1 << 3);

    send_can_data_frame_buffer(can_handle2, TxHeader, dataToSend + 1 *data_size, 1 , command,  data_size, 1 << 0);
    send_can_data_frame_buffer(can_handle2, TxHeader, dataToSend + 2 *data_size, 2 , command,  data_size, 1 << 1);
    send_can_data_frame_buffer(can_handle2, TxHeader, dataToSend + 7 *data_size, 7 , command,  data_size, 1 << 2);
    send_can_data_frame_buffer(can_handle2, TxHeader, dataToSend + 8 *data_size, 8 , command,  data_size, 1 << 3);

    send_can_data_frame_buffer(can_handle3, TxHeader, dataToSend + 4 *data_size, 4 , command,  data_size, 1 << 0);
    send_can_data_frame_buffer(can_handle3, TxHeader, dataToSend + 5 *data_size, 5 , command,  data_size, 1 << 1);
    send_can_data_frame_buffer(can_handle3, TxHeader, dataToSend + 10*data_size, 10, command,  data_size, 1 << 2);
    send_can_data_frame_buffer(can_handle3, TxHeader, dataToSend + 11*data_size, 11, command,  data_size, 1 << 3);

    uint8_t cnt = 0;
    TIM4->CNT = 0;
    while (cnt < 10) {
        cnt = TIM4->CNT;
    }

    can_handle1->Instance->TXBAR = 0x0000000f;
    can_handle2->Instance->TXBAR = 0x0000000f;
    can_handle3->Instance->TXBAR = 0x0000000f;

}

void send_torque_position(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader, float32_t torques[], float32_t positions[], uint8_t foot_scheduler[], uint8_t update_position_targets, float32_t position_targets[]){

    if (foot_scheduler[0]==1){ // send torques to leg i

        send_can_data_frame_buffer(can_handle1, TxHeader, &torques[0], 0, COMMAND_SET_INPUT_TORQUE,  4, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, &torques[1], 1, COMMAND_SET_INPUT_TORQUE,  4, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, &torques[2], 2, COMMAND_SET_INPUT_TORQUE,  4, 1 << 1);
    } else if (update_position_targets) {

        for (uint8_t i=0; i<3; i++){
            memcpy(&position_targets[i*2], &positions[0], 4);
        }
        send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[0], 0, COMMAND_SET_INPUT_POSITION,  8, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[1], 1, COMMAND_SET_INPUT_POSITION,  8, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[2], 2, COMMAND_SET_INPUT_POSITION,  8, 1 << 1);
    }
    if (foot_scheduler[1]==1){ // send torques to leg i

        send_can_data_frame_buffer(can_handle1, TxHeader, &torques[3], 3, COMMAND_SET_INPUT_TORQUE, 4, 1 << 1);
        send_can_data_frame_buffer(can_handle3, TxHeader, &torques[4], 4, COMMAND_SET_INPUT_TORQUE, 4, 1 << 0);
        send_can_data_frame_buffer(can_handle3, TxHeader, &torques[5], 5, COMMAND_SET_INPUT_TORQUE, 4, 1 << 1);
    } else if (update_position_targets) {

        for (uint8_t i=3; i<6; i++){
            memcpy(&position_targets[i*2], &positions[3], 4);
        }
        send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[3], 3, COMMAND_SET_INPUT_POSITION,  8, 1 << 1);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[4], 4, COMMAND_SET_INPUT_POSITION,  8, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[5], 5, COMMAND_SET_INPUT_POSITION,  8, 1 << 1);
    }
    if (foot_scheduler[2]==1){ // send torques to leg i

        send_can_data_frame_buffer(can_handle1, TxHeader, &torques[6], 6, COMMAND_SET_INPUT_TORQUE,  4, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, &torques[7], 7, COMMAND_SET_INPUT_TORQUE,  4, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, &torques[8], 8, COMMAND_SET_INPUT_TORQUE,  4, 1 << 3);
    } else if (update_position_targets) {

        for (uint8_t i=6; i<9; i++){
            memcpy(&position_targets[i*2], &positions[6], 4);
        }
        send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[6], 6, COMMAND_SET_INPUT_POSITION,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[7], 7, COMMAND_SET_INPUT_POSITION,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[8], 8, COMMAND_SET_INPUT_POSITION,  8, 1 << 3);
    }
    if (foot_scheduler[2]==1){ // send torques to leg i

        send_can_data_frame_buffer(can_handle1, TxHeader, &torques[9] ,  9, COMMAND_SET_INPUT_TORQUE,  4, 1 << 3);
        send_can_data_frame_buffer(can_handle3, TxHeader, &torques[10], 10, COMMAND_SET_INPUT_TORQUE,  4, 1 << 2);
        send_can_data_frame_buffer(can_handle3, TxHeader, &torques[11], 11, COMMAND_SET_INPUT_TORQUE,  4, 1 << 3);
    } else if (update_position_targets) {

        for (uint8_t i=9; i<12; i++){
            memcpy(&position_targets[i*2], &positions[9], 4);
        }
        send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[9] ,  9, COMMAND_SET_INPUT_POSITION,  8, 1 << 3);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[10], 10, COMMAND_SET_INPUT_POSITION,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[11], 11, COMMAND_SET_INPUT_POSITION,  8, 1 << 3);
    }

    uint8_t cnt = 0;
    TIM4->CNT = 0;
    while (cnt < 10) {
        cnt = TIM4->CNT;
    }

    can_handle1->Instance->TXBAR = 0x0000000f;
    can_handle2->Instance->TXBAR = 0x0000000f;
    can_handle3->Instance->TXBAR = 0x0000000f;
}

void send_positions(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader, float32_t position_targets[]){

    send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[0], 0, COMMAND_SET_INPUT_POSITION,  8, 1 << 0);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[1], 1, COMMAND_SET_INPUT_POSITION,  8, 1 << 0);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[2], 2, COMMAND_SET_INPUT_POSITION,  8, 1 << 1);

    send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[3], 3, COMMAND_SET_INPUT_POSITION,  8, 1 << 1);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[4], 4, COMMAND_SET_INPUT_POSITION,  8, 1 << 0);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[5], 5, COMMAND_SET_INPUT_POSITION,  8, 1 << 1);

    send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[6], 6, COMMAND_SET_INPUT_POSITION,  8, 1 << 2);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[7], 7, COMMAND_SET_INPUT_POSITION,  8, 1 << 2);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[8], 8, COMMAND_SET_INPUT_POSITION,  8, 1 << 3);

    send_can_data_frame_buffer(can_handle1, TxHeader, &position_targets[9] ,  9, COMMAND_SET_INPUT_POSITION,  8, 1 << 3);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[10], 10, COMMAND_SET_INPUT_POSITION,  8, 1 << 2);
    send_can_data_frame_buffer(can_handle2, TxHeader, &position_targets[11], 11, COMMAND_SET_INPUT_POSITION,  8, 1 << 3);

    uint8_t cnt = 0;
    TIM4->CNT = 0;
    while (cnt < 10) {
        cnt = TIM4->CNT;
    }

    can_handle1->Instance->TXBAR = 0x0000000f;
    can_handle2->Instance->TXBAR = 0x0000000f;
    can_handle3->Instance->TXBAR = 0x0000000f;

}

void send_control_modes_for_torque_mode(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader, int8_t foot_scheduler_diff[]){

    int32_t torque_control_data[2];
    int32_t position_control_data[2];

    torque_control_data[0] = CONTROL_MODE_TORQUE;
    torque_control_data[1] = INPUT_MODE_PASSTHROUGH;

    position_control_data[0] = CONTROL_MODE_POSITION;
    position_control_data[1] = INPUT_MODE_PASSTHROUGH;

    if (foot_scheduler_diff[0] == 1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, torque_control_data, 0, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, torque_control_data, 1, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, torque_control_data, 2, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 1);

    } else if (foot_scheduler_diff[0] == -1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 0, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 1, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 0);
        send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 2, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 1);
    }

    if (foot_scheduler_diff[1] == 1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, torque_control_data, 3, COMMAND_SET_CONTROLLER_MODE, 8, 1 << 1);
        send_can_data_frame_buffer(can_handle3, TxHeader, torque_control_data, 4, COMMAND_SET_CONTROLLER_MODE, 8, 1 << 0);
        send_can_data_frame_buffer(can_handle3, TxHeader, torque_control_data, 5, COMMAND_SET_CONTROLLER_MODE, 8, 1 << 1);

    } else if (foot_scheduler_diff[1] == -1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 3, COMMAND_SET_CONTROLLER_MODE, 8, 1 << 1);
        send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 4, COMMAND_SET_CONTROLLER_MODE, 8, 1 << 0);
        send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 5, COMMAND_SET_CONTROLLER_MODE, 8, 1 << 1);
    }

    if (foot_scheduler_diff[2] == 1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, torque_control_data, 4, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, torque_control_data, 5, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, torque_control_data, 6, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 3);

    } else  if (foot_scheduler_diff[2] == -1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 4, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 5, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 6, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 3);
    }

    if (foot_scheduler_diff[3] == 1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, torque_control_data, 9 , COMMAND_SET_CONTROLLER_MODE,  8, 1 << 3);
        send_can_data_frame_buffer(can_handle3, TxHeader, torque_control_data, 10, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle3, TxHeader, torque_control_data, 11, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 3);

    } else if (foot_scheduler_diff[3] == -1) {
        send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 9 , COMMAND_SET_CONTROLLER_MODE,  8, 1 << 3);
        send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 10, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 2);
        send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 11, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 3);
    }

    uint8_t cnt = 0;
    TIM4->CNT = 0;
    while (cnt < 10) {
        cnt = TIM4->CNT;
    }

    can_handle1->Instance->TXBAR = 0x0000000f;
    can_handle2->Instance->TXBAR = 0x0000000f;
    can_handle3->Instance->TXBAR = 0x0000000f;
}

void send_control_modes_for_position_mode(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader){

    int32_t position_control_data[2];

    position_control_data[0] = CONTROL_MODE_POSITION;
    position_control_data[1] = INPUT_MODE_PASSTHROUGH;

    send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 0, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 0);
    send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 1, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 0);
    send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 2, COMMAND_SET_CONTROLLER_MODE,  8, 1 << 1);

    send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 3, COMMAND_SET_CONTROLLER_MODE, 4, 1 << 1);
    send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 4, COMMAND_SET_CONTROLLER_MODE, 4, 1 << 0);
    send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 5, COMMAND_SET_CONTROLLER_MODE, 4, 1 << 1);

    send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 4, COMMAND_SET_CONTROLLER_MODE,  4, 1 << 2);
    send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 5, COMMAND_SET_CONTROLLER_MODE,  4, 1 << 2);
    send_can_data_frame_buffer(can_handle2, TxHeader, position_control_data, 6, COMMAND_SET_CONTROLLER_MODE,  4, 1 << 3);

    send_can_data_frame_buffer(can_handle1, TxHeader, position_control_data, 9 , COMMAND_SET_CONTROLLER_MODE,  4, 1 << 3);
    send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 10, COMMAND_SET_CONTROLLER_MODE,  4, 1 << 2);
    send_can_data_frame_buffer(can_handle3, TxHeader, position_control_data, 11, COMMAND_SET_CONTROLLER_MODE,  4, 1 << 3);

    uint8_t cnt = 0;
    TIM4->CNT = 0;
    while (cnt < 10) {
        cnt = TIM4->CNT;
    }

    can_handle1->Instance->TXBAR = 0x0000000f;
    can_handle2->Instance->TXBAR = 0x0000000f;
    can_handle3->Instance->TXBAR = 0x0000000f;

}

uint8_t wait_and_receive_packets(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
                              CAN_Packet *can_packet, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t CAN_RxData[], uint32_t *CAN_Received_Datas,
                              uint32_t *error_codes) {
    uint8_t command = can_packet->id;
//    uint8_t data_size = can_packet->data_size;
    uint8_t timed_out = 0;

    TIM4->CNT = 0;

    uint32_t current_time = 0;

    while (1) {
        uint8_t can1_fill_level = HAL_FDCAN_GetRxFifoFillLevel(can_handle1, FDCAN_RX_FIFO0);
        uint8_t can2_fill_level = HAL_FDCAN_GetRxFifoFillLevel(can_handle2, FDCAN_RX_FIFO0);
        uint8_t can3_fill_level = HAL_FDCAN_GetRxFifoFillLevel(can_handle3, FDCAN_RX_FIFO0);
        current_time = TIM4->CNT;

        if (can1_fill_level >= 4 && can2_fill_level >= 4 && can3_fill_level >= 4) {
            break;
        }
        if (current_time > 500) {
            if (can1_fill_level < 2) {
                timed_out |= 1 << 2;
            }
            if (can2_fill_level < 2) {
                timed_out |= 1 << 1;
            }
            if (can3_fill_level < 2) {
                timed_out |= 1 << 5;
            }
            break;
        }
    }

    receive_can_data_frame(can_handle1, RxHeader, CAN_RxData, CAN_Received_Datas, command, error_codes);
    receive_can_data_frame(can_handle2, RxHeader, CAN_RxData, CAN_Received_Datas, command, error_codes);
    receive_can_data_frame(can_handle3, RxHeader, CAN_RxData, CAN_Received_Datas, command, error_codes);
    return timed_out;
}

uint8_t get_estimates(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3, CAN_Packet *can_packet,
                      FDCAN_TxHeaderTypeDef *TxHeader, void *TxData, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t CAN_RxData[], uint32_t CAN_Received_Datas[], uint32_t *error_codes, TIM_HandleTypeDef *htim3) {
    uint8_t timed_out = 0;
    can_packet->RTR = 1;
    can_packet->id = 9;

    timed_out = send_can_command_to_all_controllers(can_handle1, can_handle2, can_handle3, can_packet, TxHeader, TxData, RxHeader, CAN_RxData, CAN_Received_Datas, error_codes, htim3);


    return timed_out;
}

void calculate_estimates(float_t position_estimates[], float_t velocity_estimates[], uint32_t CAN_Received_Datas[]) {
    for (uint8_t node = 0; node < 12; node++) {
        //        memcpy(position_estimates + node*4, CAN_Received_Datas + node*4, 4);
        //        memcpy(velocity_estimates + node*4, CAN_Received_Datas + node*4 + 12*4, 4);

        memcpy(&position_estimates[node], &CAN_Received_Datas[node], 4);
        memcpy(&velocity_estimates[node], &CAN_Received_Datas[node + 12], 4);


        position_estimates[node] *= angle_multiplier[node];
        velocity_estimates[node] *= angle_multiplier[node];
    }
}

void wait_until_no_pending_messages(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3){
    uint32_t pending1 = can_handle1->Instance->TXBRP;
    uint32_t pending2 = can_handle2->Instance->TXBRP;
    uint32_t pending3 = can_handle3->Instance->TXBRP;

    while(pending1 || pending2 || pending3){
        pending1 = can_handle1->Instance->TXBRP;
        pending2 = can_handle2->Instance->TXBRP;
        pending3 = can_handle3->Instance->TXBRP;
    }
}




