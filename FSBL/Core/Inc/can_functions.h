/*
 * can_functions.h
 *
 *  Created on: 1 Mar 2026
 *      Author: tomas
 */
#pragma once
#ifndef INC_CAN_FUNCTIONS_H_
#define INC_CAN_FUNCTIONS_H_

#define CONTROL_MODE_TORQUE             0x1U
#define CONTROL_MODE_POSITION           0x3U

#define INPUT_MODE_PASSTHROUGH          0x1U
#define INPUT_MODE_POS_FILTER           0x3U

#define AXIS_STATE_IDLE                 0x1U
#define AXIS_STATE_CLOSED_LOOP          0x8U

#define COMMAND_SET_CONTROLLER_MODE     0xBU
#define COMMAND_SET_INPUT_POSITION      0xCU
#define COMMAND_SET_INPUT_TORQUE        0xEU

#include "stm32n6xx_hal.h"
#include <math.h>
#include <string.h>
#include "arm_math.h"

extern const float32_t angle_multiplier[12];

typedef struct {
    uint8_t id;
    uint8_t RTR;
    uint8_t data_size;
    uint8_t data_buffer[8];
} CAN_Packet;

void reboot_odrives(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3, FDCAN_TxHeaderTypeDef *TxHeader, void *dataToSend);
uint8_t run_leg_angle_calibration(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
                               CAN_Packet *can_packet, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, FDCAN_RxHeaderTypeDef *RxHeader,
                               uint8_t CAN_RxData[], uint32_t CAN_Received_Datas[], uint32_t *error_codes, TIM_HandleTypeDef *htim3, volatile uint8_t* estimates_flag);

void send_can_remote_frame(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node, uint8_t cmd);

HAL_StatusTypeDef send_can_remote_frame_buffer(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node,
                                               uint8_t cmd, uint32_t buffer_index);

HAL_StatusTypeDef send_can_data_frame(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node, uint8_t cmd,
                                      uint8_t data_length);

HAL_StatusTypeDef send_can_data_frame_buffer(FDCAN_HandleTypeDef *can_handle, FDCAN_TxHeaderTypeDef *TxHeader, void* TxData, uint8_t node, uint8_t cmd,
                                      uint8_t data_length, uint32_t buffer_index);

void receive_can_data_frame(FDCAN_HandleTypeDef *can_handle, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t RxData[], uint32_t *CAN_Received_Datas,
                            uint8_t command, uint32_t *error_codes);

uint8_t send_can_command_to_all_controllers(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
                                            CAN_Packet *can_packet, FDCAN_TxHeaderTypeDef *TxHeader, void *TxData, FDCAN_RxHeaderTypeDef *RxHeader,
                                            uint8_t CAN_RxData[], uint32_t *CAN_Received_Datas, uint32_t *error_codes, TIM_HandleTypeDef *htim3);

uint8_t get_estimates(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3, CAN_Packet *can_packet,
                      FDCAN_TxHeaderTypeDef *TxHeader, void *TxData, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t CAN_RxData[], uint32_t CAN_Received_Datas[],
                      uint32_t *error_codes, TIM_HandleTypeDef *htim3);

uint8_t wait_and_receive_packets(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
                              CAN_Packet *can_packet, FDCAN_RxHeaderTypeDef *RxHeader, uint8_t CAN_RxData[], uint32_t CAN_Received_Datas[],
                              uint32_t *error_codes);

void calculate_estimates(float_t position_estimates[], float_t velocity_estimates[], uint32_t CAN_Received_Datas[]);

void send_command_data(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        CAN_Packet *can_packet, FDCAN_TxHeaderTypeDef *TxHeader, void *dataToSend);

void send_torque_position(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader, float32_t *torques, float32_t *positions, uint8_t foot_scheduler[], uint8_t update_position_targets, float32_t position_targets[]);
void send_positions(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader, float32_t position_targets[]);
void send_control_modes_for_torque_mode(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader, int8_t foot_scheduler_diff[]);
void send_control_modes_for_position_mode(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3,
        FDCAN_TxHeaderTypeDef *TxHeader);
void wait_until_no_pending_messages(FDCAN_HandleTypeDef *can_handle1, FDCAN_HandleTypeDef *can_handle2, FDCAN_HandleTypeDef *can_handle3);

#endif /* INC_CAN_FUNCTIONS_H_ */
