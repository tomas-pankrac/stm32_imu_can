/*
 * can_functions.h
 *
 *  Created on: 1 Mar 2026
 *      Author: tomas
 */

#ifndef INC_CAN_FUNCTIONS_H_
#define INC_CAN_FUNCTIONS_H_

#include "stm32n6xx_hal.h"
#include <math.h>
#include <string.h>
#include "arm_math.h"



typedef struct {
    uint8_t id;
    uint8_t RTR;
    uint8_t data_size;
    uint8_t data_buffer[8];
} CAN_Packet;

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

#endif /* INC_CAN_FUNCTIONS_H_ */
