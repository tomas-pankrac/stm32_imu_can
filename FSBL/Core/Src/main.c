/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "Fusion.h"
#include "endpoint_calculation.h"
#include "kalman.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_BLOCKSIZE 1024
#define IMU_CS_BANK GPIOF
#define IMU_CS_PIN GPIO_PIN_2
#define MAG_CS_BANK GPIOF
#define MAG_CS_PIN GPIO_PIN_8
#define CLK_WAIT 0x1FU
#define SAMPLE_RATE (833) // replace with actual sample rate

#define ACCELEROMETER_CONSTANT ((double_t) 16.0 / 65536)
#define GYROSCOPE_CONSTANT ((double_t) 4000.0 / 65536)
#define MAGNETOMETER_CONSTANT (0.15)
#define ADC_CONSTANT ((double_t) 21.4 / 1.4 / 4096 * 3.3)





/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;
DMA_HandleTypeDef handle_GPDMA1_Channel3;
DMA_HandleTypeDef handle_GPDMA1_Channel2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;

XSPI_HandleTypeDef hxspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void SystemIsolation_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_XSPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;


uint8_t TX_Buffer[16];
uint8_t RX_Buffer[16];

uint8_t TX_Buffer_1[512];
uint8_t RX_Buffer_1[512];

float_t position_estimates[12];
float_t velocity_estimates[12];

//union positionEstimates {
//    float_t position_estimates_float[12];
//    uint32_t position_estimates_int[12];
//};
//
//union velocityEstimates {
//    float_t velocity_estimates_float[12];
//    uint32_t velocity_estimates_int[12];
//};
//
//union positionEstimates position_estimates;
//union positionEstimates velocity_estimates;

int16_t accel_x;
int16_t accel_y;
int16_t accel_z;

int16_t gyro_x;
int16_t gyro_y;
int16_t gyro_z;

int16_t mag_x;
int16_t mag_y;
int16_t mag_z;
int16_t mag_temp;

uint32_t timestamp;
uint32_t previousTimestamp;
float_t deltaTime;

int16_t accelerometer_data[3];
double_t acceleration[3];
int16_t gyroscope_data[3];
int16_t magnetometer_data[3];
float_t euler_orientation[3];
double_t omega[3];

float32_t battery_voltage;

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t CAN_TxData[8];
uint8_t CAN_RxData[8];
uint32_t CAN_Received_Datas[24];
uint32_t CAN_Command_Datas[24];
uint32_t TxMailbox;
uint8_t can_timeout_flags;
uint8_t send_can_command_flag = 0;
uint32_t error_codes[12];
uint8_t can_state_machine=1;
uint8_t heartbeat_receive_counter=0;

float32_t forces[12] = {20,0,0,20,0,0,20,0,0,20,0,0};
float32_t position_targets[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t foot_scheduler[4] = {0,0,0,0};
int8_t foot_scheduler_diff[4] = {0,0,0,0};
float32_t body_velocity_estimate[3] = {0,0,0};
uint8_t new_calculated_data_received=0;
uint8_t new_calculated_data_buffer[100];
uint8_t new_calculated_data_counter=0;
float32_t foot_step_landing_locations[12]={};
float32_t shoulder_vectors[12] = {0.2, 0.065, 0,-0.2, 0.065, 0,0.2, -0.065, 0,-0.2, -0.065, 0,};
float32_t lift_leg_trajectory[15][12];
float32_t lift_leg_trajectory_angles[15][12];
uint8_t lift_leg_index = 0;
uint32_t lift_leg_timer_counter = 20000;

CAN_Packet can_packet_command = {0x9, 0, 0, {0, 0, 0, 0, 0, 0, 0, 0}};
CAN_Packet can_packet_estimates = {0x9, 1, 8, {0, 0, 0, 0, 0, 0, 0, 0}};

HAL_StatusTypeDef status;

const FusionMatrix gyroscopeMisalignment = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
const FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}};

const FusionMatrix accelerometerMisalignment = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
const FusionVector accelerometerSensitivity = {{1.0f, 1.0f, 1.0f}};
const FusionVector accelerometerOffset = {{0.0f, 0.0f, 0.0f}};

const FusionMatrix softIronMatrix = {{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}};
// const FusionVector hardIronOffset = {50.0f, -200.0f, 250.0f};
const FusionVector hardIronOffset = {{0.0f, 0.0f, 0.0f}};

// Set AHRS settings
const FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 4000.0f, /* replace with actual gyroscope range */
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
};

FusionBias bias;
FusionAhrs ahrs;

uint8_t fs[4] = {1, 1, 1, 1};
float32_t var_R = 1e-5;
float32_t var_V = 1e-5;
float32_t var_A = 1e-5;
float32_t var_W = 1e-5;

float32_t leg_angles[12] = {0.1, 0.8, -1.59, 0.1, 0.8, -1.59, 0.1, 0.8, -1.59, 0.1, 0.8, -1.59};

volatile uint8_t estimates_flag = 0;

uint32_t sync1_time = 0;
uint32_t sync2_time = 0;

TX_Data_t data;

uint8_t performance_mode = 0;

uint8_t parsed_command = 0;
uint8_t parsed_data_size = 0;

uint8_t estop = 0;

uint8_t parsed_control_mode = CONTROL_MODE_POSITION;
uint8_t parsed_axis_state = AXIS_STATE_IDLE;
uint8_t parsed_input_mode = INPUT_MODE_PASSTHROUGH;
uint8_t control_mode = 0;
uint8_t axis_state = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
    SystemInit();
    init_matrices();
    gen_F(0.005);
    gen_G(0.005);
    gen_Q(Q, var_A, var_W, 0.005);
    calculate_Pp(&F_inst, &Ft_inst, &P_inst, &FP_inst, &Q_inst, &Pp_inst);
    init_ep_matrices();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_XSPI2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();
  SystemIsolation_Config();
  /* USER CODE BEGIN 2 */

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim9);
    //    HAL_TIM_Base_Start_IT(&htim3);

    initialize_imu_mag(&hspi5, TX_Buffer, RX_Buffer);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

    uint8_t can_node = 3;
    uint8_t cmd_id = 0x9;

    TxHeader.Identifier = (can_node << 5) | cmd_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_REMOTE_FRAME; // FDCAN_REMOTE_FRAME, FDCAN_DATA_FRAME
    TxHeader.DataLength = 8;

    CAN_TxData[0] = 50;
    CAN_TxData[1] = 0xAA;

    initialize_ahrs(&bias, &ahrs);

    handle_GPDMA1_Channel0.Init.Request = GPDMA1_REQUEST_SPI1_RX;
    handle_GPDMA1_Channel1.Init.Request = GPDMA1_REQUEST_SPI1_TX;

    reboot_odrives(&hfdcan1, &hfdcan2, &hfdcan3, &TxHeader, CAN_TxData);

    run_leg_angle_calibration(&hfdcan1, &hfdcan2, &hfdcan3, &can_packet_estimates, &TxHeader, CAN_TxData, &RxHeader, CAN_RxData, CAN_Received_Datas, error_codes, &htim3, &estimates_flag);

    can_packet_command.id = 0xe;
    can_packet_command.RTR = 0;

    HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {

        // Set test gpios from triggering
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
//        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);

        // Zero estimates flag and timers
        uint8_t update_position_targets = 0;
        estimates_flag = 0;
        TIM5->CNT = 0;
        TIM9->CNT = 0;

        // Prepare for motor position and velocity estimates
        can_packet_estimates.id = 0x9;
        can_packet_estimates.RTR = 0x1;
        can_packet_estimates.data_size = 0x8;

        // Readout motor position and velocity estimates
        can_timeout_flags = get_estimates(&hfdcan1, &hfdcan2, &hfdcan3, &can_packet_estimates, &TxHeader, &CAN_TxData, &RxHeader, CAN_RxData, CAN_Received_Datas, error_codes, &htim3);
        TIM3->CNT = 1;
        hfdcan1.Instance->TXBAR = 0x00000003;
        hfdcan2.Instance->TXBAR = 0x00000003;
        hfdcan3.Instance->TXBAR = 0x00000003;

        // Readout accelerometer, gyroscope and magnetometer data
        readout_imu(&hspi5, TX_Buffer, RX_Buffer, accelerometer_data, gyroscope_data, magnetometer_data, &previousTimestamp, &timestamp, &deltaTime, &mag_temp);

        // Update orientation based on the IMU data
        update_ahrs(&bias, &ahrs, accelerometer_data, gyroscope_data, magnetometer_data, euler_orientation, deltaTime);

        // Readout ADC
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1);
        uint16_t AD_RES = HAL_ADC_GetValue(&hadc1);
        battery_voltage = AD_RES * ADC_CONSTANT;

        ///////////////////// First sync point - wait for the motor position and velocity estimates to finish reading out
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

        uint32_t timer9 = TIM9->CNT;

        while(estimates_flag == 0){
            timer9 = TIM9->CNT;
            if(timer9 > 1000){
                break;
            }
            continue;
        }

        // Postprocess and copy the motor position and velocity estimates
        calculate_estimates(position_estimates, velocity_estimates, CAN_Received_Datas);

        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

        // Calculate the endpoint location and velocities and the torque
        update_endpoints(4, position_estimates, velocity_estimates, forces);
        check_endpoint_limits(&estop);

        // Calculate new footstep landing positions if fsd == -1
//        euler_orientation[2] = M_PI_4;
//        foot_scheduler_diff[0] = -1;
//        foot_scheduler_diff[1] = -1;
//        foot_scheduler_diff[2] = -1;
//        foot_scheduler_diff[3] = -1;
//        body_velocity_estimate[0] = 0.5;
//        body_velocity_estimate[1] = 0.2;
//        body_velocity_estimate[2] = 0;
//        omega[2] = 0.25;

        if((foot_scheduler_diff[0] == -1 || foot_scheduler_diff[1] == -1 || foot_scheduler_diff[2] == -1 || foot_scheduler_diff[3] == -1) && new_calculated_data_received && control_mode == CONTROL_MODE_TORQUE){
            body_velocity_estimate[0] = 0.5;
            generate_footstep_landing_location(body_velocity_estimate, foot_scheduler_diff, foot_step_landing_locations, shoulder_vectors, euler_orientation[2], omega[2], 0.3);
            generate_lift_leg_trajectory(foot_scheduler_diff, foot_step_landing_locations, endpoints, lift_leg_trajectory, lift_leg_trajectory_angles);
            TIM2->CNT = 0;
            lift_leg_index = 0;
            lift_leg_timer_counter = 0;
        }

        uint32_t lift_leg_timer = TIM2->CNT;
        if(lift_leg_timer >= lift_leg_timer_counter) {
            lift_leg_index += 1;
            lift_leg_timer_counter += 20000;
            update_position_targets = 1;
        }

        // If we don't run performance mode, we need to readout also the torques from the controllers
        performance_mode = 1;
        if(!performance_mode){
            estimates_flag = 0;

            can_packet_estimates.id = 0x14;
            can_packet_estimates.RTR = 0x1;
            can_packet_estimates.data_size = 0x8;

            send_can_command_to_all_controllers(&hfdcan1, &hfdcan2, &hfdcan3, &can_packet_estimates, &TxHeader, &CAN_TxData, &RxHeader, CAN_RxData, CAN_Received_Datas, error_codes, &htim3);
            hfdcan1.Instance->TXBAR = 0x00000003;
            hfdcan2.Instance->TXBAR = 0x00000003;
            hfdcan3.Instance->TXBAR = 0x00000003;

            while(estimates_flag == 0){
                timer9 = TIM9->CNT;
                if(timer9 > 2000){
                    break;
                }
                continue;
            }
        }

        // Send either position or torque inputs
        if(parsed_control_mode == CONTROL_MODE_TORQUE) {
            float32_t torques[] = {0,0,0,0,0,0,0,0,0,0,0,0};
            send_torque_position(&hfdcan1, &hfdcan2, &hfdcan3, &TxHeader, torques, lift_leg_trajectory_angles[lift_leg_index], foot_scheduler, update_position_targets, lift_leg_trajectory_angles[lift_leg_index]);
        }
        else if (parsed_control_mode == CONTROL_MODE_POSITION){
            send_positions(&hfdcan1, &hfdcan2, &hfdcan3, &TxHeader, position_targets);
        }

        // Update data based on calculations
        update_data_structure();

        // Send telemetry and receive control data
        send_telemetry(&hspi1, &data, sizeof(data), TX_Buffer_1, RX_Buffer_1);


        wait_until_no_pending_messages(&hfdcan1, &hfdcan2, &hfdcan3);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

//        parsed_control_mode = 3;


        // Check if CONTROL_MODE has changed, if so send a CAN message to the controllers
//        can_packet_command.RTR = 0;
//        if((parsed_control_mode != control_mode) && (parsed_control_mode == CONTROL_MODE_TORQUE || parsed_control_mode == CONTROL_MODE_POSITION)){
//
//            can_packet_command.data_size = 0x8;//parsed_data_size;
//            can_packet_command.id = 0xb;//parsed_command;
//            control_mode = parsed_control_mode;
//            for(uint8_t i=0; i<12; i++){
//                CAN_Command_Datas[i*2] = (uint32_t) control_mode;
//                CAN_Command_Datas[i*2+1] = (uint32_t) parsed_input_mode;
//            }
//
//            send_command_data(&hfdcan1, &hfdcan2, &hfdcan3, &can_packet_command, &TxHeader, &CAN_Command_Datas);
//            wait_until_no_pending_messages(&hfdcan1, &hfdcan2, &hfdcan3);
//        }


        can_packet_command.RTR = 0;
        if(parsed_control_mode == CONTROL_MODE_TORQUE){
            control_mode = parsed_control_mode;
            send_control_modes_for_torque_mode(&hfdcan1, &hfdcan2, &hfdcan3, &TxHeader, foot_scheduler_diff);
            wait_until_no_pending_messages(&hfdcan1, &hfdcan2, &hfdcan3);
        } else if (parsed_control_mode == CONTROL_MODE_POSITION && parsed_control_mode != control_mode){
            control_mode = parsed_control_mode;
            send_control_modes_for_position_mode(&hfdcan1, &hfdcan2, &hfdcan3, &TxHeader);
            wait_until_no_pending_messages(&hfdcan1, &hfdcan2, &hfdcan3);
        }

//        parsed_axis_state = 1;
        // Check if AXIS_STATE has changed, if so send a CAN message to the controllers
        can_packet_command.RTR = 0;
        if(((parsed_axis_state != axis_state) && (parsed_axis_state == AXIS_STATE_CLOSED_LOOP || parsed_axis_state == AXIS_STATE_IDLE)) || estop){

            can_packet_command.data_size = 0x4;//parsed_data_size;
            can_packet_command.id = 0x7;//parsed_command;
            axis_state = parsed_axis_state;
            if(estop){
                axis_state = AXIS_STATE_IDLE;
            }
            for(uint8_t i=0; i<12; i++){
                CAN_Command_Datas[i] = (uint32_t) axis_state;
            }

            send_command_data(&hfdcan1, &hfdcan2, &hfdcan3, &can_packet_command, &TxHeader, &CAN_Command_Datas);
            wait_until_no_pending_messages(&hfdcan1, &hfdcan2, &hfdcan3);
        }

        wait_until_no_pending_messages(&hfdcan1, &hfdcan2, &hfdcan3);

//        if(heartbeat_receive_counter >= 42){
//            control_mode = 0;
//            axis_state = 0;
//            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
////            TIM9->CNT = 0;
////            timer9 = 0;
////            while(timer9 < 400){
////                timer9 = TIM9->CNT;
////            }
//            heartbeat_receive_counter = 0;
//
//        }
//        else{
//            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
//            heartbeat_receive_counter += 1;
//        }

        // Poll IMU to check if there is new data available
        uint32_t flags = poll_for_imu_new_data(&hspi5, TX_Buffer, RX_Buffer);
        while(flags < 3){
            flags = poll_for_imu_new_data(&hspi5, TX_Buffer, RX_Buffer);
            timer_wait2(10);
        }

        sync2_time = TIM5->CNT;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}
/* USER CODE BEGIN CLK 1 */
/* USER CODE END CLK 1 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the System Power Supply
  */
  if (HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable HSI */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Get current CPU/System buses clocks configuration and if necessary switch
 to intermediate HSI clock to ensure target clock can be set
  */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct);
  if ((RCC_ClkInitStruct.CPUCLKSource == RCC_CPUCLKSOURCE_IC1) ||
     (RCC_ClkInitStruct.SYSCLKSource == RCC_SYSCLKSOURCE_IC2_IC6_IC11))
  {
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_CPUCLK | RCC_CLOCKTYPE_SYSCLK);
    RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_HSI;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL1.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL1.PLLM = 1;
  RCC_OscInitStruct.PLL1.PLLN = 25;
  RCC_OscInitStruct.PLL1.PLLFractional = 0;
  RCC_OscInitStruct.PLL1.PLLP1 = 1;
  RCC_OscInitStruct.PLL1.PLLP2 = 1;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL2.PLLM = 1;
  RCC_OscInitStruct.PLL2.PLLN = 25;
  RCC_OscInitStruct.PLL2.PLLFractional = 0;
  RCC_OscInitStruct.PLL2.PLLP1 = 1;
  RCC_OscInitStruct.PLL2.PLLP2 = 1;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL4.PLLM = 1;
  RCC_OscInitStruct.PLL4.PLLN = 25;
  RCC_OscInitStruct.PLL4.PLLFractional = 0;
  RCC_OscInitStruct.PLL4.PLLP1 = 1;
  RCC_OscInitStruct.PLL4.PLLP2 = 1;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_CPUCLK|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK5
                              |RCC_CLOCKTYPE_PCLK4;
  RCC_ClkInitStruct.CPUCLKSource = RCC_CPUCLKSOURCE_IC1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_IC2_IC6_IC11;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5CLKDivider = RCC_APB5_DIV1;
  RCC_ClkInitStruct.IC1Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC1Selection.ClockDivider = 2;
  RCC_ClkInitStruct.IC2Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC2Selection.ClockDivider = 4;
  RCC_ClkInitStruct.IC6Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC6Selection.ClockDivider = 256;
  RCC_ClkInitStruct.IC11Selection.ClockSelection = RCC_ICCLKSOURCE_PLL1;
  RCC_ClkInitStruct.IC11Selection.ClockDivider = 4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

    //  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_4;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 5;
  hfdcan1.Init.NominalTimeSeg1 = 26;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 15;
  hfdcan1.Init.DataTimeSeg1 = 16;
  hfdcan1.Init.DataTimeSeg2 = 15;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 4;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 5;
  hfdcan2.Init.NominalTimeSeg1 = 26;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 15;
  hfdcan2.Init.DataTimeSeg1 = 16;
  hfdcan2.Init.DataTimeSeg2 = 15;
  hfdcan2.Init.MessageRAMOffset = 1024;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 8;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 4;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Error_Handler();
    }

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 1;
  hfdcan3.Init.NominalSyncJumpWidth = 5;
  hfdcan3.Init.NominalTimeSeg1 = 26;
  hfdcan3.Init.NominalTimeSeg2 = 5;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 15;
  hfdcan3.Init.DataTimeSeg1 = 16;
  hfdcan3.Init.DataTimeSeg2 = 15;
  hfdcan3.Init.MessageRAMOffset = 2048;
  hfdcan3.Init.StdFiltersNbr = 0;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.RxFifo0ElmtsNbr = 8;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 4;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Error_Handler();
    }

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 0x7;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi5.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi5.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 350;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 39;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 399;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief XSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_XSPI2_Init(void)
{

  /* USER CODE BEGIN XSPI2_Init 0 */

  /* USER CODE END XSPI2_Init 0 */

  XSPIM_CfgTypeDef sXspiManagerCfg = {0};

  /* USER CODE BEGIN XSPI2_Init 1 */

  /* USER CODE END XSPI2_Init 1 */
  /* XSPI2 parameter configuration*/
  hxspi2.Instance = XSPI2;
  hxspi2.Init.FifoThresholdByte = 1;
  hxspi2.Init.MemoryMode = HAL_XSPI_SINGLE_MEM;
  hxspi2.Init.MemoryType = HAL_XSPI_MEMTYPE_MICRON;
  hxspi2.Init.MemorySize = HAL_XSPI_SIZE_16B;
  hxspi2.Init.ChipSelectHighTimeCycle = 1;
  hxspi2.Init.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE;
  hxspi2.Init.ClockMode = HAL_XSPI_CLOCK_MODE_0;
  hxspi2.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
  hxspi2.Init.ClockPrescaler = 0;
  hxspi2.Init.SampleShifting = HAL_XSPI_SAMPLE_SHIFT_NONE;
  hxspi2.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
  hxspi2.Init.ChipSelectBoundary = HAL_XSPI_BONDARYOF_NONE;
  hxspi2.Init.MaxTran = 0;
  hxspi2.Init.Refresh = 0;
  hxspi2.Init.MemorySelect = HAL_XSPI_CSSEL_NCS1;
  if (HAL_XSPI_Init(&hxspi2) != HAL_OK)
  {
    Error_Handler();
  }
  sXspiManagerCfg.nCSOverride = HAL_XSPI_CSSEL_OVR_NCS1;
  sXspiManagerCfg.IOPort = HAL_XSPIM_IOPORT_2;
  sXspiManagerCfg.Req2AckTime = 1;
  if (HAL_XSPIM_Config(&hxspi2, &sXspiManagerCfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN XSPI2_Init 2 */

  /* USER CODE END XSPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPION_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF6 PF2 PF8 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_2|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief RIF Initialization Function
  * @param None
  * @retval None
  */
  static void SystemIsolation_Config(void)
{

/* USER CODE BEGIN RIF_Init 0 */

/* USER CODE END RIF_Init 0 */

  /* set all required IPs as secure privileged */
  __HAL_RCC_RIFSC_CLK_ENABLE();

  /* RIF-Aware IPs Config */

  /* set up GPDMA configuration */
  /* set GPDMA1 channel 0 used by SPI1 */
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0,DMA_CHANNEL_SEC|DMA_CHANNEL_PRIV|DMA_CHANNEL_SRC_SEC|DMA_CHANNEL_DEST_SEC)!= HAL_OK )
  {
    Error_Handler();
  }
  /* set GPDMA1 channel 1 used by SPI1 */
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1,DMA_CHANNEL_SEC|DMA_CHANNEL_PRIV|DMA_CHANNEL_SRC_SEC|DMA_CHANNEL_DEST_SEC)!= HAL_OK )
  {
    Error_Handler();
  }
  /* set GPDMA1 channel 2 used by SPI5 */
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel2,DMA_CHANNEL_SEC|DMA_CHANNEL_PRIV|DMA_CHANNEL_SRC_SEC|DMA_CHANNEL_DEST_SEC)!= HAL_OK )
  {
    Error_Handler();
  }
  /* set GPDMA1 channel 3 used by SPI5 */
  if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel3,DMA_CHANNEL_SEC|DMA_CHANNEL_PRIV|DMA_CHANNEL_SRC_SEC|DMA_CHANNEL_DEST_SEC)!= HAL_OK )
  {
    Error_Handler();
  }

/* USER CODE BEGIN RIF_Init 1 */

/* USER CODE END RIF_Init 1 */
/* USER CODE BEGIN RIF_Init 2 */

/* USER CODE END RIF_Init 2 */

}

/* USER CODE BEGIN 4 */

void update_data_structure(){

    memcpy(&data.orientation, &euler_orientation, sizeof(float32_t)*3);
    memcpy(&data.omega, &omega, sizeof(double_t)*3);
    memcpy(&data.acceleration, &acceleration, sizeof(double_t)*3);

    memcpy(&data.leg_angles, &position_estimates, sizeof(float32_t)*12);

    memcpy(&data.endpoints, &endpoints, sizeof(float32_t)*12);
    memcpy(&data.endpoints_velocity, &endpoints_velocity, sizeof(float32_t)*12);

    memcpy(&data.can_data_low, &CAN_Received_Datas, sizeof(uint32_t)*12);
    memcpy(&data.can_data_high, &CAN_Received_Datas[12], sizeof(uint32_t)*12);

    memcpy(&data.magnetometer_data, &magnetometer_data, sizeof(int16_t)*3);

    memcpy(&data.error_codes, &error_codes, sizeof(uint32_t)*12);

    data.deltaTime = deltaTime;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if (htim == &htim3) {
        switch (can_state_machine){
        case 0:
            hfdcan1.Instance->TXBAR = 0x00000003;
            hfdcan2.Instance->TXBAR = 0x00000003;
            hfdcan3.Instance->TXBAR = 0x00000003;
            can_state_machine += 1;
            TIM3->CNT = 1;
            TIM3->SR &= ~TIM_SR_UIF;
            HAL_TIM_Base_Start_IT(htim);
//            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            break;

        case 1:
            hfdcan1.Instance->TXBAR = 0x0000000c;
            hfdcan2.Instance->TXBAR = 0x0000000c;
            hfdcan3.Instance->TXBAR = 0x0000000c;
            can_state_machine = 4;
            TIM3->CNT = 1;
            TIM3->SR &= ~TIM_SR_UIF;
            HAL_TIM_Base_Start_IT(htim);
//            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
            break;

        case 2:
            hfdcan1.Instance->TXBAR = 0x00000004;
            hfdcan2.Instance->TXBAR = 0x00000004;
            hfdcan3.Instance->TXBAR = 0x00000004;
            can_state_machine += 1;
            TIM3->CNT = 1;
            TIM3->SR &= ~TIM_SR_UIF;
            HAL_TIM_Base_Start_IT(htim);
            break;

        case 3:
            hfdcan1.Instance->TXBAR = 0x00000008;
            hfdcan2.Instance->TXBAR = 0x00000008;
            hfdcan3.Instance->TXBAR = 0x00000008;
            can_state_machine += 1;
            TIM3->CNT = 1;
            TIM3->SR &= ~TIM_SR_UIF;
            HAL_TIM_Base_Start_IT(htim);

            break;

        case 4:
            can_timeout_flags = wait_and_receive_packets(&hfdcan1, &hfdcan2, &hfdcan3, &can_packet_estimates, &RxHeader, CAN_RxData, CAN_Received_Datas, error_codes);
            can_state_machine = 1;
            TIM3->SR &= ~TIM_SR_UIF;
            HAL_TIM_Base_Stop_IT(htim);
            estimates_flag = 1;
//            HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
            break;
        }
    }
}

void cache_clean(void *addr, size_t size)
{
    uint32_t start = (uint32_t)addr & ~31;
    uint32_t end   = ((uint32_t)addr + size + 31) & ~31;
    SCB_CleanDCache_by_Addr((uint32_t*)start, end - start);
}

void cache_invalidate(void *addr, size_t size)
{
    uint32_t start = (uint32_t)addr & ~31;
    uint32_t end   = ((uint32_t)addr + size + 31) & ~31;
    SCB_InvalidateDCache_by_Addr((uint32_t*)start, end - start);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
    if (hspi == &hspi1){
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
        cache_invalidate(RX_Buffer_1, 512);
        parse_input_can_command(RX_Buffer_1);
    }
    return;
}

void send_telemetry(void *handle, TX_Data_t *data, uint32_t data_size, uint8_t *TX_Buffer_1, uint8_t *RX_Buffer_1) {
    memcpy(TX_Buffer_1, data, data_size);
    cache_clean(TX_Buffer_1, 512);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_SPI_TransmitReceive_DMA(handle, TX_Buffer_1, RX_Buffer_1, data_size);
}

//void receive_input_can_command(void *handle, uint8_t *TX_Buffer_1, uint8_t *RX_Buffer_1, CAN_Packet *can_packet) {
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET);
//    HAL_SPI_TransmitReceive(handle, TX_Buffer_1, RX_Buffer_1, 10, 1);
//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
//    parse_input_can_command(RX_Buffer_1, can_packet);
//}

void parse_input_can_command(uint8_t RX_Buffer_1[]) {
    uint8_t sync_word = RX_Buffer_1[0];

    if (sync_word != 0xaa){
        performance_mode = 0;
        parsed_command = 0xe;
        parsed_data_size = 4;
        parsed_control_mode = CONTROL_MODE_POSITION;
        parsed_axis_state = AXIS_STATE_IDLE;
        for(uint8_t i=0; i<24; i++){
            CAN_Command_Datas[i] = 0;
        }

    } else {
        performance_mode = RX_Buffer_1[1];
        parsed_command = RX_Buffer_1[2];
        parsed_data_size = RX_Buffer_1[3];
        parsed_control_mode = RX_Buffer_1[4] & 0xf;
        parsed_input_mode = (RX_Buffer_1[4] >> 4) & 0xf;
        parsed_axis_state = RX_Buffer_1[5];

        memcpy(foot_scheduler, &RX_Buffer_1[6], 4);
        memcpy(foot_scheduler_diff, &RX_Buffer_1[10], 4);

        new_calculated_data_received = RX_Buffer_1[14];
//        if (new_calculated_data_counter < 100){
//            new_calculated_data_buffer[new_calculated_data_counter++] = new_calculated_data_received;
//        }
        memcpy(body_velocity_estimate, &RX_Buffer_1[15], 12);

        if(parsed_control_mode == CONTROL_MODE_TORQUE){
            memcpy(forces, &RX_Buffer_1[27], 12*4);
        } else{
            for(uint8_t i=0; i<12; i++){
                memcpy(&position_targets[i*2], &RX_Buffer_1[27+i*4], 4);
            }
        }
    }
}

void initialize_ahrs(FusionBias *bias, FusionAhrs *ahrs) {
    FusionBiasInitialise(bias, SAMPLE_RATE);
    FusionAhrsInitialise(ahrs);
    FusionAhrsSetSettings(ahrs, &settings);
}

void update_ahrs(FusionBias *bias, FusionAhrs *ahrs, int16_t accelerometer_data[], int16_t gyroscope_data[], int16_t magnetometer_data[],
        float_t euler_orientation[], float_t deltaTime) {

    FusionVector gyroscope = {{gyroscope_data[0] * GYROSCOPE_CONSTANT, gyroscope_data[1] * GYROSCOPE_CONSTANT, gyroscope_data[2] * GYROSCOPE_CONSTANT}};
    FusionVector accelerometer = {{accelerometer_data[0] * ACCELEROMETER_CONSTANT, accelerometer_data[1] * ACCELEROMETER_CONSTANT,
            accelerometer_data[2] * ACCELEROMETER_CONSTANT}};
    FusionVector magnetometer = {{magnetometer_data[0] * -MAGNETOMETER_CONSTANT, magnetometer_data[1] * MAGNETOMETER_CONSTANT,
            magnetometer_data[2] * MAGNETOMETER_CONSTANT}};

    gyroscope = FusionModelInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);

    accelerometer = FusionModelInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

    magnetometer = FusionModelMagnetic(magnetometer, softIronMatrix, hardIronOffset);

    gyroscope = FusionBiasUpdate(bias, gyroscope);

    FusionAhrsUpdate(ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(ahrs));

    //    const FusionVector earth = FusionAhrsGetEarthAcceleration(ahrs);

    memcpy(euler_orientation, euler.array, sizeof(float) * 3);
}

void readout_imu(void* spi_handle, uint8_t* TX_Buffer, uint8_t* RX_Buffer, int16_t* accelerometer_data, int16_t* gyroscope_data, int16_t* magnetometer_data, uint32_t* previousTimestamp, uint32_t* timestamp, float_t* deltaTime, int16_t* mag_temp){
    TX_Buffer[0] = 0xf;
//    cache_clean(TX_Buffer, 16);
    read_imu_fd(spi_handle, LSM6DSO_OUTX_L_G, &TX_Buffer[0], &RX_Buffer[0], 12);

    gyroscope_data[0] = RX_Buffer[0] + (RX_Buffer[1] << 8);
    gyroscope_data[1] = RX_Buffer[2] + (RX_Buffer[3] << 8);
    gyroscope_data[2] = RX_Buffer[4] + (RX_Buffer[5] << 8);

    accelerometer_data[0] = RX_Buffer[6] + (RX_Buffer[7] << 8);
    accelerometer_data[1] = RX_Buffer[8] + (RX_Buffer[9] << 8);
    accelerometer_data[2] = RX_Buffer[10] + (RX_Buffer[11] << 8);

    acceleration[0] = accelerometer_data[0] * ACCELEROMETER_CONSTANT;
    acceleration[1] = accelerometer_data[1] * ACCELEROMETER_CONSTANT;
    acceleration[2] = accelerometer_data[2] * ACCELEROMETER_CONSTANT;

    omega[0] = gyroscope_data[0] * GYROSCOPE_CONSTANT;
    omega[1] = gyroscope_data[1] * GYROSCOPE_CONSTANT;
    omega[2] = gyroscope_data[2] * GYROSCOPE_CONSTANT;

    read_imu_fd(spi_handle, LSM6DSO_TIMESTAMP0, &TX_Buffer[0], &RX_Buffer[0], 4);

    *previousTimestamp = *timestamp;
    *timestamp = (RX_Buffer[3] << 24) + (RX_Buffer[2] << 16) + (RX_Buffer[1] << 8) + (RX_Buffer[0]);
    *deltaTime = (*timestamp - *previousTimestamp) * 25.0 / 1000000.0;

    read_mag_fd(spi_handle, LIS2MDL_STATUS_REG, &TX_Buffer[0], &RX_Buffer[0], 9);

    magnetometer_data[0] = RX_Buffer[1] + (RX_Buffer[2] << 8) + 95;
    magnetometer_data[1] = RX_Buffer[3] + (RX_Buffer[4] << 8) + 90;
    magnetometer_data[2] = RX_Buffer[5] + (RX_Buffer[6] << 8) - 250;
    *mag_temp = (RX_Buffer[7] + (RX_Buffer[8] << 8)) / 8;
//    cache_invalidate(RX_Buffer, 16);
}

uint8_t poll_for_imu_new_data(void* spi_handle, uint8_t* TX_Buffer, uint8_t* RX_Buffer){

    //    read_imu_fd(spi_handle, LSM6DSO_OUTX_L_G, &TX_Buffer[0], &RX_Buffer[0], 1);

    uint8_t reg = LSM6DSO_STATUS_REG | 0x80;
    TX_Buffer[0] = reg;
    TX_Buffer[1] = 0;
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(spi_handle, TX_Buffer, RX_Buffer, 2, 1000);
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_SET);

    uint8_t return_flags = RX_Buffer[0] & 0x3;

    return return_flags;
}

void timer_wait(void) {
    uint32_t start;
    start = TIM6->CNT;
    while (1) {
        if (TIM6->CNT - start >= CLK_WAIT) {
            break;
        }
    }
}

void timer_wait2(uint16_t clks) {
    uint32_t timer2 = 0;
    TIM6->CNT = 0;
    while (timer2 < clks) {
        timer2 = TIM6->CNT;
    }
}

void platform_write_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_SET);
    timer_wait();
}

void platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    reg |= 0x80;
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_SET);
    timer_wait();
}

void platform_write_mag(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    HAL_GPIO_WritePin(MAG_CS_BANK, MAG_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(MAG_CS_BANK, MAG_CS_PIN, GPIO_PIN_SET);
    timer_wait();
}

void platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    reg |= 0x80;
    HAL_GPIO_WritePin(MAG_CS_BANK, MAG_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Receive(handle, bufp, len, 1000);
    HAL_GPIO_WritePin(MAG_CS_BANK, MAG_CS_PIN, GPIO_PIN_SET);
    timer_wait();
}

void initialize_imu_mag(void *handle, uint8_t *TX_Buffer, uint8_t *RX_Buffer) {
    TX_Buffer[0] = 0b00000001;

    platform_write_imu(handle, LSM6DSO_CTRL3_C, &TX_Buffer[0], 1);
    HAL_Delay(5);

    read_imu_fd(handle, LSM6DSO_WHO_AM_I, &TX_Buffer[0], &RX_Buffer[0], 1);

    TX_Buffer[0] = 0x0;
    TX_Buffer[1] = 0x0;
    platform_write_imu(handle, LSM6DSO_FUNC_CFG_ACCESS, &TX_Buffer[0], 1);

    TX_Buffer[0] = 0b01000100; //
    platform_write_imu(handle, LSM6DSO_CTRL3_C, &TX_Buffer[0], 1);
    read_imu_fd(handle, LSM6DSO_CTRL3_C, &TX_Buffer[0], &RX_Buffer[0], 1);

    TX_Buffer[0] = 0b01111100; //+-8g at 833Hz
    platform_write_imu(handle, LSM6DSO_CTRL1_XL, &TX_Buffer[0], 1);
    read_imu_fd(handle, LSM6DSO_CTRL1_XL, &TX_Buffer[0], &RX_Buffer[0], 1);

    TX_Buffer[0] = 0b01111100; //+-2000dps at 833Hz
    platform_write_imu(handle, LSM6DSO_CTRL2_G, &TX_Buffer[0], 1);
    read_imu_fd(handle, LSM6DSO_CTRL2_G, &TX_Buffer[0], &RX_Buffer[0], 1);

    TX_Buffer[0] = 0b00100000;
    platform_write_imu(handle, LSM6DSOX_CTRL10_C, &TX_Buffer[0], 1);
    read_imu_fd(handle, LSM6DSOX_CTRL10_C, &TX_Buffer[0], &RX_Buffer[0], 1);

    TX_Buffer[0] = 0b00100011;
    platform_write_mag(handle, LIS2MDL_CFG_REG_A, &TX_Buffer[0], 1);

    HAL_Delay(5);

    TX_Buffer[0] = 0b01000011;
    platform_write_mag(handle, LIS2MDL_CFG_REG_A, &TX_Buffer[0], 1);

    HAL_Delay(50);

    TX_Buffer[0] = 0b00001111;

    platform_write_mag(handle, LIS2MDL_CFG_REG_A, &TX_Buffer[0], 1);

    HAL_Delay(1);

    TX_Buffer[0] = 0b00110100;

    platform_write_mag(handle, LIS2MDL_CFG_REG_C, &TX_Buffer[0], 1);

    TX_Buffer[0] = 0b00001100;

    platform_write_mag(handle, LIS2MDL_CFG_REG_A, &TX_Buffer[0], 1);

    read_mag_fd(handle, LIS2MDL_WHO_AM_I, &TX_Buffer[0], &RX_Buffer[0], 1);
}

/*
 4 wire
 */

void read_imu_fd(void *handle, uint8_t reg, uint8_t *buft, uint8_t *bufr, uint16_t len) {
    reg |= 0x80;
    buft[0] = reg;
    buft[1] = 0;
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(handle, buft, bufr, len + 1, 1000);
    HAL_GPIO_WritePin(IMU_CS_BANK, IMU_CS_PIN, GPIO_PIN_SET);
    for (int i = 0; i < len + 1; ++i) {
        bufr[i] = bufr[i + 1];
    }
    timer_wait();
}

void read_mag_fd(void *handle, uint8_t reg, uint8_t *buft, uint8_t *bufr, uint16_t len) {
    reg |= 0x80;
    buft[0] = reg;
    buft[1] = 0;
    HAL_GPIO_WritePin(MAG_CS_BANK, MAG_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(handle, buft, bufr, len + 1, 1000);
    HAL_GPIO_WritePin(MAG_CS_BANK, MAG_CS_PIN, GPIO_PIN_SET);
    for (int i = 0; i < len + 1; ++i) {
        bufr[i] = bufr[i + 1];
    }
    timer_wait();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
       line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
