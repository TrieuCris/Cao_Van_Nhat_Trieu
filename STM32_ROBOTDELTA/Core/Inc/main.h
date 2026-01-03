/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/**
 * @brief Trạng thái chung của robot.
 */
typedef enum {
    ROBOT_STATE_IDLE,   // Robot đang rảnh và sẵn sàng nhận lệnh
    ROBOT_STATE_MOVING, // Robot đang thực hiện một khối lệnh chuyển động
    ROBOT_STATE_HOMING, // Robot đang trong quá trình Homing
    ROBOT_STATE_ESTOP   // Robot đang ở trạng thái dừng khẩn cấp
} RobotState;

/**
 * @brief Đại diện cho một khối lệnh chuyển động nguyên tử.
 *        Mỗi khối lệnh chứa thông tin về số bước cho mỗi động cơ,
 *        thời gian thực hiện, và trạng thái của các cơ cấu chấp hành khác.
 */
typedef struct {
    uint32_t id;                // ID duy nhất của khối lệnh, dùng để báo cáo lại khi hoàn thành
    int32_t motor_steps[3];     // Số bước cho mỗi động cơ (M1, M2, M3). Dấu +/- chỉ chiều quay.
    
    // ✅ CRITICAL: Thời gian thực hiện (ms) - Dùng để tính DDS speed_addend chính xác
    uint32_t duration;          // Thời gian thực hiện lệnh (đơn vị: milliseconds). Nhận từ 't' của PC.
    
    uint32_t step_interval;     // [DEPRECATED] Không dùng nữa cho DDS, giữ để tương thích ngược.
    uint16_t servo_pulse;       // Giá trị pulse cho servo (thường từ 500-2500).
    bool pump_state;            // Trạng thái của bơm (true: BẬT, false: TẮT).
} MotionBlock;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
// Limit Switches (NC với Pull-up)
#define LS1_Pin GPIO_PIN_12
#define LS1_GPIO_Port GPIOB
#define LS2_Pin GPIO_PIN_13
#define LS2_GPIO_Port GPIOB
#define LS3_Pin GPIO_PIN_14
#define LS3_GPIO_Port GPIOB

// Motor 1 (Delta Robot)
#define M1_PUL_Pin GPIO_PIN_1
#define M1_PUL_GPIO_Port GPIOB
#define M1_DIR_Pin GPIO_PIN_10
#define M1_DIR_GPIO_Port GPIOB
#define M1_ENA_Pin GPIO_PIN_11
#define M1_ENA_GPIO_Port GPIOB

// Motor 2 (Delta Robot)
#define M2_PUL_Pin GPIO_PIN_0
#define M2_PUL_GPIO_Port GPIOB
#define M2_DIR_Pin GPIO_PIN_5
#define M2_DIR_GPIO_Port GPIOA
#define M2_ENA_Pin GPIO_PIN_6
#define M2_ENA_GPIO_Port GPIOA

// Motor 3 (Delta Robot)
#define M3_PUL_Pin GPIO_PIN_7
#define M3_PUL_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_3
#define M3_DIR_GPIO_Port GPIOA
#define M3_ENA_Pin GPIO_PIN_4
#define M3_ENA_GPIO_Port GPIOA

// Conveyor (Băng tải)
#define CONV_PUL_Pin GPIO_PIN_0
#define CONV_PUL_GPIO_Port GPIOA
#define CONV_DIR_Pin GPIO_PIN_1
#define CONV_DIR_GPIO_Port GPIOA
#define CONV_ENA_Pin GPIO_PIN_2
#define CONV_ENA_GPIO_Port GPIOA
#define CONV_SENSOR_Pin GPIO_PIN_15
#define CONV_SENSOR_GPIO_Port GPIOB

// Emergency Stop (NC với Pull-up)
#define ESTOP_Pin GPIO_PIN_8
#define ESTOP_GPIO_Port GPIOA

// Vacuum Pump
#define PUMP_Pin GPIO_PIN_9
#define PUMP_GPIO_Port GPIOA

// Status LEDs
#define GREEN_LED_Pin GPIO_PIN_10
#define GREEN_LED_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_15
#define RED_LED_GPIO_Port GPIOA

// Camera Light (Đèn trợ sáng camera - Relay thường đóng)
#define CAMERA_LIGHT_Pin GPIO_PIN_4
#define CAMERA_LIGHT_GPIO_Port GPIOB

// Button LED
#define BUTTON_LED_Pin GPIO_PIN_3
#define BUTTON_LED_GPIO_Port GPIOB

// Hardware Buttons (NO with Pull-down)
#define BTN_START_Pin GPIO_PIN_6
#define BTN_START_GPIO_Port GPIOB
#define BTN_STOP_Pin GPIO_PIN_7
#define BTN_STOP_GPIO_Port GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
