/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cdc_handler.h"
#include "command_queue.h"
#include "robot_control.h"
#include "conveyor.h"
#include "status_led.h"
#include "command_parser.h"
#include "button_handler.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char main_command_buffer[RX_BUFFER_SIZE]; // NOLINT
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // Khởi tạo tất cả các module phần mềm
  status_led_init();
  cdc_handler_init();
  queue_init();
  robot_init();
  conveyor_init();
  button_handler_init();  // Khởi tạo module xử lý nút bấm và debounce
  
  // Đợi một chút để USB CDC ổn định
  HAL_Delay(100);
  
  // Gửi thông báo hệ thống đã sẵn sàng
  cdc_handler_send_response("SYS_READY");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Xử lý lệnh từ USB (NON-BLOCKING)
    // Kiểm tra xem cdc_handler có lệnh mới không
    if (cdc_handler_get_command(main_command_buffer, RX_BUFFER_SIZE))
    {
        // Nếu có, thực thi lệnh trong main loop (an toàn!)
        parse_command(main_command_buffer);
    }

    // Xử lý hàng đợi lệnh: kiểm tra và gửi lệnh cho robot nếu rảnh
    queue_process();
    
    // Xử lý các thông báo DONE từ hàng đợi (do ISR đẩy vào)
    queue_handle_done_messages();

    // Cập nhật trạng thái nút bấm và E-Stop (có debounce)
    button_handler_update();
    
    // Nếu E-Stop vừa thay đổi (được phát hiện bên trong button_handler_update), 
    // ta nên gửi status ngay. Tuy nhiên button_handler_update gọi callback robot_estop_triggered.
    // Để đơn giản, ta kiểm tra cạnh ở đây.
    static bool last_estop_val = false;
    bool current_estop_val = estop_is_triggered();
    if (current_estop_val != last_estop_val) {
        last_estop_val = current_estop_val;
        send_status_report(); // Gửi status ngay khi E-Stop đổi trạng thái
    }

    // --- Xử lý sự kiện nút bấm ---
    if (button_just_pressed(BUTTON_1) || button_just_pressed(BUTTON_2)) {
        send_status_report(); // Gửi trạng thái đầy đủ ngay khi nhấn nút
    }

    // --- Giám sát cảm biến khay (Tray Sensor) ---
    static int last_tray_state = -1;
    int current_tray_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;
    
    if (last_tray_state == -1) {
        last_tray_state = current_tray_state;
    } 
    else if (current_tray_state != last_tray_state) {
        HAL_Delay(20);
        int confirm_state = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;
        if (confirm_state == current_tray_state) {
            last_tray_state = current_tray_state;
            send_status_report(); // Gửi trạng thái đầy đủ khi cảm biến khay thay đổi
        }
    }

    // Cập nhật trạng thái homing (gọi mỗi vòng lặp)
    robot_update_homing_state();
    
    // ✅ Kiểm tra Servo Idle để ngắt xung (Auto-Detach)
    robot_poll_servo_idle();
    
    // ✅ Clear flags để tránh memory leak (không log nhưng vẫn clear)
    for (int i = 0; i < 3; i++) {
        robot_get_and_clear_flag_homing_ls_trig(i);
    }

    // Cập nhật LED trạng thái (gọi mỗi vòng lặp để nhấp nháy được smooth)
    status_led_update();

    // --- Xử lý Homing Watchdog và các cờ báo hiệu từ ISR ---

    // Watchdog cho Homing (chỉ kiểm tra khi đang homing)
    if (robot_get_state() == ROBOT_STATE_HOMING && robot_get_homing_state() == HOMING_STATE_RAISING)
    {
        if ((HAL_GetTick() - robot_get_homing_start_tick()) > HOMING_WATCHDOG_MS)
        {
            robot_abort(); // Dừng robot
            cdc_handler_send_response("ERROR:Homing timed out");
        }
    }

    // Các cờ khác
    if (robot_get_and_clear_flag_homing_timeout_backing_off()) cdc_handler_send_response("ERROR:HOMING_TIMEOUT_BACKING_OFF");
    if (robot_get_and_clear_flag_homing_done()) cdc_handler_send_response("HOME_DONE");

    int misconfig_motor;
    if (robot_get_and_clear_flag_homing_dir_misconfig(&misconfig_motor)) {
        cdc_handler_send_response("ERROR:HOMING_DIR_MISCONFIG on motor %d", misconfig_motor);
    }

    // 🚨 SAFETY: Kiểm tra limit switch bị kẹt/hỏng
    for (int i = 0; i < 3; i++) {
        if (robot_get_and_clear_flag_ls_stuck(i)) {
            cdc_handler_send_response("ERROR:LIMIT_SWITCH_STUCK:M%d (backoff >2000 steps, sensor may be broken)", i + 1);
            robot_abort(); // Dừng khẩn cấp
        }
    }

        for (int i = 0; i < 3; i++) {

            if (robot_get_and_clear_flag_up_blocked(i)) {

                // 🚨 XỬ LÝ SỰ CỐ: Chạm công tắc hành trình bất ngờ

                cdc_handler_send_response("ERROR:CRITICAL:Motor %d hit limit switch during move! (Lost steps/Position mismatch)", i + 1);

                

                // Dừng robot và reset trạng thái

                robot_abort();

                

                // Xóa hàng đợi lệnh (vì các lệnh tiếp theo sẽ sai tọa độ)

                queue_flush();

                

                // Báo cho PC biết cần phải Homing lại

                cdc_handler_send_response("ERROR:REQUIRE_HOMING");

            }

        }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  // ✅ FIX: Cấu hình GPIO cho TIM2_CH1 (PA0) để xuất PWM cho băng tải
  HAL_TIM_MspPostInit(&htim2);
  // Cấu hình channel PWM sẽ được thực hiện trong conveyor_init
  // sMasterConfig và HAL_TIMEx_MasterConfigSynchronization đã được chuyển vào conveyor_init
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
  // Việc khởi tạo chi tiết timer này (prescaler, period, interrupt callback)
  // đã được chuyển vào hàm robot_init() trong file robot_control.c
  // để tránh bị CubeMX ghi đè.
  // Hàm này được giữ lại ở đây để duy trì cấu trúc dự án của CubeMX.
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  // Việc khởi tạo chi tiết timer này cho Servo (PWM)
  // đã được chuyển vào hàm robot_init() trong file robot_control.c
  // để tránh bị CubeMX ghi đè.
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);  // ✅ PA9 (PUMP) = 0V = TẮT

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6
                          |GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_LIGHT_GPIO_Port, CAMERA_LIGHT_Pin, GPIO_PIN_RESET);  // Đèn trợ sáng BẬT khi khởi động (Relay thường đóng: LOW = đóng relay = bật đèn)

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11
                           PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 (Reserved - NC with PULLUP) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 (Conveyor Sensor - NPN NO with PULLUP) */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6, PB7 (START and STOP buttons - NO with VCC) */
  /* Logic: Pressed = 1 (VCC), Released = 0 (Internal PULLDOWN) */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 (E-Stop - NC with PULLUP, interrupt on RISING_FALLING edge) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Các cấu hình GPIO khác như công tắc hành trình, nút nhấn...
  // nên được thực hiện trong các module tương ứng (ví dụ: robot_init)
  // để đảm bảo tính đóng gói và dễ quản lý.

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief GPIO EXTI Callback - Xử lý ngắt E-Stop
 * @param GPIO_Pin: Pin gây ra ngắt
 * @note: Đã chuyển sang xử lý debounce trong button_handler_update()
 *        Ngắt này chỉ để phản ứng nhanh, logic thực sự xử lý ở polling
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // ✅ XỬ LÝ NGAY TRONG NGẮT (Hard Real-time Safety)
    // Lý do bảo vệ đồ án: Để đảm bảo an toàn tối đa, tín hiệu dừng khẩn cấp được xử lý 
    // ngay lập tức trong ngắt ngoại vi khi NÚT ĐƯỢC NHẤN.
    
    if (GPIO_Pin == GPIO_PIN_8) // E_STOP_Pin (PA8)
    {
        // Kiểm tra trạng thái vật lý của chân PA8
        // NC (Thường đóng) + PullUp:
        // - Bình thường (Đóng): Nối GND -> Mức 0 (LOW)
        // - Nhấn E-Stop (Mở): Ngắt GND, PullUp kéo lên -> Mức 1 (HIGH)
        
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) 
        {
            // --- TRƯỜNG HỢP 1: NÚT VỪA ĐƯỢC NHẤN (RISING) ---
            // Ưu tiên cao nhất: Ngắt toàn bộ xung động cơ và dừng băng tải NGAY LẬP TỨC
            robot_estop_triggered(); 
            
            // Gửi cảnh báo lên PC ngay lập tức để giao diện cập nhật
            send_status_report();
        }
        // Trường hợp NHẢ (Falling) để cho Main Loop xử lý qua cơ chế Polling debounce
    }
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
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
