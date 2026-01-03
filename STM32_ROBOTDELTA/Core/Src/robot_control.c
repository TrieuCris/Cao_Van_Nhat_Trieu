#include "robot_control.h"
#include "command_queue.h"
#include "cdc_handler.h"
#include "conveyor.h"
#include "status_led.h"
#include "stm32f1xx_hal_tim.h" // Thêm header cho các hàm timer
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

// --- Cấu hình mức kích hoạt công tắc hành trình (LOW hoặc HIGH) ---
#ifndef LS_ACTIVE_LEVEL
#define LS_ACTIVE_LEVEL GPIO_PIN_SET  // NC nối GND + Pull-up: chạm = HIGH, không chạm = LOW
#endif

// --- Cấu hình chiều DIR cho đi lên / đi xuống (có thể đảo bằng build define nếu wiring ngược) ---
#ifndef HOMING_DIR_UP_LEVEL
#define HOMING_DIR_UP_LEVEL GPIO_PIN_RESET
#endif
#ifndef HOMING_DIR_DOWN_LEVEL
#define HOMING_DIR_DOWN_LEVEL GPIO_PIN_SET  // Phải khác UP để thực sự đi xuống
#endif

// --- 🚨 SAFETY: Giới hạn lùi an toàn (Max Back-off Steps) ---
// [UPDATE MICROSTEP 1/4]: Tăng gấp 2 lần (2000 -> 4000)
// Nếu robot lùi quá 4000 bước mà cảm biến vẫn chưa nhả -> Cảm biến bị kẹt/hỏng
#define HOMING_MAX_BACKOFF_STEPS 4000

// --- Giá trị bước tuyệt đối tại vị trí Home (-45 độ) ---
// [UPDATE MICROSTEP 1/4]: User yêu cầu reset về 0 khi home
#define STEPS_AT_HOME_NEG45 0 // Reset về 0, PC sẽ coi đây là 0 độ (Home Angle)

// --- Tần số của timer "nhịp tim" điều khiển chuyển động ---
// [UPDATE OPTIMIZATION]: Tăng lên 80kHz để vi bước 1/8 mượt hơn
#define STEPPER_TIMER_FREQ 80000 // 80kHz, tương đương 12.5us mỗi tick.

// --- Auto-Detach Servo Config ---
#define SERVO_IDLE_TIMEOUT_MS 2000 // 2 giây không hoạt động sẽ ngắt xung servo

// --- Hằng số DDS (Direct Digital Synthesis) cho thuật toán chính xác ---
// DDS_CONST = 2^32 / STEPPER_TIMER_FREQ = 4294967296 / 80000 = 53687.09
// [FIX] Dùng 53688 (làm tròn LÊN) để đảm bảo đủ số xung trong duration_ms
// Nếu dùng 53687 (làm tròn xuống), mỗi block sẽ thiếu ~1 step do sai số tích lũy
#define DDS_CONST 53688UL  // Hệ số chuyển đổi từ Hz sang speed_addend (CEIL)

// Giả sử bạn có các biến Timer Handle được tạo bởi CubeMX
extern TIM_HandleTypeDef htim3; // Timer cho xung step motor
extern TIM_HandleTypeDef htim4; // Timer cho Servo

// --- Biến cục bộ cho module robot_control ---
static volatile RobotState current_robot_state = ROBOT_STATE_IDLE;
static volatile int32_t absolute_motor_steps[3] = {0, 0, 0}; // Để theo dõi vị trí tuyệt đối
static volatile HomingState current_homing_state = HOMING_STATE_IDLE;
static volatile uint32_t homing_start_tick_ms = 0; // Tick bắt đầu homing để watchdog
static volatile int32_t homing_start_steps[3] = {0,0,0}; // Baseline bước lúc bắt đầu homing
static volatile bool servo_pwm_started = false; // Theo dõi xem PWM servo đã được khởi động chưa
static volatile bool robot_homed = false; // Theo dõi xem robot đã hoàn thành homing chưa
static uint32_t last_servo_activity_tick = 0; // Thời điểm cuối cùng servo hoạt động

// Các biến cho quá trình Homing
static volatile bool homing_motor_done[3] = {false, false, false};
static volatile bool homing_backoff_done[3] = {false, false, false}; // Theo dõi back-off cho từng motor
static volatile int32_t homing_backoff_steps[3] = {0, 0, 0}; // Đếm số bước đã lùi (safety)
static volatile uint32_t homing_confirmation_counter[3] = {0, 0, 0}; // Để xác nhận cảm biến ổn định
#define HOMING_DEBOUNCE_COUNT 30 // Cần 30 lần liên tiếp đọc HIGH để xác nhận chạm cảm biến (chống nhiễu)
#define HOMING_BACKOFF_DEBOUNCE_COUNT 30 // Cần 30 lần liên tiếp đọc LOW để xác nhận nhả cảm biến

// --- Cờ báo lỗi limit switch kẹt cứng (stuck) ---
static volatile bool g_flag_ls_stuck[3] = {false, false, false};

// --- Debounce counter cho trạng thái MOVING (Chống nhiễu khi đang chạy) ---
static volatile uint8_t moving_ls_debounce[3] = {0, 0, 0};
#define MOVING_LS_DEBOUNCE_THRESHOLD 15 // 15 ticks * 12.5us = ~187us filter

// --- Biến cờ (Flag) để giao tiếp không chặn từ ISR ---
static volatile bool g_flag_homing_timeout_backing_off = false;
static volatile bool g_flag_homing_all_switched = false;
static volatile bool g_flag_homing_backing_off_start = false;
static volatile bool g_flag_homing_dir_misconfig = false;
static volatile int g_flag_homing_dir_misconfig_motor = -1;
static volatile bool g_flag_homing_ls_trig[3] = {false, false, false};
static volatile bool g_flag_homing_early_fallback[3] = {false, false, false};
static volatile bool g_flag_up_blocked[3] = {false, false, false};
static volatile bool g_flag_homing_done = false;

typedef struct {
    bool active;
    bool direction;      // true for forward, false for reverse
    uint32_t steps_to_go;
    
    // --- DDS (Direct Digital Synthesis) - Thuật toán tạo xung chính xác ---
    uint32_t speed_addend;   // Giá trị cộng dồn mỗi lần ngắt (tương ứng vận tốc)
    uint32_t accumulator;    // Biến tích lũy, khi tràn (overflow) -> tạo xung
    // -------------------------------------------------------------------------
    
    GPIO_TypeDef* pul_port;
    uint16_t pul_pin;
    bool pul_state; // Trạng thái hiện tại của chân PUL (false: LOW, true: HIGH)
} MotorMotionState;

static volatile MotorMotionState motor_states[3];

// --- Khai báo hàm xử lý ngắt nội bộ ---
static void robot_timer_irq_handler(TIM_HandleTypeDef *htim);

// Helper: kiểm tra công tắc hành trình theo index (0..2)
// [OPTIMIZATION]: Inline và sử dụng IDR để đọc nhanh nhất
static inline bool robot_is_limit_switch_triggered_fast(int idx) {
    // Đọc trực tiếp thanh ghi IDR để tiết kiệm chu kỳ CPU (bỏ qua HAL check)
    if (idx == 0) return (LS1_GPIO_Port->IDR & LS1_Pin) == (LS_ACTIVE_LEVEL ? LS1_Pin : 0);
    if (idx == 1) return (LS2_GPIO_Port->IDR & LS2_Pin) == (LS_ACTIVE_LEVEL ? LS2_Pin : 0);
    if (idx == 2) return (LS3_GPIO_Port->IDR & LS3_Pin) == (LS_ACTIVE_LEVEL ? LS3_Pin : 0);
    return false;
}

/**
 * @brief Khởi tạo bộ đếm chu kỳ DWT.
 */
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Chuyển đổi góc servo thành giá trị pulse cho timer.
 * @param angle_x100 Góc * 100 (0-27000 cho 0-270.00°). Ví dụ: 27000 = 270.0°
 * @return Giá trị pulse (500-2500 us)
 */
uint16_t robot_servo_angle_to_pulse(uint32_t angle_x100) {
    // Giới hạn góc trong khoảng 0-27000 (0-270°)
    if (angle_x100 > 27000) angle_x100 = 27000;
    // Ánh xạ: 500 + (angle_x100 * 20) / 270
    return 500 + (uint16_t)((angle_x100 * 20) / 270);
}


void robot_init(void) {
    // Khởi động với ENA = DISABLED
    HAL_GPIO_WritePin(M1_ENA_GPIO_Port, M1_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_ENA_GPIO_Port, M2_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_ENA_GPIO_Port, M3_ENA_Pin, GPIO_PIN_SET);

    dwt_init();

    motor_states[0].pul_port = M1_PUL_GPIO_Port; motor_states[0].pul_pin = M1_PUL_Pin;
    motor_states[1].pul_port = M2_PUL_GPIO_Port; motor_states[1].pul_pin = M2_PUL_Pin;
    motor_states[2].pul_port = M3_PUL_GPIO_Port; motor_states[2].pul_pin = M3_PUL_Pin;

    /* USER CODE BEGIN robot_init_TIM4 */
    // Cấu hình TIM4 cho Servo (PWM 50Hz - 20ms period)
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 71;
    htim4.Init.Period = 19999; 
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;  // 1.5ms = center position for 270-degree servo
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    
    // Start PWM immediately to prevent servo from receiving undefined signals
    // This sets servo to center position (135 degrees) at startup
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    servo_pwm_started = true;
    last_servo_activity_tick = HAL_GetTick(); // Khởi tạo tick
    /* USER CODE END robot_init_TIM4 */

    /* USER CODE BEGIN robot_init_TIM3 */
    // Cấu hình timer 3 làm "nhịp tim" (80kHz)
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (timer_clock / 1000000) - 1; // 1MHz counter clock
    htim3.Init.Period = (1000000 / STEPPER_TIMER_FREQ) - 1; // Period for 80kHz
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE END robot_init_TIM3 */
    
    current_robot_state = ROBOT_STATE_IDLE;
    cdc_handler_send_response("SYS_READY");
    
    // ✅ Check E-Stop status immediately on startup
    // PA8 is E-Stop pin (NC + PullUp -> Active High when pressed/open)
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
        robot_estop_triggered(); 
    }
}

// Hàm nội bộ để áp dụng cấu hình block vào motor
static void robot_apply_block(const MotionBlock* block) {
    // ✅ Luôn cho phép điều khiển Servo để xử lý sự cố (không cần Home)
    if (!servo_pwm_started) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        servo_pwm_started = true;
    }
    // Cập nhật thời gian hoạt động để tránh auto-detach ngay lập tức
    last_servo_activity_tick = HAL_GetTick();

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, block->servo_pulse);
    
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, block->pump_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint32_t max_steps = 0;
    for (int i = 0; i < 3; i++) {
        uint32_t steps = abs(block->motor_steps[i]);
        if (steps > max_steps) max_steps = steps;
    }
    
    if (max_steps == 0) return;
    
    uint32_t duration_ms = block->duration;
    if (duration_ms == 0) duration_ms = 1;

    for (int i = 0; i < 3; i++) {
        uint32_t steps = abs(block->motor_steps[i]);
        if (steps == 0) {
            motor_states[i].active = false;
            continue;
        }

        motor_states[i].active = true;
        motor_states[i].steps_to_go = steps;
        
        // ✅ FIX: Chỉ reset accumulator khi bắt đầu từ IDLE (trajectory mới)
        // Khi đang MOVING (nối block), giữ nguyên accumulator để đảm bảo liên tục
        // NHƯNG: Nếu tốc độ thay đổi đáng kể, vẫn nên reset để tránh sai pha
        // Giải pháp: Reset về 50% khi speed_addend mới khác >10% so với cũ
        uint64_t numerator = (uint64_t)steps * 1000ULL * (uint64_t)DDS_CONST;
        // ✅ FIX: Làm tròn (rounding) thay vì cắt bỏ (truncation)
        uint32_t new_speed_addend = (uint32_t)((numerator + (duration_ms / 2)) / duration_ms);
        
        // Kiểm tra xem có cần reset accumulator không
        uint32_t old_addend = motor_states[i].speed_addend;
        uint32_t diff = (new_speed_addend > old_addend) ? 
                        (new_speed_addend - old_addend) : (old_addend - new_speed_addend);
        bool speed_changed_significantly = (old_addend == 0) || (diff > old_addend / 10);
        
        if (speed_changed_significantly) {
            // Tốc độ thay đổi >10% hoặc block đầu tiên -> Reset accumulator
            motor_states[i].accumulator = 0x80000000UL;
            motor_states[i].pul_state = false;
        }
        // Nếu tốc độ tương tự, giữ nguyên accumulator để nối tiếp mượt mà
        
        motor_states[i].speed_addend = new_speed_addend;

        // Cập nhật lại Port/Pin (an toàn)
        if (i == 0) { motor_states[i].pul_port = M1_PUL_GPIO_Port; motor_states[i].pul_pin = M1_PUL_Pin; }
        else if (i == 1) { motor_states[i].pul_port = M2_PUL_GPIO_Port; motor_states[i].pul_pin = M2_PUL_Pin; }
        else { motor_states[i].pul_port = M3_PUL_GPIO_Port; motor_states[i].pul_pin = M3_PUL_Pin; }

        GPIO_TypeDef* dir_port;
        uint16_t dir_pin;
        if (i == 0) { dir_port = M1_DIR_GPIO_Port; dir_pin = M1_DIR_Pin; }
        else if (i == 1) { dir_port = M2_DIR_GPIO_Port; dir_pin = M2_DIR_Pin; }
        else { dir_port = M3_DIR_GPIO_Port; dir_pin = M3_DIR_Pin; }
        HAL_GPIO_WritePin(dir_port, dir_pin, (block->motor_steps[i] > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        
        motor_states[i].direction = (block->motor_steps[i] > 0);
    }
    
    // Enable motors
    HAL_GPIO_WritePin(M1_ENA_GPIO_Port, M1_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_ENA_GPIO_Port, M2_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_ENA_GPIO_Port, M3_ENA_Pin, GPIO_PIN_RESET);
    status_led_set_status(LED_STATUS_RUNNING);
}

void robot_execute_block(const MotionBlock* block) {
    if (current_robot_state != ROBOT_STATE_IDLE) return;
    
    current_robot_state = ROBOT_STATE_MOVING;
    
    // Reset debounce counters
    moving_ls_debounce[0] = 0;
    moving_ls_debounce[1] = 0;
    moving_ls_debounce[2] = 0;
    
    robot_apply_block(block);
    
    bool any_active = false;
    for(int i=0; i<3; i++) if(motor_states[i].active) any_active = true;
    
    if (any_active) {
        HAL_TIM_Base_Start_IT(&htim3);
    } else {
        current_robot_state = ROBOT_STATE_IDLE;
        status_led_set_status(LED_STATUS_IDLE);
        queue_finish_current_block();
    }
}

/**
 * @briefHàm xử lý ngắt nội bộ cho timer robot (TIM3).
 *        [OPTIMIZED] Sử dụng BSRR/IDR thay vì HAL.
 */
static void robot_timer_irq_handler(TIM_HandleTypeDef *htim) {
    if (current_robot_state == ROBOT_STATE_MOVING) {
        bool still_moving = false;
        bool up_blocked_debug[3] = {false,false,false};
        for (int i = 0; i < 3; i++) {
            if (!motor_states[i].active) continue;
            
            // Logic cũ: DIR == HOMING_DIR_UP_LEVEL (RESET) -> đang đi lên.
            GPIO_TypeDef* dir_port;
            uint16_t dir_pin;
            if (i == 0) { dir_port = M1_DIR_GPIO_Port; dir_pin = M1_DIR_Pin; }
            else if (i == 1) { dir_port = M2_DIR_GPIO_Port; dir_pin = M2_DIR_Pin; }
            else { dir_port = M3_DIR_GPIO_Port; dir_pin = M3_DIR_Pin; }
            
            // Đọc nhanh trạng thái DIR từ thanh ghi ODR (Output Data Register) vì ta đang lái nó
            bool is_up_dir = ((dir_port->ODR & dir_pin) == (HOMING_DIR_UP_LEVEL ? dir_pin : 0));

            if (is_up_dir) {
                if (robot_is_limit_switch_triggered_fast(i)) {
                    if (moving_ls_debounce[i] < MOVING_LS_DEBOUNCE_THRESHOLD) {
                        moving_ls_debounce[i]++;
                    }
                } else {
                    moving_ls_debounce[i] = 0;
                }

                if (moving_ls_debounce[i] >= MOVING_LS_DEBOUNCE_THRESHOLD) {
                    // 🚨 CRITICAL ERROR: Chạm LS khi đang chạy -> Mất bước/Sai lệch nghiêm trọng
                    // Giải pháp: Dừng TOÀN BỘ robot ngay lập tức thay vì chỉ dừng 1 trục
                    
                    for(int k=0; k<3; k++) {
                        motor_states[k].active = false;
                        motor_states[k].steps_to_go = 0;
                        motor_states[k].accumulator = 0;
                        // Kéo chân PUL xuống thấp
                        if (motor_states[k].pul_state) {
                            motor_states[k].pul_port->BSRR = (uint32_t)motor_states[k].pul_pin << 16U;
                            motor_states[k].pul_state = false;
                        }
                    }
                    
                    up_blocked_debug[i] = true; // Đánh dấu trục bị lỗi để báo về Main
                    still_moving = false;       // Ép vòng lặp dừng ngay
                    break;                      // Thoát vòng lặp for i
                }
            } else {
                // Nếu đang đi xuống, reset counter an toàn
                moving_ls_debounce[i] = 0;
            }

            // ========== DDS Algorithm ==========
            uint32_t old_acc = motor_states[i].accumulator;
            motor_states[i].accumulator += motor_states[i].speed_addend;
            
            if (motor_states[i].accumulator < old_acc) {
                // --- Tạo xung (Pulse) ---
                if (motor_states[i].steps_to_go > 0) {
                    // [OPTIMIZATION] Ghi trực tiếp BSRR (Set Bit)
                    motor_states[i].pul_port->BSRR = motor_states[i].pul_pin;
                    motor_states[i].pul_state = true;

                    motor_states[i].steps_to_go--;
                    if (motor_states[i].direction) absolute_motor_steps[i]++;
                    else absolute_motor_steps[i]--;
                }
            } else {
                if (motor_states[i].pul_state) {
                    // [OPTIMIZATION] Ghi trực tiếp BSRR (Reset Bit)
                    motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
                    motor_states[i].pul_state = false;
                }
            }
            // ==================================

            // [FIX BUG] Last Pulse Cutoff: 
            // Trước đây: steps_to_go == 0 thì ép xuống LOW ngay -> mất xung cuối.
            // Bây giờ: Chờ cho DDS tự đưa pul_state về LOW (ở lần ngắt tiếp theo) rồi mới active = false.
            if (motor_states[i].steps_to_go == 0 && motor_states[i].pul_state == false) {
                motor_states[i].active = false;
            }

            if (motor_states[i].active) still_moving = true;
        }

        for (int i=0;i<3;i++) { if (up_blocked_debug[i]) g_flag_up_blocked[i] = true; }

        if (!still_moving) {
            // ✅ FIX BUG: Thêm delay nhỏ giữa các block để đảm bảo driver nhận đủ xung cuối
            // Pulse width min của TB6600 là ~2.5us, 500 NOPs @ 72MHz ≈ 7us (an toàn)
            for (volatile int delay = 0; delay < 500; delay++) { __NOP(); }
            
            queue_finish_current_block(); 
            MotionBlock next_block;
            if (queue_pop_next_from_isr(&next_block)) {
                // ✅ FIX: Thêm delay setup time cho DIR pin trước khi phát xung
                // TB6600 cần ~5us setup time cho DIR, 800 NOPs @ 72MHz ≈ 11us (an toàn tuyệt đối)
                for (volatile int delay = 0; delay < 800; delay++) { __NOP(); }
                robot_apply_block(&next_block);
            } else {
                HAL_TIM_Base_Stop_IT(&htim3);
                current_robot_state = ROBOT_STATE_IDLE;
                status_led_set_status(LED_STATUS_IDLE);
            }
        }
    }
    else if (current_robot_state == ROBOT_STATE_HOMING) {
        switch (current_homing_state) {
            case HOMING_STATE_RAISING: {
                bool all_homed = true;
                
                // [OPTIMIZATION] Dùng hàm fast check limit switch
                if (!homing_motor_done[0]) {
                    if (robot_is_limit_switch_triggered_fast(0)) {
                        homing_confirmation_counter[0]++;
                        if (homing_confirmation_counter[0] >= HOMING_DEBOUNCE_COUNT) {
                            homing_motor_done[0] = true;
                            g_flag_homing_ls_trig[0] = true;
                        }
                    } else { homing_confirmation_counter[0] = 0; }
                }
                if (!homing_motor_done[1]) {
                    if (robot_is_limit_switch_triggered_fast(1)) {
                        homing_confirmation_counter[1]++;
                        if (homing_confirmation_counter[1] >= HOMING_DEBOUNCE_COUNT) {
                            homing_motor_done[1] = true;
                            g_flag_homing_ls_trig[1] = true;
                        }
                    } else { homing_confirmation_counter[1] = 0; }
                }
                if (!homing_motor_done[2]) {
                    if (robot_is_limit_switch_triggered_fast(2)) {
                        homing_confirmation_counter[2]++;
                        if (homing_confirmation_counter[2] >= HOMING_DEBOUNCE_COUNT) {
                            homing_motor_done[2] = true;
                            g_flag_homing_ls_trig[2] = true;
                        }
                    } else { homing_confirmation_counter[2] = 0; }
                }

                for (int i = 0; i < 3; i++) {
                    if (!homing_motor_done[i]) {
                        all_homed = false;
                        motor_states[i].active = true;
                        
                        // DDS Homing
                        uint32_t old_acc = motor_states[i].accumulator;
                        motor_states[i].accumulator += motor_states[i].speed_addend;
                        
                        if (motor_states[i].accumulator < old_acc) {
                            // [OPTIMIZATION] BSRR Set
                            motor_states[i].pul_port->BSRR = motor_states[i].pul_pin;
                            motor_states[i].pul_state = true;
                            absolute_motor_steps[i]--; 
                        } else {
                            if (motor_states[i].pul_state) {
                                // [OPTIMIZATION] BSRR Reset
                                motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
                                motor_states[i].pul_state = false;
                            }
                        }
                    } else {
                        motor_states[i].accumulator = 0;
                        if (motor_states[i].pul_state) {
                            motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
                            motor_states[i].pul_state = false;
                        }
                    }
                }

                if (all_homed) {
                    g_flag_homing_all_switched = true;
                    current_homing_state = HOMING_STATE_BACKOFF;
                    
                    // Đảo chiều DIR xuống
                    HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, HOMING_DIR_DOWN_LEVEL);
                    HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, HOMING_DIR_DOWN_LEVEL);
                    HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, HOMING_DIR_DOWN_LEVEL);
                    
                    // [MICROSTEP 1/4]: Backoff 400Hz cho microstep 1/4
                    uint32_t backoff_freq_hz = 200; 
                    uint32_t backoff_speed_addend = (uint32_t)(((uint64_t)backoff_freq_hz * DDS_CONST));
                    
                    for (int i = 0; i < 3; i++) {
                        homing_backoff_done[i] = false;
                        homing_confirmation_counter[i] = 0;
                        homing_backoff_steps[i] = 0; 
                        g_flag_ls_stuck[i] = false;
                        motor_states[i].accumulator = 0;
                        motor_states[i].pul_state = false;
                        motor_states[i].speed_addend = backoff_speed_addend; 
                    }
                    g_flag_homing_backing_off_start = true;
                }
                break;
            }
            
            case HOMING_STATE_BACKOFF: {
                bool all_backoff_done = true;
                
                for (int i = 0; i < 3; i++) {
                    if (!homing_backoff_done[i]) {
                        // [OPTIMIZATION] Dùng hàm fast check
                        if (!robot_is_limit_switch_triggered_fast(i)) { // Nhả = LOW (hoặc ngược lại với active level)
                             // Logic: !triggered nghĩa là đã nhả
                             homing_confirmation_counter[i]++;
                             if (homing_confirmation_counter[i] >= HOMING_BACKOFF_DEBOUNCE_COUNT) {
                                 homing_backoff_done[i] = true;
                                 absolute_motor_steps[i] = STEPS_AT_HOME_NEG45;
                             }
                        } else { homing_confirmation_counter[i] = 0; }

                        if (homing_backoff_steps[i] > HOMING_MAX_BACKOFF_STEPS) {
                            homing_backoff_done[i] = true;
                            g_flag_ls_stuck[i] = true;
                        }
                    }
                }
                
                for (int i = 0; i < 3; i++) {
                    if (!homing_backoff_done[i]) {
                        all_backoff_done = false;
                        motor_states[i].active = true;
                        
                        // DDS Backoff
                        uint32_t old_acc = motor_states[i].accumulator;
                        motor_states[i].accumulator += motor_states[i].speed_addend;
                        
                        if (motor_states[i].accumulator < old_acc) {
                            motor_states[i].pul_port->BSRR = motor_states[i].pul_pin;
                            motor_states[i].pul_state = true;
                            absolute_motor_steps[i]++; 
                            homing_backoff_steps[i]++; 
                        } else {
                            if (motor_states[i].pul_state) {
                                motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
                                motor_states[i].pul_state = false;
                            }
                        }
                    } else {
                        motor_states[i].accumulator = 0;
                        motor_states[i].active = false;
                        if (motor_states[i].pul_state) {
                            motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
                            motor_states[i].pul_state = false;
                        }
                    }
                }
                
                if (all_backoff_done) {
                    current_homing_state = HOMING_STATE_DONE;
                }
                break;
            }

            case HOMING_STATE_DONE: {
                HAL_TIM_Base_Stop_IT(&htim3);
                current_robot_state = ROBOT_STATE_IDLE;
                current_homing_state = HOMING_STATE_IDLE;
                robot_homed = true;
                
                if (!servo_pwm_started) {
                    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
                    servo_pwm_started = true;
                }
                uint16_t pulse_135deg = robot_servo_angle_to_pulse(13500);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pulse_135deg);
                
                g_flag_homing_done = true;
                break;
            }
            default: break;
        }
    }
}

RobotState robot_get_state(void) {
    return current_robot_state;
}

HomingState robot_get_homing_state(void) {
    return current_homing_state;
}

bool robot_is_homed(void) {
    return robot_homed;
}

void robot_update_state(void) {}

void robot_get_absolute_steps(int32_t* steps_array) {
    steps_array[0] = absolute_motor_steps[0];
    steps_array[1] = absolute_motor_steps[1];
    steps_array[2] = absolute_motor_steps[2];
}

void robot_update_homing_state(void) {}

void robot_start_homing(void) {
    if (current_robot_state != ROBOT_STATE_IDLE) {
        cdc_handler_send_response("ERROR:Robot is busy");
        return;
    }

    for(int i=0; i<3; i++) homing_motor_done[i] = false;
    robot_homed = false; 
    absolute_motor_steps[0] = 0;
    absolute_motor_steps[1] = 0;
    absolute_motor_steps[2] = 0;

    current_robot_state = ROBOT_STATE_HOMING;
    current_homing_state = HOMING_STATE_RAISING;
    homing_start_tick_ms = HAL_GetTick(); 
    homing_start_steps[0] = 0;
    homing_start_steps[1] = 0;
    homing_start_steps[2] = 0;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, robot_servo_angle_to_pulse(13500));

    HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, HOMING_DIR_UP_LEVEL);
    HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, HOMING_DIR_UP_LEVEL);
    HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, HOMING_DIR_UP_LEVEL);
    
    GPIO_PinState init_ls1 = HAL_GPIO_ReadPin(LS1_GPIO_Port, LS1_Pin);
    GPIO_PinState init_ls2 = HAL_GPIO_ReadPin(LS2_GPIO_Port, LS2_Pin);
    GPIO_PinState init_ls3 = HAL_GPIO_ReadPin(LS3_GPIO_Port, LS3_Pin);
    cdc_handler_send_response("DEBUG:LS_INIT_RAW:LS1=%d,LS2=%d,LS3=%d", init_ls1, init_ls2, init_ls3);

    if (init_ls1 == GPIO_PIN_SET) { homing_motor_done[0] = true; }
    if (init_ls2 == GPIO_PIN_SET) { homing_motor_done[1] = true; }
    if (init_ls3 == GPIO_PIN_SET) { homing_motor_done[2] = true; }
    
    if (homing_motor_done[0] && homing_motor_done[1] && homing_motor_done[2]) {
        current_homing_state = HOMING_STATE_BACKOFF;
        HAL_GPIO_WritePin(M1_DIR_GPIO_Port, M1_DIR_Pin, HOMING_DIR_DOWN_LEVEL);
        HAL_GPIO_WritePin(M2_DIR_GPIO_Port, M2_DIR_Pin, HOMING_DIR_DOWN_LEVEL);
        HAL_GPIO_WritePin(M3_DIR_GPIO_Port, M3_DIR_Pin, HOMING_DIR_DOWN_LEVEL);
        cdc_handler_send_response("DEBUG:ALL_LS_PRE_TRIGGERED, skip RAISING");
    } else {
        current_homing_state = HOMING_STATE_RAISING;
    }

    // [UPDATE] Tăng tốc độ Raising một chút vì vi bước nhỏ
    // 667Hz cũ -> 667*2 = 1334Hz (microstep 1/4)
    uint32_t raising_freq_hz = 800; 
    uint32_t raising_speed_addend = (uint32_t)(((uint64_t)raising_freq_hz * DDS_CONST));
    
    for (int i = 0; i < 3; i++) {
        motor_states[i].active = true;  
        motor_states[i].speed_addend = raising_speed_addend;  
        motor_states[i].accumulator = 0;  
        motor_states[i].pul_state = false; 
        homing_confirmation_counter[i] = 0; 
        homing_backoff_done[i] = false; 
        homing_backoff_steps[i] = 0; 
        g_flag_ls_stuck[i] = false;   
        
        if (i == 0) { motor_states[i].pul_port = M1_PUL_GPIO_Port; motor_states[i].pul_pin = M1_PUL_Pin; }
        else if (i == 1) { motor_states[i].pul_port = M2_PUL_GPIO_Port; motor_states[i].pul_pin = M2_PUL_Pin; }
        else { motor_states[i].pul_port = M3_PUL_GPIO_Port; motor_states[i].pul_pin = M3_PUL_Pin; }
    }

    HAL_GPIO_WritePin(M1_ENA_GPIO_Port, M1_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_ENA_GPIO_Port, M2_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_ENA_GPIO_Port, M3_ENA_Pin, GPIO_PIN_RESET);

    cdc_handler_send_response("ACK:HOME started");

    HAL_TIM_Base_Start_IT(&htim3);
}

void robot_estop_triggered(void) {
    HAL_TIM_Base_Stop_IT(&htim3); 
    conveyor_stop(); 
    queue_flush();
    current_robot_state = ROBOT_STATE_IDLE;
    status_led_set_status(LED_STATUS_ESTOP);
}

void robot_estop_released(void) {
    current_robot_state = ROBOT_STATE_IDLE;
    current_homing_state = HOMING_STATE_IDLE; // ✅ Reset Homing State
    robot_homed = false;                      // ✅ Reset Homing Status
    queue_flush();
    status_led_set_status(LED_STATUS_IDLE);
    cdc_handler_send_response("ESTOP_OFF");
}

void robot_abort(void) {
    HAL_TIM_Base_Stop_IT(&htim3);
    current_robot_state = ROBOT_STATE_IDLE;
    current_homing_state = HOMING_STATE_IDLE;
    robot_homed = false; 
    for (int i = 0; i < 3; i++) {
        homing_motor_done[i] = false;
        homing_confirmation_counter[i] = 0;
        motor_states[i].active = false;
        motor_states[i].steps_to_go = 0;
        motor_states[i].accumulator = 0;  
        if (motor_states[i].pul_state) {
            motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
            motor_states[i].pul_state = false;
        }
    }
    status_led_set_status(LED_STATUS_IDLE);
}

void robot_disable_motors(void) {
    HAL_TIM_Base_Stop_IT(&htim3);
    for (int i = 0; i < 3; i++) {
        motor_states[i].active = false;
        motor_states[i].steps_to_go = 0;
        motor_states[i].accumulator = 0;  
        motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
        motor_states[i].pul_state = false;
    }
    current_robot_state = ROBOT_STATE_IDLE;
    current_homing_state = HOMING_STATE_IDLE;

    HAL_GPIO_WritePin(M1_ENA_GPIO_Port, M1_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M2_ENA_GPIO_Port, M2_ENA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(M3_ENA_GPIO_Port, M3_ENA_Pin, GPIO_PIN_SET);
    
    for (volatile int i = 0; i < 1000; i++); 
}

void robot_enable_motors(void) {
    HAL_TIM_Base_Stop_IT(&htim3);
    for (int i = 0; i < 3; i++) {
        motor_states[i].active = false;
        motor_states[i].steps_to_go = 0;
        motor_states[i].accumulator = 0;  
        motor_states[i].pul_port->BSRR = (uint32_t)motor_states[i].pul_pin << 16U;
        motor_states[i].pul_state = false;
    }
    current_robot_state = ROBOT_STATE_IDLE;
    current_homing_state = HOMING_STATE_IDLE;

    HAL_GPIO_WritePin(M1_ENA_GPIO_Port, M1_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_ENA_GPIO_Port, M2_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_ENA_GPIO_Port, M3_ENA_Pin, GPIO_PIN_RESET);
    
    for (volatile int i = 0; i < 1000; i++); 
    conveyor_stop();
}

uint32_t robot_get_homing_start_tick(void) {
    return homing_start_tick_ms;
}

bool robot_get_and_clear_flag_homing_timeout_backing_off(void) {
    bool flag = g_flag_homing_timeout_backing_off;
    g_flag_homing_timeout_backing_off = false;
    return flag;
}

bool robot_get_and_clear_flag_homing_all_switched(void) {
    bool flag = g_flag_homing_all_switched;
    g_flag_homing_all_switched = false;
    return flag;
}

bool robot_get_and_clear_flag_homing_backing_off_start(void) {
    bool flag = g_flag_homing_backing_off_start;
    g_flag_homing_backing_off_start = false;
    return flag;
}

bool robot_get_and_clear_flag_homing_dir_misconfig(int* motor_idx) {
    bool flag = g_flag_homing_dir_misconfig;
    if (flag && motor_idx != NULL) {
        *motor_idx = g_flag_homing_dir_misconfig_motor;
    }
    g_flag_homing_dir_misconfig = false;
    g_flag_homing_dir_misconfig_motor = -1;
    return flag;
}

bool robot_get_and_clear_flag_homing_ls_trig(int idx) {
    if (idx < 0 || idx >= 3) return false;
    bool flag = g_flag_homing_ls_trig[idx];
    g_flag_homing_ls_trig[idx] = false;
    return flag;
}

bool robot_get_and_clear_flag_homing_early_fallback(int idx) {
    if (idx < 0 || idx >= 3) return false;
    bool flag = g_flag_homing_early_fallback[idx];
    g_flag_homing_early_fallback[idx] = false;
    return flag;
}

bool robot_get_and_clear_flag_up_blocked(int idx) {
    if (idx < 0 || idx >= 3) return false;
    bool flag = g_flag_up_blocked[idx];
    g_flag_up_blocked[idx] = false;
    return flag;
}

bool robot_get_and_clear_flag_homing_done(void) {
    bool flag = g_flag_homing_done;
    g_flag_homing_done = false;
    return flag;
}

bool robot_set_servo_angle_manual(uint32_t angle_x100) {
    // ✅ Luôn cho phép điều khiển Servo (không cần Home)
    if (!servo_pwm_started) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        servo_pwm_started = true;
    }
    // Cập nhật thời gian hoạt động
    last_servo_activity_tick = HAL_GetTick();

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, robot_servo_angle_to_pulse(angle_x100));
    return true;
}

bool robot_get_and_clear_flag_ls_stuck(int idx) {
    if (idx < 0 || idx >= 3) return false;
    bool flag = g_flag_ls_stuck[idx];
    g_flag_ls_stuck[idx] = false;
    return flag;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        robot_timer_irq_handler(htim);
    }
}

void robot_poll_servo_idle(void) {
    if (servo_pwm_started) {
        if (HAL_GetTick() - last_servo_activity_tick > SERVO_IDLE_TIMEOUT_MS) {
            // Quá thời gian chờ -> Tắt PWM để servo "nghỉ" (không gầm, không lắc)
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
            servo_pwm_started = false;
        }
    }
}
