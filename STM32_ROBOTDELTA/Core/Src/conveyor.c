#include "conveyor.h"
#include "cdc_handler.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include <math.h>

extern TIM_HandleTypeDef htim2; // Timer cho PWM băng tải

// ============================================================================
// TÍNH TOÁN THÔNG SỐ TỪ CƠ HỌC
// ============================================================================
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Chu vi puly (mm)
#define PULLEY_CIRCUMFERENCE_MM     (2.0f * M_PI * CONVEYOR_PULLEY_RADIUS_MM)

// Số xung cần để di chuyển 1mm
// steps_per_mm = (STEPS_PER_REV * GEAR_RATIO) / CIRCUMFERENCE
#define STEPS_PER_MM                ((float)CONVEYOR_STEPS_PER_REV * CONVEYOR_GEAR_RATIO / PULLEY_CIRCUMFERENCE_MM)

// Timer clock (sau prescaler) - giả sử 1MHz (PSC=71 với 72MHz system clock)
#define TIMER_CLOCK_HZ              1000000UL

// Tốc độ tối đa và tối thiểu (mm/s)
#define SPEED_MIN_MM_S              1
#define SPEED_MAX_MM_S              100

// ============================================================================
// BIẾN NỘI BỘ
// ============================================================================
static volatile bool is_running = false;
static uint16_t current_speed_mm_s = 40;  // Tốc độ mặc định 40 mm/s

// Debounce cho cảm biến khay
#define TRAY_SENSOR_DEBOUNCE_COUNT 10
static uint8_t tray_sensor_debounce_counter = 0;
static bool tray_sensor_stable_state = false;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Tính tần số PWM (Hz) từ tốc độ mm/s
 * 
 * Công thức:
 *   freq_hz = speed_mm_s * STEPS_PER_MM
 * 
 * Ví dụ: speed = 100 mm/s, STEPS_PER_MM = 17
 *   freq = 100 * 17 = 1700 Hz
 */
static uint32_t speed_to_frequency(uint16_t speed_mm_s) {
    if (speed_mm_s < SPEED_MIN_MM_S) speed_mm_s = SPEED_MIN_MM_S;
    if (speed_mm_s > SPEED_MAX_MM_S) speed_mm_s = SPEED_MAX_MM_S;
    
    float freq = (float)speed_mm_s * STEPS_PER_MM;
    return (uint32_t)freq;
}

/**
 * @brief Tính giá trị ARR từ tần số mong muốn
 * 
 * ARR = TIMER_CLOCK / freq - 1
 */
static uint32_t frequency_to_arr(uint32_t freq_hz) {
    if (freq_hz == 0) return 65535;  // Tránh chia cho 0
    
    uint32_t arr = TIMER_CLOCK_HZ / freq_hz;
    if (arr < 2) arr = 2;           // ARR tối thiểu
    if (arr > 65535) arr = 65535;   // ARR tối đa (16-bit timer)
    
    return arr - 1;
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

void conveyor_init(void) {
    // Tắt ENA khi khởi động (HIGH = disable cho TB6600)
    HAL_GPIO_WritePin(CONV_ENA_GPIO_Port, CONV_ENA_Pin, GPIO_PIN_SET);
    
    is_running = false;
    
    // Cấu hình PWM channel
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 100;  // Placeholder, sẽ set lại khi start
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
    
    cdc_handler_send_response("INFO:Conveyor init, STEPS_PER_MM=%.2f", STEPS_PER_MM);
}

void conveyor_set_speed_mm_s(uint16_t speed_mm_s) {
    // Clamp speed
    if (speed_mm_s < SPEED_MIN_MM_S) speed_mm_s = SPEED_MIN_MM_S;
    if (speed_mm_s > SPEED_MAX_MM_S) speed_mm_s = SPEED_MAX_MM_S;
    
    current_speed_mm_s = speed_mm_s;
    
    // Nếu đang chạy, cập nhật ngay lập tức
    if (is_running) {
        uint32_t freq = speed_to_frequency(current_speed_mm_s);
        uint32_t arr = frequency_to_arr(freq);
        
        __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2);  // Duty 50%
    }
}

uint16_t conveyor_get_speed_mm_s(void) {
    return current_speed_mm_s;
}

void conveyor_start(bool forward) {
    // Set hướng
    HAL_GPIO_WritePin(CONV_DIR_GPIO_Port, CONV_DIR_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    
    // Tính tần số và ARR
    uint32_t freq = speed_to_frequency(current_speed_mm_s);
    uint32_t arr = frequency_to_arr(freq);
    
    // Cấu hình timer
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr / 2);  // Duty 50%
    
    // Enable driver và start PWM
    HAL_GPIO_WritePin(CONV_ENA_GPIO_Port, CONV_ENA_Pin, GPIO_PIN_RESET);  // LOW = enable
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    
    is_running = true;
}

void conveyor_stop(void) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(CONV_ENA_GPIO_Port, CONV_ENA_Pin, GPIO_PIN_SET);  // HIGH = disable
    
    is_running = false;
}

bool conveyor_is_running(void) {
    return is_running;
}

void conveyor_irq_handler(void) {
    // Hiện tại không dùng đếm xung
    // Có thể mở rộng sau nếu cần đo quãng đường đã chạy
}

bool conveyor_is_tray_detected(void) {
    GPIO_PinState current_reading = HAL_GPIO_ReadPin(CONV_SENSOR_GPIO_Port, CONV_SENSOR_Pin);
    bool target_state = (current_reading == GPIO_PIN_SET);
    
    if (target_state == tray_sensor_stable_state) {
        tray_sensor_debounce_counter = 0;
    } else {
        tray_sensor_debounce_counter++;
        if (tray_sensor_debounce_counter >= TRAY_SENSOR_DEBOUNCE_COUNT) {
            tray_sensor_stable_state = target_state;
            tray_sensor_debounce_counter = 0;
        }
    }
    return tray_sensor_stable_state;
}
