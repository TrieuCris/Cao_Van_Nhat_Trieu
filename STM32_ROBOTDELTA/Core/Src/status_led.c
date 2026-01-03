#include "status_led.h"
#include "stm32f1xx_hal.h"

// Biến trạng thái LED
static volatile LEDStatus current_status = LED_STATUS_IDLE;
static uint32_t last_toggle_tick = 0;

void status_led_init(void)
{
    // Khởi tạo LED
    // PA10 (Green LED) - Set = ON, Reset = OFF (Active High)
    // PA15 (Red LED) - Set = ON, Reset = OFF (Active High)
    
    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);   // Tắt xanh
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);       // Tắt đỏ
    
    current_status = LED_STATUS_IDLE;
    last_toggle_tick = HAL_GetTick();
}

void status_led_set_status(LEDStatus status)
{
    if (current_status != status) {
        current_status = status;
        // Reset trạng thái đèn về mặc định khi chuyển state để tránh bị "kẹt" ở trạng thái tắt
        if (status == LED_STATUS_IDLE) {
             HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
             HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
        } else if (status == LED_STATUS_ERROR) {
             HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
             HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
        }
        last_toggle_tick = HAL_GetTick();
    }
}

LEDStatus status_led_get_current_status(void)
{
    return current_status;
}

void status_led_update(void)
{
    uint32_t current_tick = HAL_GetTick();
    
    switch (current_status)
    {
        case LED_STATUS_IDLE:
            // ✅ Idle: Đèn xanh sáng liên tục, đèn đỏ tắt
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);    // ON
            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);      // OFF
            break;
            
        case LED_STATUS_RUNNING:
            // 🟢 Running: Đèn xanh nhấp nháy 500ms (1Hz), đèn đỏ tắt
            if (current_tick - last_toggle_tick >= 500) {
                HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
                last_toggle_tick = current_tick;
            }
            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);      // OFF
            break;
            
        case LED_STATUS_ERROR:
            // 🔴 Error: Đèn đỏ sáng liên tục, đèn xanh tắt
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);  // OFF
            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);        // ON
            break;
            
        case LED_STATUS_ERROR_FLASH:
            // 🔴 Error Flash: Đèn đỏ nhấp nháy 200ms (nhanh), đèn xanh tắt
            if (current_tick - last_toggle_tick >= 200) {
                HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
                last_toggle_tick = current_tick;
            }
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);  // OFF
            break;
            
        case LED_STATUS_ESTOP:
            // ⚠️ E-Stop: Đèn ĐỎ nhấp nháy 1000ms (chậm - giống GUI), Đèn Xanh tắt
            if (current_tick - last_toggle_tick >= 1000) {
                HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
                last_toggle_tick = current_tick;
            }
            HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);  // Xanh OFF
            break;
            
        default:
            status_led_set_status(LED_STATUS_IDLE);
            break;
    }
}
