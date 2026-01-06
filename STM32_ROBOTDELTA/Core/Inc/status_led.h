#ifndef __STATUS_LED_H
#define __STATUS_LED_H

#include "main.h"

/**
 * @brief Các trạng thái của hệ thống được hiển thị trên LED
 */
typedef enum {
    LED_STATUS_IDLE,        // ✅ Bình thường - Đèn xanh sáng
    LED_STATUS_RUNNING,     // 🟢 Đang chạy - Đèn xanh nhấp nháy
    LED_STATUS_ERROR,       // 🔴 Lỗi - Đèn đỏ sáng
    LED_STATUS_ERROR_FLASH, // 🔴 Lỗi nghiêm trọng - Đèn đỏ nhấp nháy
    LED_STATUS_ESTOP        // ⚠️ Emergency Stop - Cả 2 đèn nhấp nháy xen kẽ
} LEDStatus;

/**
 * @brief Khởi tạo module LED trạng thái
 */
void status_led_init(void);

/**
 * @brief Cập nhật trạng thái LED
 * @param status Trạng thái cần hiển thị
 */
void status_led_set_status(LEDStatus status);

/**
 * @brief Cập nhật LED (gọi trong vòng lặp chính hoặc timer)
 */
void status_led_update(void);

/**
 * @brief Lấy trạng thái LED hiện tại
 */
LEDStatus status_led_get_current_status(void);

/**
 * @brief Tự động cập nhật LED status dựa trên trạng thái robot và E-STOP
 * @note Gọi trong main loop để STM32 tự quản lý đèn xanh/đỏ
 */
void robot_update_status_leds(void);

#endif // __STATUS_LED_H
