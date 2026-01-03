#ifndef __BUTTON_HANDLER_H
#define __BUTTON_HANDLER_H

#include "main.h"
#include <stdbool.h>

// Định nghĩa các nút bấm
typedef enum {
    BUTTON_1 = 0,      // PB6
    BUTTON_2 = 1,      // PB7
    BUTTON_COUNT
} ButtonId;

/**
 * @brief Khởi tạo module xử lý nút bấm
 */
void button_handler_init(void);

/**
 * @brief Cập nhật trạng thái các nút bấm (gọi từ main loop hoặc timer)
 * Nên gọi mỗi 1-5ms
 */
void button_handler_update(void);

/**
 * @brief Kiểm tra nút có đang được nhấn không
 * @param btn_id ID của nút cần kiểm tra
 * @return true nếu nút đang được nhấn
 */
bool button_is_pressed(ButtonId btn_id);

/**
 * @brief Kiểm tra nút vừa được nhấn xuống (edge detection)
 * @param btn_id ID của nút cần kiểm tra
 * @return true chỉ ở chu kỳ đầu tiên khi nút được nhấn
 */
bool button_just_pressed(ButtonId btn_id);

/**
 * @brief Kiểm tra nút vừa được nhả ra (edge detection)
 * @param btn_id ID của nút cần kiểm tra
 * @return true chỉ ở chu kỳ đầu tiên khi nút được nhả
 */
bool button_just_released(ButtonId btn_id);

/**
 * @brief Kiểm tra E-Stop có đang được kích hoạt không (có debounce)
 * @return true nếu E-Stop đang ở trạng thái khẩn cấp
 */
bool estop_is_triggered(void);

/**
 * @brief Kiểm tra E-Stop vừa được kích hoạt (edge detection)
 * @return true chỉ ở chu kỳ đầu tiên khi E-Stop được kích hoạt
 */
bool estop_just_triggered(void);

/**
 * @brief Kiểm tra E-Stop vừa được nhả (edge detection)
 * @return true chỉ ở chu kỳ đầu tiên khi E-Stop được nhả
 */
bool estop_just_released(void);

/**
 * @brief Điều khiển LED của nút bấm
 * @param btn_id ID của nút
 * @param state true = SÁNG, false = TẮT
 */
void button_set_led(ButtonId btn_id, bool state);

#endif // __BUTTON_HANDLER_H
