#include "button_handler.h"
#include "robot_control.h"
#include <string.h>

// Cấu hình debounce
#define DEBOUNCE_TIME_MS 20        // Thời gian debounce (ms)
#define ESTOP_DEBOUNCE_TIME_MS 10  // Thời gian debounce cho E-Stop (ngắn hơn để phản ứng nhanh)

// Cấu trúc lưu trạng thái nút bấm
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t debounce_counter;
    bool current_state;       // Trạng thái hiện tại sau debounce
    bool previous_state;      // Trạng thái chu kỳ trước
    bool pressed_event;       // Cờ đánh dấu sự kiện nhấn nút (để tránh spam)
    bool is_active_high;      // true: nhấn = HIGH, false: nhấn = LOW
} ButtonState;

// Trạng thái các nút bấm
static ButtonState buttons[BUTTON_COUNT];
static ButtonState estop_state;

// Thời gian cập nhật lần cuối (để tính debounce)
static uint32_t last_update_tick = 0;

void button_handler_init(void) {
    // Cấu hình Button 1 (START) - NO với VCC
    buttons[BUTTON_1].port = BTN_START_GPIO_Port;
    buttons[BUTTON_1].pin = BTN_START_Pin;
    buttons[BUTTON_1].is_active_high = true;  // Active High: nhấn = 1 (VCC)
    
    // Cấu hình Button 2 (STOP) - NO với VCC
    buttons[BUTTON_2].port = BTN_STOP_GPIO_Port;
    buttons[BUTTON_2].pin = BTN_STOP_Pin;
    buttons[BUTTON_2].is_active_high = true;  // Active High: nhấn = 1 (VCC)
    
    // Cấu hình E-Stop (PA8) - NC với PULLUP
    estop_state.port = ESTOP_GPIO_Port;
    estop_state.pin = ESTOP_Pin;
    estop_state.debounce_counter = 0;
    estop_state.current_state = false;
    estop_state.previous_state = false;
    estop_state.is_active_high = true;  // NC + PULLUP: kích hoạt = HIGH
    
    // Tắt LED nút bấm khi khởi động
    button_set_led(BUTTON_1, false);
    // button_set_led(BUTTON_2, false); // Nút STOP không có LED

    last_update_tick = HAL_GetTick();
}

/**
 * @brief Cập nhật trạng thái một nút với debounce
 */
static void update_button_state(ButtonState* btn, uint8_t debounce_threshold) {
    // Đọc trạng thái GPIO
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(btn->port, btn->pin);
    
    // Xác định trạng thái logic (có xét đến active high/low)
    bool target_state;
    if (btn->is_active_high) {
        target_state = (pin_state == GPIO_PIN_SET);
    } else {
        target_state = (pin_state == GPIO_PIN_RESET);
    }
    
    // Thuật toán debounce
    if (target_state == btn->current_state) {
        // Trạng thái ổn định, reset counter
        btn->debounce_counter = 0;
    } else {
        // Trạng thái đang thay đổi, tăng counter
        btn->debounce_counter++;
        
        // Nếu đủ số lần đọc liên tiếp, chấp nhận thay đổi
        if (btn->debounce_counter >= debounce_threshold) {
            btn->previous_state = btn->current_state;
            btn->current_state = target_state;
            btn->debounce_counter = 0;
            
            // Nếu chuyển từ KHÔNG nhấn sang NHẤN -> Set cờ sự kiện
            if (btn->current_state == true && btn->previous_state == false) {
                btn->pressed_event = true;
            }
        }
    }
}

void button_handler_update(void) {
    uint32_t current_tick = HAL_GetTick();
    
    // Chỉ cập nhật mỗi 1ms để có độ phân giải debounce tốt
    if (current_tick - last_update_tick < 1) {
        return;
    }
    last_update_tick = current_tick;
    
    // Cập nhật các nút bấm thường (debounce 20ms = 20 lần đọc)
    for (int i = 0; i < BUTTON_COUNT; i++) {
        update_button_state(&buttons[i], DEBOUNCE_TIME_MS);
    }
    
    // Cập nhật E-Stop (debounce 10ms = 10 lần đọc, nhanh hơn)
    bool prev_estop = estop_state.current_state;
    update_button_state(&estop_state, ESTOP_DEBOUNCE_TIME_MS);
    
    // Nếu E-Stop thay đổi trạng thái, gọi callback
    if (estop_state.current_state != prev_estop) {
        if (estop_state.current_state) {
            // E-Stop vừa được kích hoạt
            robot_estop_triggered();
        } else {
            // E-Stop vừa được nhả
            robot_estop_released();
        }
    }
}

bool button_is_pressed(ButtonId btn_id) {
    if (btn_id >= BUTTON_COUNT) return false;
    return buttons[btn_id].current_state;
}

bool button_just_pressed(ButtonId btn_id) {
    if (btn_id >= BUTTON_COUNT) return false;
    
    // Kiểm tra cờ sự kiện đã được set bởi bộ debounce chưa
    if (buttons[btn_id].pressed_event) {
        buttons[btn_id].pressed_event = false; // Xóa cờ (consume event)
        return true;
    }
    return false;
}

bool button_just_released(ButtonId btn_id) {
    if (btn_id >= BUTTON_COUNT) return false;
    return (!buttons[btn_id].current_state && buttons[btn_id].previous_state);
}

bool estop_is_triggered(void) {
    return estop_state.current_state;
}

bool estop_just_triggered(void) {
    return (estop_state.current_state && !estop_state.previous_state);
}

bool estop_just_released(void) {
    return (!estop_state.current_state && estop_state.previous_state);
}

void button_set_led(ButtonId btn_id, bool state) {
    if (btn_id == BUTTON_1) {
        // BUTTON_1 (START) -> BUTTON_LED_Pin (PB3)
        HAL_GPIO_WritePin(BUTTON_LED_GPIO_Port, BUTTON_LED_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    } 
    // BUTTON_2 (STOP) không có đèn LED (PB4 not connected)
}