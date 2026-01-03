#include "command_parser.h"
#include "robot_control.h"
#include "conveyor.h"
#include "command_queue.h"
#include "cdc_handler.h"
#include "button_handler.h"
#include "status_led.h"  // ✅ Added missing include
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// --- Các hàm trợ giúp phân tích JSON thủ công ---
static const char* find_json_value_start(const char* json, const char* key) {
    const char* key_ptr = strstr(json, key);
    if (key_ptr == NULL) return NULL;
    const char* value_ptr = key_ptr + strlen(key);
    while (*value_ptr == ' ' || *value_ptr == ':') value_ptr++;
    return value_ptr;
}

// ⚠️ REMOVED: parse_float_value() - Chuyển sang dùng integer (fixed-point)

static bool parse_long_value(const char* json, const char* key, long* value) {
    const char* value_str = find_json_value_start(json, key);
    if (value_str) { *value = atol(value_str); return true; }
    return false;
}
static bool parse_int_array(const char* json, const char* key, int32_t* arr, int max_elements) {
    const char* array_str = find_json_value_start(json, key);
    if (!array_str || *array_str != '[') return false;
    array_str++;
    char* temp_str = (char*)array_str;
    for (int i = 0; i < max_elements; ++i) {
        arr[i] = atol(temp_str);
        while (*temp_str != ',' && *temp_str != ']') {
            if (*temp_str == '\0') return false;
            temp_str++;
        }
        if (*temp_str == ']') return (i == max_elements - 1);
        temp_str++;
    }
    return false;
}

// --- Hàm xử lý lệnh (SAFE PARSER) ---
// Tách ID từ rest của command string bằng cách tìm ':'
static bool extract_id_and_payload(char* input, char* out_id, size_t id_size, char** out_payload) {
    char* colon = strchr(input, ':');
    if (!colon) return false;
    
    // Copy ID
    size_t id_len = colon - input;
    if (id_len >= id_size) return false;
    strncpy(out_id, input, id_len);
    out_id[id_len] = '\0';
    
    // Payload là phần sau ':'
    *out_payload = colon + 1;
    return true;
}

// Hàm wrapper cho strtok - không còn sử dụng, xóa để tránh warning

static void handle_add_block(char* rest) {
    char id_str[32] = {0};
    char* json_payload = NULL;
    
    // Parse ID and JSON payload safely
    if (!extract_id_and_payload(rest, id_str, sizeof(id_str), &json_payload)) {
        cdc_handler_send_response("ERROR:Invalid ADD_BLOCK format (no colon)");
        return;
    }
    
    // Debug: In ra ID string và phần đầu JSON
    // ✅ PERFORMANCE: Tắt debug log này để tránh tràn buffer USB khi gửi ADD_BLOCK liên tục (JOG)
    // cdc_handler_send_response("DEBUG:ADD_BLOCK id='%s', json start='%.15s'", id_str, json_payload);
    
    // Xóa newline ở cuối JSON nếu có
    if (json_payload != NULL) {
        size_t len = strlen(json_payload);
        if (len > 0 && json_payload[len - 1] == '\n') {
            json_payload[len - 1] = '\0';
        }
    }
    
    if (json_payload == NULL || *json_payload != '{') {
        cdc_handler_send_response("ERROR:Invalid ADD_BLOCK format (json=%s, first='%c')",
                                 json_payload ? "OK" : "NULL",
                                 json_payload ? *json_payload : '?');
        return;
    }
    MotionBlock new_block = {0};
    new_block.id = atol(id_str);
    long b_val = 0;
    long angle_x100 = 0;      // ✅ FIXED-POINT: Góc * 100 (27000 = 270.0°)
    long time_ms = 0;         // ✅ FIXED-POINT: Thời gian tính bằng mili-giây
    bool ok_t = parse_long_value(json_payload, "\"t\"", &time_ms);
    bool ok_s = parse_int_array(json_payload, "\"s\"", new_block.motor_steps, 3);
    bool ok_a = parse_long_value(json_payload, "\"a\"", &angle_x100);
    bool ok_b = parse_long_value(json_payload, "\"b\"", &b_val);

    if (!ok_t || !ok_s || !ok_a || !ok_b) {
        cdc_handler_send_response("ERROR:ADD_BLOCK parse fail: t=%d s=%d a=%d b=%d", 
                                 ok_t, ok_s, ok_a, ok_b);
        return;
    }

    // Validation mức cơ bản (dùng integer)
    // ✅ FIX BUG 3: Thêm kiểm tra time_ms < 10 để tránh chia cho 0 và tần số quá cao
    if (time_ms < 10 || time_ms > 30000) { // 10ms-30 giây = 10-30000ms
        cdc_handler_send_response("ERROR:ADD_BLOCK invalid time (must be 10-30000 ms)");
        return;
    }
    for (int i = 0; i < 3; ++i) {
        if (labs(new_block.motor_steps[i]) > 200000) {
            cdc_handler_send_response("ERROR:ADD_BLOCK steps too large");
            return;
        }
    }
    // ✅ OPTIMIZATION: PC gửi góc tọa độ (coordinate angle), STM32 chuyển đổi sang góc vật lý
    // Góc tọa độ: -225° đến 45° (tương ứng -22500 đến 4500 trong fixed-point)
    // Góc vật lý: 0° đến 270° (tương ứng 0 đến 27000 trong fixed-point)
    // Công thức: physical = coord + 225
    if (angle_x100 < -22500L || angle_x100 > 4500L) { // -225.00° đến 45.00° = -22500 đến 4500
        cdc_handler_send_response("ERROR:ADD_BLOCK servo coord angle out of range (-22500 to 4500)");
        return;
    }
    if (!(b_val == 0 || b_val == 1)) {
        cdc_handler_send_response("ERROR:ADD_BLOCK pump state invalid");
        return;
    }
    
    // ✅ Chuyển đổi góc tọa độ sang góc vật lý: physical = coord + 225
    // Đảm bảo kết quả luôn >= 0 và <= 27000
    long angle_physical_x100 = angle_x100 + 22500L;  // -22500 + 22500 = 0, 4500 + 22500 = 27000
    
    // ✅ VALIDATION: Đảm bảo góc vật lý trong phạm vi hợp lệ (0-27000)
    if (angle_physical_x100 < 0 || angle_physical_x100 > 27000L) {
        cdc_handler_send_response("ERROR:ADD_BLOCK servo physical angle out of range (0 to 27000)");
        return;
    }
    
    // ✅ FIXED-POINT: Chuyển angle_physical_x100 (0-27000) sang pulse (500-2500)
    // angle_degree = angle_physical_x100 / 100
    // pulse = 500 + (angle_degree / 270) * 2000
    // = 500 + (angle_physical_x100 / 100 / 270) * 2000
    // = 500 + (angle_physical_x100 * 2000) / 27000
    // = 500 + (angle_physical_x100 * 20) / 270
    new_block.servo_pulse = 500 + (uint16_t)((angle_physical_x100 * 20) / 270);
    // ✅ FIX: b=1 → pump ON, b=0 → pump OFF (Active High)
    new_block.pump_state = (bool)b_val;
    
    // ✅ CRITICAL: Lưu duration (ms) vào block để tính DDS chính xác
    // PC đã gửi time_ms (milliseconds) - lưu trực tiếp
    new_block.duration = time_ms;
    
    // *** [DEPRECATED] Tính step_interval - Giữ lại để tương thích ngược ***
    // Lưu ý: Code DDS mới KHÔNG dùng step_interval nữa, dùng duration
    // Tìm số bước lớn nhất để tính tần số motor
    uint32_t max_steps = 0;
    for (int i = 0; i < 3; i++) {
        uint32_t abs_steps = (uint32_t)labs(new_block.motor_steps[i]);
        if (abs_steps > max_steps) {
            max_steps = abs_steps;
        }
    }
    
    // Tính step_interval (deprecated, chỉ để backup)
    #define STEPPER_TIMER_FREQ 40000
    
    if (time_ms > 0 && max_steps > 0) {
        uint32_t motor_freq_x1000 = (uint32_t)(((uint64_t)max_steps * 1000000ULL) / time_ms);
        uint32_t max_freq_x1000 = 20000UL * 1000UL;
        if (motor_freq_x1000 > max_freq_x1000) {
            motor_freq_x1000 = max_freq_x1000;
        }
        if (motor_freq_x1000 > 100) {
            new_block.step_interval = (40000000UL) / motor_freq_x1000;
        } else {
            new_block.step_interval = 0xFFFFFFFF;
        }
    } else {
        new_block.step_interval = 2;
    }
    
    queue_add_block(&new_block);
    // ✅ PERFORMANCE: Bỏ ACK cho ADD_BLOCK để giảm tải USB
    // PC không cần ACK, chỉ cần nhận DONE khi hoàn thành
    cdc_handler_send_response("ACK:%ld", new_block.id);
}

// --- Hàm xử lý lệnh CONVEYOR được cập nhật (an toàn hơn, không dùng strtok) ---
static void handle_conveyor(char* rest) {
    // Tìm `:` để tách action
    char* colon = strchr(rest, ':');
    char action[32] = {0};
    char* payload = "";
    
    if (colon) {
        size_t action_len = colon - rest;
        if (action_len >= sizeof(action)) {
            cdc_handler_send_response("ERROR:CONVEYOR action too long");
            return;
        }
        strncpy(action, rest, action_len);
        action[action_len] = '\0';
        payload = colon + 1;
    } else {
        // Không có `:` - lệnh đơn giản như "CONVEYOR:STOP"
        strncpy(action, rest, sizeof(action) - 1);
        action[sizeof(action) - 1] = '\0';
        payload = "";
    }
    
    // Xóa newline nếu có
    size_t len = strlen(action);
    if (len > 0 && action[len - 1] == '\n') {
        action[len - 1] = '\0';
    }

    if (strcmp(action, "START") == 0) {
        // Xóa newline từ payload
        char direction[32] = {0};
        strncpy(direction, payload, sizeof(direction) - 1);
        len = strlen(direction);
        if (len > 0 && direction[len - 1] == '\n') {
            direction[len - 1] = '\0';
        }
        
        if (strlen(direction) == 0) {
            cdc_handler_send_response("ERROR:Missing direction");
            return;
        }
        
        if (strcmp(direction, "FWD") == 0) {
            conveyor_start(true);
            cdc_handler_send_response("ACK:START_FWD");
        } else if (strcmp(direction, "REV") == 0) {
            conveyor_start(false);
            cdc_handler_send_response("ACK:START_REV");
        } else {
            cdc_handler_send_response("ERROR:Invalid direction");
        }
    }
    else if (strcmp(action, "STOP") == 0) {
        conveyor_stop();
        cdc_handler_send_response("ACK:STOP");
    }
    else if (strcmp(action, "SET_SPEED") == 0) {
        char speed_str[32] = {0};
        strncpy(speed_str, payload, sizeof(speed_str) - 1);
        len = strlen(speed_str);
        if (len > 0 && speed_str[len - 1] == '\n') {
            speed_str[len - 1] = '\0';
        }
        
        if (strlen(speed_str) == 0) {
            cdc_handler_send_response("ERROR:Missing speed value");
            return;
        }
        
        // Tốc độ theo mm/s (1-500)
        uint16_t speed_mm_s = atoi(speed_str);
        conveyor_set_speed_mm_s(speed_mm_s);
        cdc_handler_send_response("ACK:SET_SPEED:%u mm/s", speed_mm_s);
    }
    else if (strcmp(action, "GET_SPEED") == 0) {
        uint16_t speed = conveyor_get_speed_mm_s();
        cdc_handler_send_response("ACK:GET_SPEED:%u mm/s", speed);
    }
    else if (strcmp(action, "STATUS") == 0) {
        bool running = conveyor_is_running();
        uint16_t speed = conveyor_get_speed_mm_s();
        cdc_handler_send_response("ACK:CONVEYOR_STATUS:%s,%u mm/s", 
                                  running ? "RUNNING" : "STOPPED", speed);
    }
    else {
        cdc_handler_send_response("ERROR:Unknown CONVEYOR action '%s'", action);
    }
}

// --- Hàm xử lý lệnh điều khiển LED nút bấm ---
static void handle_button_led(char* rest) {
    // Format: BTN_LED:id:state
    // id: 0 (START), 1 (STOP)
    // state: 0 (OFF), 1 (ON)
    
    char* colon = strchr(rest, ':');
    if (!colon) {
        cdc_handler_send_response("ERROR:BTN_LED missing second colon");
        return;
    }
    
    // Parse ID
    size_t id_len = colon - rest;
    char id_str[8] = {0};
    if (id_len >= sizeof(id_str)) {
        cdc_handler_send_response("ERROR:BTN_LED id too long");
        return;
    }
    strncpy(id_str, rest, id_len);
    id_str[id_len] = '\0';
    int btn_id = atoi(id_str);
    
    // Parse State
    int state = atoi(colon + 1);
    
    if (btn_id >= 0 && btn_id < BUTTON_COUNT) {
        button_set_led((ButtonId)btn_id, (bool)state);
        // cdc_handler_send_response("ACK:BTN_LED:%d:%d", btn_id, state); // Tạm tắt ACK nếu gửi nhiều
    } else {
        cdc_handler_send_response("ERROR:Invalid button ID %d", btn_id);
    }
}

// --- Hàm gửi báo cáo trạng thái toàn diện ---
void send_status_report(void) {
    // 1. Run State (0=Idle/Estop, 1=Moving/Homing) & Homed State
    RobotState rs = robot_get_state();
    int run_state = (rs == ROBOT_STATE_MOVING || rs == ROBOT_STATE_HOMING) ? 1 : 0;
    int is_homed = robot_is_homed() ? 1 : 0;
    
    // 2. E-Stop & Buttons
    int estop = estop_is_triggered() ? 1 : 0;
    int btn_start = button_is_pressed(BUTTON_1) ? 1 : 0;
    int btn_stop = button_is_pressed(BUTTON_2) ? 1 : 0;
    
    // 3. Conveyor (-1: Rev, 0: Stop, 1: Fwd) & Speed
    int conv_status = 0;
    if (conveyor_is_running()) {
        GPIO_PinState dir_state = HAL_GPIO_ReadPin(CONV_DIR_GPIO_Port, CONV_DIR_Pin);
        conv_status = (dir_state == GPIO_PIN_RESET) ? 1 : -1;
    }
    uint16_t conv_speed = conveyor_get_speed_mm_s();
    
    // 4. Pump
    int pump_status = (HAL_GPIO_ReadPin(PUMP_GPIO_Port, PUMP_Pin) == GPIO_PIN_SET) ? 1 : 0;
    
    // 5. Servo Angle
    uint32_t servo_pulse = TIM4->CCR4;
    long servo_angle_x100 = 0;
    if (servo_pulse >= 500) {
        servo_angle_x100 = (long)((servo_pulse - 500) * 270 / 20);
    }
    long servo_coord_x100 = servo_angle_x100 - 22500L;
    
    // 6. Tray Sensor
    int tray_sensor = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;
    
    // 7. Motor Steps
    int32_t steps[3];
    robot_get_absolute_steps(steps);
    
    cdc_handler_send_response("STATUS:%d:%d:%d:%d:%d:%d:%u:%d:%ld:%d:%ld,%ld,%ld", 
                             run_state, is_homed, estop, btn_start, btn_stop, 
                             conv_status, conv_speed, pump_status, servo_coord_x100, tray_sensor,
                             steps[0], steps[1], steps[2]);
}

// --- Hàm phân tích lệnh chính ---
void parse_command(char* command_string) {
    // Tìm `:` đầu tiên để tách command name
    char* first_colon = strchr(command_string, ':');
    
    char command_name[32] = {0};
    char* rest = NULL;
    
    if (first_colon == NULL) {
        // Không có `:` - lệnh đơn giản (PING, HOME, STATUS, etc)
        // Copy toàn bộ command_string, xóa newline nếu có
        strncpy(command_name, command_string, sizeof(command_name) - 1);
        command_name[sizeof(command_name) - 1] = '\0';
        
        // Xóa newline ở cuối
        size_t len = strlen(command_name);
        if (len > 0 && command_name[len - 1] == '\n') {
            command_name[len - 1] = '\0';
        }
        rest = "";
    } else {
        // Có `:` - lệnh phức tạp (ADD_BLOCK:..., CONVEYOR:..., etc)
        size_t cmd_len = first_colon - command_string;
        if (cmd_len >= sizeof(command_name)) {
            cdc_handler_send_response("ERROR:Command name too long");
            return;
        }
        strncpy(command_name, command_string, cmd_len);
        command_name[cmd_len] = '\0';
        rest = first_colon + 1;
    }
    
    if (strlen(command_name) == 0) {
        cdc_handler_send_response("ERROR:Empty command");
        return;
    }

    if (strcmp(command_name, "PING") == 0) {
        // ✅ AUTO-ENABLE: Tự động kích hoạt motors khi nhận PING (kết nối mới)
        robot_enable_motors();
        
        // ✅ NEW: Gửi kèm trạng thái E-Stop để PC đồng bộ ngay khi kết nối
        if (estop_is_triggered()) {
            cdc_handler_send_response("PONG:ESTOP");
        } else {
            cdc_handler_send_response("PONG");
        }
    }
    else if (strcmp(command_name, "HOME") == 0) {
        robot_start_homing();
    }
    else if (strcmp(command_name, "FLUSH_BUFFER") == 0) {
        robot_abort(); // Dừng mọi hoạt động
        queue_flush(); // Xóa hàng đợi
        // Giữ ENA không thả (theo yêu cầu: chỉ disconnect mới thả ENA)
        cdc_handler_send_response("ACK:FLUSH");
    }
    else if (strcmp(command_name, "FLUSH_AFTER_CURRENT") == 0) {
        // ✅ NEW: Xóa queue nhưng giữ lại lệnh đang chạy
        // Không gọi robot_abort() - để lệnh hiện tại hoàn thành tự nhiên
        queue_flush_after_current();
        cdc_handler_send_response("ACK:FLUSH_AFTER_CURRENT");
    }
    // ✅ REMOVED: CLEAR_QUEUE - Trùng chức năng với FLUSH_BUFFER, đã xóa
    // Dùng FLUSH_BUFFER thay thế
    // ✅ REMOVED: ABORT - Không được sử dụng, đã xóa
    // Dùng FLUSH_BUFFER thay thế nếu cần dừng + xóa queue
    // else if (strcmp(command_name, "ABORT") == 0) {
    //     robot_abort();
    //     cdc_handler_send_response("ACK:ABORT");
    // }
    else if (strcmp(command_name, "CONVEYOR") == 0) {
        handle_conveyor(rest);
    }
    else if (strcmp(command_name, "ADD_BLOCK") == 0) {
        handle_add_block(rest);
    }
    else if (strcmp(command_name, "STATUS") == 0) {
        send_status_report();
    }
    // ✅ REMOVED: ENABLE/DISABLE handlers - Không dùng trong flow chính
    // (PING tự động ENABLE, RESET_SYSTEM tự động DISABLE)
    // Giữ lại code nếu cần debug thủ công:
    /*
    else if (strcmp(command_name, "DISABLE") == 0) {
        robot_disable_motors();
        cdc_handler_send_response("ACK:Motors Disabled");
    }
    else if (strcmp(command_name, "ENABLE") == 0) {
        robot_enable_motors();
        cdc_handler_send_response("ACK:Motors Enabled");
    }
    */
    
    // ✅ REMOVED: SERVO handler - Không được dùng (servo điều khiển qua ADD_BLOCK)
    // Giữ lại code nếu cần debug thủ công:
    /*
    else if (strcmp(command_name, "SERVO") == 0) {
        long angle_x100 = atol(rest);
        if (robot_set_servo_angle_manual((uint32_t)angle_x100)) {
            cdc_handler_send_response("ACK:SERVO:%ld", angle_x100);
        } else {
            cdc_handler_send_response("ERROR:Robot not homed, cannot control servo");
        }
    }
    */
    else if (strcmp(command_name, "RESET_SYSTEM") == 0) {
        // ✅ NEW: Reset toàn bộ hệ thống (tương đương ngắt kết nối)
        robot_abort();
        queue_flush();
        conveyor_stop();
        robot_disable_motors();
        cdc_handler_reset();
        cdc_handler_send_response("ACK:SYSTEM_RESET");
    }
    else if (strcmp(command_name, "BTN_LED") == 0) {
        handle_button_led(rest);
    }
    else if (strcmp(command_name, "SYS_LED") == 0) {
        // SYS_LED:status_code
        // 0: IDLE, 1: RUNNING, 2: ERROR, 3: ERROR_FLASH, 4: ESTOP
        if (rest) {
            int status = atoi(rest);
            status_led_set_status((LEDStatus)status);
            // cdc_handler_send_response("ACK:SYS_LED:%d", status);
        }
    }
    else {
        // ✅ OPTIMIZATION: Nếu không match lệnh nào, kiểm tra xem có phải ADD_BLOCK không có prefix không
        // Format: id:json (ví dụ: 0:{"t":500,"s":[100,200,300],"a":13500,"b":1})
        // Nếu command_name là số (toàn số) và có ':' tiếp theo, coi là ADD_BLOCK
        bool is_numeric = true;
        for (size_t i = 0; i < strlen(command_name); i++) {
            if (command_name[i] < '0' || command_name[i] > '9') {
                is_numeric = false;
                break;
            }
        }
        
        // ✅ FIX: Kiểm tra kỹ hơn - rest phải tồn tại, không rỗng, và bắt đầu bằng '{' để xác nhận là JSON
        if (is_numeric && strlen(command_name) > 0 && rest != NULL && strlen(rest) > 0) {
            // Tìm ký tự đầu tiên không phải whitespace trong rest
            const char* json_start = rest;
            while (*json_start == ' ' || *json_start == '\t') {
                json_start++;
            }
            
            // Kiểm tra rest bắt đầu bằng '{' để xác nhận là JSON
            if (*json_start == '{') {
                // ✅ OPTIMIZATION: ADD_BLOCK không có prefix - Format: id:json
                // command_name là id, rest là json payload
                // handle_add_block() cần format id:json, nên pass toàn bộ command_string
                handle_add_block(command_string);  // command_string = "id:json"
            } else {
                cdc_handler_send_response("ERROR:ADD_BLOCK format invalid (expected JSON starting with '{', got '%c')", *json_start ? *json_start : '\0');
            }
        } else {
            cdc_handler_send_response("ERROR:Unknown command '%s'", command_name);
        }
    }
}