#ifndef __ROBOT_CONTROL_H
#define __ROBOT_CONTROL_H

#include "main.h" // Include main.h to get all HAL definitions
#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal_tim.h"

/**
 * @brief Trạng thái của quá trình Homing.
 */
typedef enum {
    HOMING_STATE_IDLE,          // Chưa bắt đầu hoặc đã hoàn thành
    HOMING_STATE_RAISING,       // Đang đi lên để chạm công tắc hành trình
    HOMING_STATE_BACKOFF,       // Đang hạ xuống để cảm biến nhả ra (back-off)
    HOMING_STATE_DONE           // Đã hoàn thành (trạng thái tạm thời trước khi về IDLE)
} HomingState;


#define HOMING_WATCHDOG_MS 15000 // 15 giây tối đa cho toàn bộ quy trình homing

// --- Public Function Prototypes ---

/**
 * @brief Khởi tạo module điều khiển robot, bao gồm các chân GPIO và timer.
 */
void robot_init(void);

/**
 * @brief Bắt đầu thực thi một khối lệnh chuyển động (MotionBlock).
 * @param block Con trỏ tới khối lệnh cần thực thi.
 */
void robot_execute_block(const MotionBlock* block);

/**
 * @brief Bắt đầu quy trình Homing cho robot.
 */
void robot_start_homing(void);

/**
 * @brief Lấy trạng thái hiện tại của robot.
 * @return Trạng thái RobotState.
 */
RobotState robot_get_state(void);

/**
 * @brief Lấy trạng thái hiện tại của quá trình Homing.
 * @return Trạng thái HomingState.
 */
HomingState robot_get_homing_state(void);

/**
 * @brief Lấy số bước tuyệt đối của các động cơ.
 * @param steps_array Mảng 3 phần tử để lưu số bước.
 */
void robot_get_absolute_steps(int32_t* steps_array);

/**
 * @brief Lấy tick hệ thống (ms) tại thời điểm bắt đầu homing.
 * @return Giá trị HAL_GetTick() lúc bắt đầu homing.
 */
uint32_t robot_get_homing_start_tick(void);

/**
 * @brief Cập nhật trạng thái homing (được gọi từ main loop).
 */
void robot_update_homing_state(void);

/**
 * @brief Dừng khẩn cấp mọi hoạt động của robot.
 */
void robot_abort(void);

/**
 * @brief Vô hiệu hóa tất cả các động cơ (thả trôi).
 */
void robot_disable_motors(void);

/**
 * @brief Kích hoạt lại tất cả các động cơ (enable motors).
 */
void robot_enable_motors(void);

/**
 * @brief Xử lý khi E-Stop được kích hoạt.
 */
void robot_estop_triggered(void);

/**
 * @brief Xử lý khi E-Stop được nhả.
 */
void robot_estop_released(void);

/**
 * @brief Chuyển đổi góc servo sang giá trị pulse cho timer.
 * @param angle_x100 Góc * 100 (0-27000 = 0-270.00°)
 */
uint16_t robot_servo_angle_to_pulse(uint32_t angle_x100);

/**
 * @brief Kiểm tra xem robot đã hoàn thành homing chưa.
 * @return true nếu đã home, false nếu chưa.
 */
bool robot_is_homed(void);

/**
 * @brief Điều khiển servo thủ công (✅ Luôn cho phép - không cần home).
 * @param angle_x100 Góc VẬT LÝ * 100 (0-27000 = 0-270°).
 * @return true nếu thành công.
 */
bool robot_set_servo_angle_manual(uint32_t angle_x100);
bool robot_set_servo_angle_manual(uint32_t angle_x100);

// --- Get-and-clear flag functions for main loop ---
bool robot_get_and_clear_flag_homing_timeout_backing_off(void);
bool robot_get_and_clear_flag_homing_all_switched(void);
bool robot_get_and_clear_flag_homing_backing_off_start(void);
bool robot_get_and_clear_flag_homing_dir_misconfig(int* motor_idx);
bool robot_get_and_clear_flag_homing_ls_trig(int idx);
bool robot_get_and_clear_flag_homing_early_fallback(int idx);
bool robot_get_and_clear_flag_up_blocked(int idx);
bool robot_get_and_clear_flag_homing_done(void);
bool robot_get_and_clear_flag_ls_stuck(int idx);

/**
 * @brief Kiểm tra và tự động ngắt PWM servo nếu không hoạt động quá lâu (Giảm nhiễu/lắc)
 * Gọi hàm này trong vòng lặp chính (main loop)
 */
void robot_poll_servo_idle(void);

#endif // __ROBOT_CONTROL_H