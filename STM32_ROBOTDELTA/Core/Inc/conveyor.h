#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

// ============================================================================
// THÔNG SỐ CƠ HỌC BĂNG TẢI - CẦN ĐIỀU CHỈNH THEO THỰC TẾ
// ============================================================================

// Driver TB6600: Số xung (step) trên 1 vòng motor (tùy thuộc cấu hình DIP switch)
// Ví dụ: 1600 pulse/rev (microstepping 1/8), 3200 pulse/rev (1/16)
#define CONVEYOR_STEPS_PER_REV      1600    // Số xung/vòng motor

// Tỉ số truyền (nếu có hộp số hoặc puly giảm tốc)
// Ví dụ: Motor quay 2 vòng -> Puly chủ động quay 1 vòng => RATIO = 2.0
#define CONVEYOR_GEAR_RATIO         2.0f    // Tỉ số truyền (1.0 = truyền trực tiếp)

// Bán kính puly chủ động (mm)
// Chu vi puly = 2 * PI * R => Quãng đường = Chu vi * số vòng
#define CONVEYOR_PULLEY_RADIUS_MM   18.14f   // Bán kính puly (mm)



// ============================================================================
// API FUNCTIONS
// ============================================================================

// Khởi tạo băng tải (PWM, GPIO)
void conveyor_init(void);

// Đặt tốc độ theo mm/s (sẽ được lưu và áp dụng khi START)
void conveyor_set_speed_mm_s(uint16_t speed_mm_s);

// Lấy tốc độ hiện tại (mm/s)
uint16_t conveyor_get_speed_mm_s(void);

// Chạy băng tải (forward = true: tiến, false: lùi)
void conveyor_start(bool forward);

// Dừng băng tải
void conveyor_stop(void);

// Kiểm tra băng tải có đang chạy không
bool conveyor_is_running(void);

// Hàm gọi trong ngắt Timer (nếu cần đếm xung cho tương lai)
void conveyor_irq_handler(void);

// Kiểm tra cảm biến khay (có debounce)
bool conveyor_is_tray_detected(void);

#endif // CONVEYOR_H
