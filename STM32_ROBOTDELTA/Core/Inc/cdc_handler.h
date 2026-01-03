/* Mở file: cdc_handler.h */
#ifndef __CDC_HANDLER_H
#define __CDC_HANDLER_H

#include "usbd_cdc_if.h"
#include <stdbool.h> // Thêm thư viện này

// ✅ FIX BUG 1: Ring Buffer để tránh mất gói tin USB CDC
// Kích thước bộ đệm nhận lệnh (chuỗi hoàn chỉnh)
#define RX_BUFFER_SIZE 512  // ✅ Tăng lên 512 để nhận lệnh ADD_BLOCK dài

// Kích thước Ring Buffer cho dữ liệu thô từ USB (PHẢI là lũy thừa của 2 để tối ưu)
#define RING_BUFFER_SIZE 2048  // ✅ Tăng lên 2048 để đủ cho burst 44 lệnh ADD_BLOCK

void cdc_handler_init(void);
void cdc_handler_reset(void);
void cdc_handler_receive(uint8_t* buf, uint32_t len);
void cdc_handler_send_response(const char* format, ...);

// ✅ Hàm xử lý Ring Buffer - main loop gọi hàm này để parse lệnh từ dữ liệu thô
/**
 * @brief Xử lý dữ liệu từ Ring Buffer và tìm lệnh hoàn chỉnh (kết thúc bằng '\n').
 * @param out_buffer Buffer để sao chép lệnh vào.
 * @param max_len Kích thước của out_buffer.
 * @return true nếu có lệnh mới, false nếu không.
 * @note Hàm này PHẢI được gọi liên tục từ main loop để tránh Ring Buffer tràn.
 */
bool cdc_handler_get_command(char* out_buffer, uint16_t max_len);

#endif /* __CDC_HANDLER_H */