/* Mở file: cdc_handler.c */

#include "cdc_handler.h"
#include "usbd_cdc_if.h"
#include "command_parser.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// ✅ FIX BUG 1: Circular (Ring) Buffer Implementation
// Cấu trúc này lưu trữ dữ liệu thô từ USB CDC ISR
// Main loop sẽ đọc từ đây để parse thành lệnh
typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    volatile uint16_t head; // Vị trí ghi (ISR ghi vào đây)
    volatile uint16_t tail; // Vị trí đọc (Main loop đọc từ đây)
} RingBuffer_t;

static RingBuffer_t ring_buffer;

// ✅ PERFORMANCE: Non-blocking send - Không còn timeout
// CDC_Transmit_FS chỉ thử 1 lần, nếu BUSY thì bỏ qua

// Bộ đệm tạm để ghép các byte thành lệnh hoàn chỉnh (dùng bởi main loop)
static char rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_buffer_ptr = 0;

// ✅ OPTIMIZATION: Ring Buffer với Bit Mask (nhanh hơn modulo)
// Điều kiện: RING_BUFFER_SIZE phải là lũy thừa của 2 (2048 = 2^11 ✓)
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1)

// ✅ OPTIMIZATION: Overflow counter để debug
static volatile uint32_t ring_buffer_overflow_count = 0;

// ✅ Ring Buffer Helper Functions
/**
 * @brief Kiểm tra xem Ring Buffer có trống không.
 */
static inline bool ring_buffer_is_empty(void) {
    return (ring_buffer.head == ring_buffer.tail);
}

/**
 * @brief Kiểm tra xem Ring Buffer có đầy không.
 * ✅ OPTIMIZATION: Dùng bit mask thay vì modulo (nhanh hơn 5-10x)
 */
static inline bool ring_buffer_is_full(void) {
    return (((ring_buffer.head + 1) & RING_BUFFER_MASK) == ring_buffer.tail);
}

/**
 * @brief Ghi 1 byte vào Ring Buffer (từ ISR).
 * @return true nếu thành công, false nếu buffer đầy.
 * ✅ OPTIMIZATION: Dùng bit mask thay vì modulo
 */
static bool ring_buffer_put(uint8_t data) {
    if (ring_buffer_is_full()) {
        ring_buffer_overflow_count++; // ✅ DEBUG: Đếm số lần overflow
        return false; // Buffer đầy, mất dữ liệu (cần tăng RING_BUFFER_SIZE)
    }
    ring_buffer.buffer[ring_buffer.head] = data;
    ring_buffer.head = (ring_buffer.head + 1) & RING_BUFFER_MASK; // ✅ Bit mask
    return true;
}

/**
 * @brief Đọc 1 byte từ Ring Buffer (từ main loop).
 * @return true nếu đọc được, false nếu buffer trống.
 * ✅ OPTIMIZATION: Dùng bit mask thay vì modulo
 */
static bool ring_buffer_get(uint8_t* data) {
    if (ring_buffer_is_empty()) {
        return false; // Không có dữ liệu
    }
    *data = ring_buffer.buffer[ring_buffer.tail];
    ring_buffer.tail = (ring_buffer.tail + 1) & RING_BUFFER_MASK; // ✅ Bit mask
    return true;
}

void cdc_handler_init(void)
{
    ring_buffer.head = 0;
    ring_buffer.tail = 0;
    rx_buffer_ptr = 0;
    rx_buffer[0] = '\0';
}

void cdc_handler_reset(void)
{
    ring_buffer.head = 0;
    ring_buffer.tail = 0;
    rx_buffer_ptr = 0;
    rx_buffer[0] = '\0';
}

/**
 * @brief Callback từ USB CDC ISR - PHẢI thực thi cực nhanh.
 * @note Chỉ việc ghi dữ liệu vào Ring Buffer, KHÔNG parse ở đây.
 */
void cdc_handler_receive(uint8_t* buf, uint32_t len)
{
    // ✅ FIX BUG 1: Không còn kiểm tra command_ready
    // Luôn chấp nhận dữ liệu mới từ USB và ghi vào Ring Buffer
    for (uint32_t i = 0; i < len; i++)
    {
        if (!ring_buffer_put(buf[i])) {
            // Ring Buffer đầy - Đây là trường hợp hiếm (Main loop bị kẹt)
            // Có thể đặt cờ cảnh báo ở đây nếu cần debug
            break;
        }
    }
}

/**
 * @brief Xử lý dữ liệu từ Ring Buffer và tìm lệnh hoàn chỉnh.
 * @note Hàm này PHẢI được gọi liên tục từ main loop.
 */
bool cdc_handler_get_command(char* out_buffer, uint16_t max_len)
{
    // Đọc từng byte từ Ring Buffer cho đến khi gặp '\n' hoặc buffer đầy
    uint8_t byte;
    while (ring_buffer_get(&byte))
    {
        // Nếu nhận được ký tự xuống dòng hoặc buffer đầy
        if (byte == '\n' || rx_buffer_ptr >= RX_BUFFER_SIZE - 1)
        {
            rx_buffer[rx_buffer_ptr] = '\0'; // Kết thúc chuỗi

            if (rx_buffer_ptr > 0) // Nếu có dữ liệu (tránh dòng trống)
            {
                // ✅ OPTIMIZATION: Dùng rx_buffer_ptr thay vì strlen() (nhanh hơn)
                // ✅ OPTIMIZATION: Dùng memcpy thay vì strcpy/strncpy (nhanh hơn 2-3x)
                uint16_t copy_len = (rx_buffer_ptr < max_len) ? rx_buffer_ptr : (max_len - 1);
                memcpy(out_buffer, rx_buffer, copy_len);
                out_buffer[copy_len] = '\0'; // Đảm bảo null terminator
                
                rx_buffer_ptr = 0; // Reset con trỏ buffer cho lệnh tiếp theo
                return true; // Có lệnh mới
            }
            rx_buffer_ptr = 0; // Reset nếu là dòng trống
        }
        else if (byte != '\r') // Bỏ qua ký tự '\r'
        {
            rx_buffer[rx_buffer_ptr++] = byte; // Thêm ký tự vào buffer
        }
    }
    
    return false; // Chưa có lệnh hoàn chỉnh
}


void cdc_handler_send_response(const char* format, ...)
{
    // ✅ OPTIMIZATION: Tăng buffer size từ 128 lên 256 để đủ cho phản hồi dài
    char buffer[256]; 
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0)
    {
        // Đảm bảo chuỗi kết thúc bằng '\n'
        if (len > sizeof(buffer) - 2) len = sizeof(buffer) - 2;
        buffer[len] = '\n';
        buffer[len + 1] = '\0';
        len++; // Tăng len để bao gồm cả '\n'

        // ✅ PERFORMANCE: NON-BLOCKING - Chỉ thử gửi 1 lần
        // Nếu USB bận (PC không kịp đọc), bỏ qua gói tin này
        // Robot chạy mượt quan trọng hơn là gửi phản hồi
        uint8_t tx_status = CDC_Transmit_FS((uint8_t*)buffer, len);
        
        // Không còn vòng lặp chờ - Nếu bận thì chấp nhận mất gói tin
        if (tx_status == USBD_BUSY)
        {
            // Bỏ qua, không thể gửi - Robot tiếp tục hoạt động bình thường
        }
    }
}