#include "command_queue.h"
#include "robot_control.h"
#include "cdc_handler.h"
#include <string.h>

#define QUEUE_SIZE 64 // Tăng lên 64 để tận dụng RAM thừa và chạy mượt hơn với S-Curve segments dày đặc
#define DONE_QUEUE_SIZE 128 // ✅ Tăng kích thước hàng đợi DONE để tránh mất mát gói tin khi chạy nhanh

static MotionBlock queue_buffer[QUEUE_SIZE];
static volatile uint8_t head = 0; // Vị trí để ghi khối lệnh mới
static volatile uint8_t tail = 0; // Vị trí để đọc khối lệnh tiếp theo
static volatile uint8_t count = 0; // Số lượng khối lệnh trong hàng đợi

static volatile uint32_t done_queue[DONE_QUEUE_SIZE];
static volatile uint8_t done_head = 0;
static volatile uint8_t done_tail = 0;

static uint32_t current_block_id = 0; // ID của khối lệnh đang được thực thi

void queue_init(void) {
    head = 0;
    tail = 0;
    count = 0;
    current_block_id = 0;
    done_head = 0;
    done_tail = 0;
}

bool queue_is_full(void) {
    return count >= QUEUE_SIZE;
}

bool queue_add_block(const MotionBlock* block) {
    if (queue_is_full()) {
        cdc_handler_send_response("ERROR:Queue is full");
        return false;
    }

    // Tắt ngắt tạm thời để đảm bảo tính toàn vẹn dữ liệu
    __disable_irq();
    
    memcpy(&queue_buffer[head], block, sizeof(MotionBlock));
    head = (head + 1) % QUEUE_SIZE;
    count++;

    // Bật lại ngắt
    __enable_irq();

    return true; // ACK sẽ được gửi bởi command_parser
}

void queue_process(void) {
    // Nếu robot đang rảnh, và có lệnh trong hàng đợi, và không có lệnh nào đang chạy
    if (robot_get_state() == ROBOT_STATE_IDLE && count > 0 && current_block_id == 0) {
        // Tắt ngắt tạm thời
        __disable_irq();

        // Lấy khối lệnh tiếp theo từ hàng đợi
        MotionBlock* next_block = &queue_buffer[tail];
        current_block_id = next_block->id; // Lưu lại ID để báo cáo khi xong
        
        tail = (tail + 1) % QUEUE_SIZE;
        count--;

        // Bật lại ngắt
        __enable_irq();

        // Giao cho robot thực thi
        robot_execute_block(next_block);
    }
}

void queue_finish_current_block(void) {
    if (current_block_id != 0) {
        // ✅ FIX: Protect critical section from interrupt race condition
        __disable_irq(); // Bắt đầu vùng nguy hiểm
        
        // Thay vì gửi trực tiếp, đẩy vào hàng đợi done
        uint8_t next_head = (done_head + 1) % DONE_QUEUE_SIZE;
        if (next_head != done_tail) {
            done_queue[done_head] = current_block_id;
            done_head = next_head;
        }
        current_block_id = 0; // Đánh dấu đã xong
        
        __enable_irq();  // Kết thúc vùng nguy hiểm
    }
}

bool queue_pop_next_from_isr(MotionBlock* block) {
    if (count > 0) {
        *block = queue_buffer[tail];
        current_block_id = block->id; // Cập nhật ID mới ngay lập tức
        tail = (tail + 1) % QUEUE_SIZE;
        count--;
        return true;
    }
    return false;
}

void queue_handle_done_messages(void) {
    while (done_head != done_tail) {
        uint32_t id = done_queue[done_tail];
        done_tail = (done_tail + 1) % DONE_QUEUE_SIZE;
        cdc_handler_send_response("DONE:%lu", id);
    }
}

void queue_flush(void) {
    // Tắt ngắt tạm thời
    __disable_irq();

    // Reset hàng đợi lệnh
    head = 0;
    tail = 0;
    count = 0;
    current_block_id = 0; // ✅ FIX: Giải phóng hàng đợi để nhận lệnh mới sau khi Flush/E-Stop
    
    // Reset hàng đợi done? Không nên, cứ để nó gửi nốt
    
    // Bật lại ngắt
    __enable_irq();
}

void queue_flush_after_current(void) {
    // ✅ Xóa tất cả lệnh TRONG QUEUE, NHƯNG giữ lại lệnh đang thực thi
    // Lệnh đang thực thi (current_block_id != 0) sẽ hoàn thành bình thường
    // và gửi DONE như thường lệ
    
    __disable_irq();
    
    // Reset queue - xóa hết lệnh chưa bắt đầu
    head = 0;
    tail = 0;
    count = 0;
    
    // KHÔNG reset current_block_id - lệnh hiện tại vẫn chạy tiếp
    
    __enable_irq();
}