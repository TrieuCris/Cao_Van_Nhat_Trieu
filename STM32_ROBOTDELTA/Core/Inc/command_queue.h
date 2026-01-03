#ifndef __COMMAND_QUEUE_H
#define __COMMAND_QUEUE_H

#include "main.h"

/**
 * @brief Khởi tạo hàng đợi lệnh.
 */
void queue_init(void);

/**
 * @brief Thêm một MotionBlock vào cuối hàng đợi.
 * @param block Con trỏ tới MotionBlock cần thêm.
 * @return true nếu thêm thành công, false nếu hàng đợi đầy.
 */
bool queue_add_block(const MotionBlock* block);

/**
 * @brief Xử lý hàng đợi.
 *        Hàm này nên được gọi liên tục trong vòng lặp chính.
 *        Nó sẽ kiểm tra xem robot có rảnh không và có lệnh trong hàng đợi không,
 *        nếu có, nó sẽ lấy lệnh tiếp theo và yêu cầu robot thực thi.
 */
void queue_process(void);

/**
 * @brief Báo cho hàng đợi biết rằng khối lệnh hiện tại đã hoàn thành.
 *        Hàm này sẽ được gọi bởi module robot_control khi một di chuyển kết thúc.
 */
void queue_finish_current_block(void);

/**
 * @brief Xóa tất cả các khối lệnh đang chờ trong hàng đợi.
 *        Lệnh đang thực thi (nếu có) sẽ không bị ảnh hưởng.
 */
void queue_flush(void);

/**
 * @brief Xóa tất cả lệnh trong queue NHƯNG giữ lại lệnh đang thực thi.
 *        Robot sẽ thực hiện nốt lệnh hiện tại, gửi DONE, rồi dừng.
 *        Dùng cho JOG stop: dừng nhanh nhưng vẫn giữ vị trí chính xác.
 */
void queue_flush_after_current(void);

/**
 * @brief Kiểm tra xem hàng đợi có đầy không.
 * @return true nếu đầy, false nếu còn chỗ.
 */
bool queue_is_full(void);

/**
 * @brief Lấy khối lệnh tiếp theo từ hàng đợi (dùng trong ISR).
 * @param block Con trỏ để lưu khối lệnh lấy được.
 * @return true nếu lấy thành công, false nếu hàng đợi rỗng.
 */
bool queue_pop_next_from_isr(MotionBlock* block);

/**
 * @brief Xử lý và gửi các thông báo DONE từ hàng đợi sự kiện (dùng trong main loop).
 */
void queue_handle_done_messages(void);

#endif // __COMMAND_QUEUE_H