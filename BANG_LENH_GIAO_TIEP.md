# BẢNG CÁC LỆNH GIAO TIẾP (COMMUNICATION PROTOCOL)

Dưới đây là danh sách các gói tin được sử dụng để giao tiếp giữa Máy tính (PC) và Vi điều khiển (STM32).

| STT | Tên Lệnh (Command) | Cấu Trúc Gói Tin (Syntax) | Mô Tả Chức Năng (Description) | Ví Dụ (Example) |
| :--- | :--- | :--- | :--- | :--- |
| 1 | **PING** | `PING` | Kiểm tra kết nối. STM32 sẽ phản hồi `PONG` nếu sẵn sàng. | `PING` |
| 2 | **ADD_BLOCK** | `ADD_BLOCK:ID:JSON` | Thêm một lệnh di chuyển vào hàng đợi. JSON chứa thời gian (t), bước (s), góc servo (a), bơm (b). | `ADD_BLOCK:101:{"t":1500,"s":[100,200,300],"a":-9000,"b":1}` |
| 3 | **STATUS** | `STATUS` | Yêu cầu STM32 báo cáo trạng thái hiện tại (Vị trí, Cảm biến, Công tắc hành trình). | `STATUS` |
| 4 | **HOME** | `HOME` | Yêu cầu Robot thực hiện quy trình về gốc (Homing Sequence) để xác định tọa độ chuẩn. | `HOME` |
| 5 | **FLUSH_BUFFER** | `FLUSH_BUFFER` | Xóa sạch hàng đợi lệnh hiện tại và dừng động cơ ngay lập tức (Dùng cho nút Stop). | `FLUSH_BUFFER` |
| 6 | **FLUSH_AFTER** | `FLUSH_AFTER_CURRENT` | Xóa các lệnh trong hàng đợi nhưng cho phép lệnh đang chạy được hoàn thành (Dùng cho Jog Stop). | `FLUSH_AFTER_CURRENT` |
| 7 | **CONV_START** | `CONVEYOR:START:DIR` | Bật băng tải. `DIR` là hướng (`FWD` hoặc `REV`). | `CONVEYOR:START:FWD` |
| 8 | **CONV_STOP** | `CONVEYOR:STOP` | Dừng băng tải. | `CONVEYOR:STOP` |
| 9 | **CONV_SPEED** | `CONVEYOR:SET_SPEED:VAL` | Cài đặt tốc độ băng tải (mm/s). | `CONVEYOR:SET_SPEED:40` |
| 10 | **BTN_LED** | `BTN_LED:ID:STATE` | Điều khiển đèn LED trên nút bấm cứng. `STATE` (0/1). | `BTN_LED:0:1` |

---

### Cấu trúc phản hồi từ STM32 (Response)

| STT | Tên Phản Hồi | Cấu Trúc | Mô Tả |
| :--- | :--- | :--- | :--- |
| 1 | **ACK** | `ACK:COMMAND` | Xác nhận đã nhận lệnh thành công. |
| 2 | **DONE** | `DONE:ID` | Báo cáo đã thực hiện xong lệnh Block có ID tương ứng. |
| 3 | **STATUS** | `STATUS:RS:HOMED:ESTOP:START:STOP:CONV:SPD:PUMP:SERVO:TRAY:STEPS` | Gói tin trạng thái định kỳ, chứa toàn bộ thông tin hệ thống. (Chi tiết các trường bên dưới). | `STATUS:0:1:0:0:0:1:40:0:-9000:1:0,0,0` |

---

### Chi tiết các trường trong phản hồi STATUS

| Ký hiệu | Tên đầy đủ | Ý nghĩa dữ liệu |
| :--- | :--- | :--- |
| **RS** | Robot State | Trạng thái máy (0: IDLE, 1: MOVING, 2: HOMING, 3: ESTOP). |
| **HOMED** | Homed State | Trạng thái về gốc (0: Chưa Home, 1: Đã Home). |
| **ESTOP** | Emergency Stop | Trạng thái dừng khẩn cấp (0: Bình thường, 1: Đang nhấn). |
| **START** | Start Button | Trạng thái nút nhấn START vật lý (0/1). |
| **STOP** | Stop Button | Trạng thái nút nhấn STOP vật lý (0/1). |
| **CONV** | Conveyor Status | Trạng thái băng tải (0: Dừng, 1: Đang chạy). |
| **SPD** | Conveyor Speed | Vận tốc băng tải hiện tại (mm/s). |
| **PUMP** | Pump Status | Trạng thái bơm hút (0: Tắt, 1: Bật). |
| **SERVO** | Servo Angle | Góc quay của trục xoay cuối (độ x 100). |
| **TRAY** | Tray Sensor | Trạng thái cảm biến khay hứng (0: Không có, 1: Có khay). |
| **STEPS** | Motor Steps | Vị trí hiện tại của 3 trục tính theo bước (VD: `s1,s2,s3`). |
| 4 | **EVENT** | `EVENT:TRAY_FOUND` | Thông báo sự kiện bất thường (VD: Có khay, Mất khay). |
| 5 | **ERROR** | `ERROR:MESSAGE` | Báo lỗi hệ thống (VD: Chạm giới hạn, Lỗi Homing). |
