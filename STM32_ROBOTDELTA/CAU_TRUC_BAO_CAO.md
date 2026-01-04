# CẤU TRÚC BÁO CÁO PHẦN MỀM FIRMWARE

## 📋 TỔNG QUAN LUỒNG XỬ LÝ

```
PC (Python) 
    ↓ USB CDC
[PHẦN 1] Nhận & Phân tích lệnh (Command Parser)
    ├─→ HOME → [PHẦN 2] Homing State Machine → Bật TIM3 ISR (Homing)
    ├─→ ADD_BLOCK → Queue → [PHẦN 3] Motion Control → Bật TIM3 ISR (Moving)
    ├─→ CONVEYOR → Điều khiển PWM băng tải (mô tả ngay)
    ├─→ FLUSH_BUFFER → Dừng & Xóa queue (mô tả ngay)
    └─→ STATUS, PING, ... → Xử lý đơn giản
```

---

## 📚 CẤU TRÚC CÁC PHẦN

### **PHẦN 0: CHƯƠNG TRÌNH CHÍNH (main.c)** ✅ Đã có
**File:** `LUUDO_CHUONGTRINH_CHINH.md`

**Nội dung:**
- Kiến trúc Non-Blocking Loop
- Lưu đồ tổng quan: Khởi tạo → Vòng lặp chính
- Polling các module: USB, Queue, Sensors, Watchdog
- **Vai trò:** Điều phối tổng thể, không đi sâu logic từng module

---

### **PHẦN 1: NHẬN VÀ XỬ LÝ LỆNH** (Communication & Command Dispatching)
**File tạo mới:** `LUUDO_NHAN_VA_XU_LY_LENH.md`

#### **1.1. USB CDC - Giao tiếp Serial**
- Circular Buffer (512 bytes)
- `cdc_handler_get_command()`: Nhận lệnh từ PC
- `cdc_handler_send_response()`: Gửi phản hồi về PC
- Non-blocking polling trong main loop

**Mô tả:** Text + sơ đồ Circular Buffer (không cần lưu đồ phức tạp)

---

#### **1.2. Command Parser - Phân tích cú pháp**
**Lưu đồ:** Flowchart `parse_command()`

```
Nhận chuỗi lệnh
    ↓
Tách command_name (trước dấu ':')
    ↓
Switch-case routing:
    ├─ PING → Trả "PONG" + Enable motors
    ├─ HOME → Gọi robot_start_homing() → [ĐI ĐẾN PHẦN 2]
    ├─ ADD_BLOCK → Parse JSON → Thêm vào Queue → [ĐI ĐẾN 1.3]
    ├─ CONVEYOR:START:FWD → conveyor_start(true) [MÔ TẢ NGAY]
    ├─ CONVEYOR:STOP → conveyor_stop() [MÔ TẢ NGAY]
    ├─ FLUSH_BUFFER → robot_abort() + queue_flush() [MÔ TẢ NGAY]
    ├─ STATUS → send_status_report()
    └─ Lệnh không hợp lệ → "ERROR:Unknown command"
```

**Chi tiết các lệnh đơn giản (mô tả luôn, không cần lưu đồ riêng):**

- **CONVEYOR (Băng tải):**
  ```c
  conveyor_start(true);  // FWD: Set DIR=LOW, Start PWM TIM2
  conveyor_start(false); // REV: Set DIR=HIGH, Start PWM TIM2
  conveyor_stop();       // Stop PWM
  ```
  - Công thức PWM: `Duty% = (Speed_mm/s / 500) × 100%`
  - Timer: TIM2_CH1 (PA0), 10 kHz PWM

- **FLUSH_BUFFER (Dừng khẩn cấp):**
  ```c
  robot_abort();    // Dừng TIM3 ISR, reset state → IDLE
  queue_flush();    // Xóa toàn bộ hàng đợi
  ```

- **STATUS (Truy vấn trạng thái):**
  - Gửi: Run state, Homed, E-Stop, Buttons, Conveyor, Pump, Servo angle, Motor steps
  - Format: `STATUS:1:1:0:0:0:1:150:1:13500:0:0,0,0`

---

#### **1.3. Command Queue - Hàng đợi lệnh**
**Lưu đồ:** Flowchart `queue_process()` + FIFO structure

```
ADD_BLOCK:id:{"t":500, "s":[100,200,300], "a":13500, "b":1}
    ↓
Parse JSON → MotionBlock
    ↓
queue_add_block(&block) → FIFO Queue (32 slots)
    ↓
Main loop gọi queue_process() mỗi vòng
    ↓
Kiểm tra:
    ├─ Robot IDLE? (không phải MOVING/HOMING)
    ├─ Queue không rỗng?
    └─ Đã homing?
    ↓ (Cả 3 điều kiện đúng)
queue_pop() → Lấy block đầu tiên
    ↓
robot_execute_motion_block(&block) → [ĐI ĐẾN PHẦN 3]
```

**Cấu trúc FIFO:**
```c
typedef struct {
    uint32_t id;              // ID lệnh (để gửi DONE:id)
    int32_t motor_steps[3];   // Số bước 3 motor
    uint16_t servo_pulse;     // Xung PWM servo (500-2500)
    bool pump_state;          // Bật/tắt bơm hút
    uint32_t duration;        // Thời gian di chuyển (ms)
} MotionBlock;

MotionBlock queue[32];
uint8_t queue_head = 0;
uint8_t queue_tail = 0;
uint8_t queue_count = 0;
```

**Xử lý DONE:**
```
ISR TIM3 (khi hoàn thành di chuyển)
    ↓
Đặt cờ done_flag = true
Lưu done_id = block.id
    ↓
Main loop: queue_handle_done_messages()
    ↓
Gửi "DONE:id" lên PC
Xóa cờ done_flag
```

---

### **PHẦN 2: HOMING - TÌM ĐIỂM GỐC** (State Machine - Homing) ✅ Đã có
**File:** `LUUDO_HOMING_STATE_MACHINE.md`

**Nội dung:**
- **Khi nào bật TIM3?** `robot_start_homing()` → `HAL_TIM_Base_Start_IT(&htim3)`
- State Machine: RAISING → BACKOFF → DONE
- DDS algorithm trong ISR
- Debounce Limit Switch
- Safety watchdog

**Trạng thái robot:** `ROBOT_STATE_HOMING`

---

### **PHẦN 3: PHÁT XUNG ĐỘNG CƠ - ĐIỀU KHIỂN CHUYỂN ĐỘNG** (State Machine - Moving)
**File tạo mới:** `LUUDO_PHAT_XUNG_DONG_CO.md`

#### **3.1. Luồng xử lý từ Queue đến bật TIM3**

**Chuỗi hàm gọi:**
```
Main Loop: queue_process()
    ↓ (Kiểm tra: IDLE? Queue không rỗng? current_block_id==0? Đã homing?)
robot_execute_block()
    ↓ (Set state = MOVING, reset debounce)
robot_apply_block()
    ↓ (Tính DDS, set DIR, servo, pump)
HAL_TIM_Base_Start_IT(&htim3) ← BẬT TIM3 Ở ĐÂY
```

**Chi tiết từng hàm:**

**1. queue_process() - Kiểm tra điều kiện:**
```c
void queue_process(void) {
    // ✅ 3 điều kiện cần thiết:
    if (robot_get_state() == ROBOT_STATE_IDLE &&  // (1) Robot rảnh
        count > 0 &&                                // (2) Queue không rỗng
        current_block_id == 0) {                    // (3) Không có lệnh đang chạy
        
        // Lấy block đầu tiên từ FIFO
        MotionBlock* next_block = &queue_buffer[tail];
        current_block_id = next_block->id;  // Lưu ID để báo DONE sau
        tail = (tail + 1) % QUEUE_SIZE;
        count--;
        
        // Gửi cho robot thực thi
        robot_execute_block(next_block);
    }
}
```

**2. robot_execute_block() - Chuẩn bị trạng thái:**
```c
void robot_execute_block(const MotionBlock* block) {
    if (current_robot_state != ROBOT_STATE_IDLE) return;
    
    // ✅ Chuyển sang trạng thái MOVING
    current_robot_state = ROBOT_STATE_MOVING;
    
    // Reset debounce counters (phát hiện limit switch trong lúc chạy)
    moving_ls_debounce[0] = 0;
    moving_ls_debounce[1] = 0;
    moving_ls_debounce[2] = 0;
    
    // Tính toán và cấu hình phần cứng
    robot_apply_block(block);
    
    // Kiểm tra có motor nào active không
    bool any_active = false;
    for(int i=0; i<3; i++) {
        if(motor_states[i].active) any_active = true;
    }
    
    // ✅ BẬT TIM3 nếu có ít nhất 1 motor cần chạy
    if (any_active) {
        HAL_TIM_Base_Start_IT(&htim3);  // ← ĐIỂM BẬT TIM3
    } else {
        // Edge case: Block không có motor nào chạy (steps=0)
        current_robot_state = ROBOT_STATE_IDLE;
        queue_finish_current_block();  // Gửi DONE ngay
    }
}
```

**3. robot_apply_block() - Tính toán DDS:**
```c
static void robot_apply_block(const MotionBlock* block) {
    // 1. Set Servo PWM
    if (!servo_pwm_started) {
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
        servo_pwm_started = true;
    }
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, block->servo_pulse);
    last_servo_activity_tick = HAL_GetTick();
    
    // 2. Set Pump (bơm hút)
    HAL_GPIO_WritePin(PUMP_GPIO_Port, PUMP_Pin, 
                      block->pump_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // 3. Tính DDS speed_addend cho mỗi motor
    uint32_t duration_ms = block->duration;
    
    for (int i = 0; i < 3; i++) {
        uint32_t steps = abs(block->motor_steps[i]);
        
        if (steps == 0) {
            motor_states[i].active = false;
            continue;  // Bỏ qua motor không chạy
        }
        
        // ✅ Công thức DDS:
        // speed_addend = (steps × 1000 × DDS_CONST) / duration_ms
        uint64_t numerator = (uint64_t)steps * 1000ULL * (uint64_t)DDS_CONST;
        uint32_t speed_addend = (uint32_t)((numerator + duration_ms/2) / duration_ms);
        
        motor_states[i].active = true;
        motor_states[i].steps_to_go = steps;
        motor_states[i].speed_addend = speed_addend;
        motor_states[i].accumulator = 0x80000000UL;  // Bắt đầu từ 50%
        motor_states[i].pul_state = false;
        
        // 4. Set DIR pins (chiều quay)
        GPIO_TypeDef* dir_port;
        uint16_t dir_pin;
        if (i == 0) { dir_port = M1_DIR_GPIO_Port; dir_pin = M1_DIR_Pin; }
        else if (i == 1) { dir_port = M2_DIR_GPIO_Port; dir_pin = M2_DIR_Pin; }
        else { dir_port = M3_DIR_GPIO_Port; dir_pin = M3_DIR_Pin; }
        
        HAL_GPIO_WritePin(dir_port, dir_pin, 
                          (block->motor_steps[i] > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    
    // 5. Enable motors (ENA = LOW)
    HAL_GPIO_WritePin(M1_ENA_GPIO_Port, M1_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M2_ENA_GPIO_Port, M2_ENA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M3_ENA_GPIO_Port, M3_ENA_Pin, GPIO_PIN_RESET);
    
    // 6. Set LED = RUNNING
    status_led_set_status(LED_STATUS_RUNNING);
}
```

**Điểm quan trọng:**
- **Queue chỉ kiểm tra điều kiện**, KHÔNG bật TIM3
- **robot_execute_block()** mới bật TIM3 (sau khi cấu hình xong)
- **Servo + Pump** được set trong `robot_apply_block()` TRƯỚC khi bật TIM3
- **DIR pins** được set TRƯỚC khi bật TIM3 (cần setup time 5µs)

---

#### **3.2. ISR TIM3 - Phát xung Step**
**Lưu đồ:** Flowchart ISR logic (MOVING state)

```
ISR TIM3 (80 kHz - Mỗi 12.5µs)
    ↓
if (current_robot_state == MOVING)
    ↓
Vòng lặp 3 motors:
    ├─ Motor 0
    ├─ Motor 1  
    └─ Motor 2
        ↓
    if (motor_states[i].active && steps_to_go > 0)
        ↓
    DDS Algorithm:
        accumulator += speed_addend
        if (accumulator overflow) → Tràn 32-bit
            ├─ Tạo xung HIGH: BSRR = pul_pin
            ├─ steps_to_go--
            └─ absolute_motor_steps[i] += direction
        else
            └─ Tạo xung LOW: BSRR = pul_pin << 16
        ↓
    if (steps_to_go == 0)
        ├─ motor_states[i].active = false
        └─ Tắt xung (LOW)
    ↓
Kiểm tra: CẢ 3 motor done?
    ↓ (Có)
Dừng TIM3 ISR
Đặt cờ done_flag = true
done_id = current_block_id
current_robot_state = IDLE
    ↓
Main loop: queue_handle_done_messages()
    ↓
Gửi "DONE:id" lên PC
```

---

#### **3.3. DDS Algorithm (Digital Differential Analyzer)**

**Nguyên lý:**
```
Mục tiêu: Tạo xung với tần số f (Hz) từ timer T (80 kHz)

Công thức:
    DDS_CONST = 2^32 / T = 4294967296 / 80000 = 53688
    speed_addend = f × DDS_CONST
    
Mỗi tick ISR:
    accumulator += speed_addend
    if (accumulator tràn 32-bit) → Tạo xung
    
Tần số thực tế:
    f_actual = speed_addend / DDS_CONST (Hz)
```

**Ví dụ:**
```c
// Muốn 1000 Hz (1000 steps/s)
speed_addend = 1000 × 53688 = 53,688,000

// Trong ISR (80,000 lần/giây):
accumulator += 53,688,000
// Tràn sau ~80 lần cộng → 1000 xung/giây ✓
```

**Ưu điểm:**
- ✅ Không cần chia (chỉ cộng) → Cực nhanh trong ISR
- ✅ Độ chính xác cao (32-bit resolution)
- ✅ Dễ đồng bộ 3 motor (mỗi motor 1 accumulator riêng)

---

#### **3.4. Synchronization - Đồng bộ 3 motor**

**Vấn đề:** 3 motor cần hoàn thành cùng lúc dù số bước khác nhau

**Giải pháp:** Điều chỉnh tần số mỗi motor

```
PC tính toán:
    max_steps = max(|s1|, |s2|, |s3|)
    duration_ms = Thời gian di chuyển (ms)
    
Mỗi motor:
    freq[i] = |steps[i]| / (duration_ms / 1000)
    
STM32:
    speed_addend[i] = freq[i] × DDS_CONST
    
Kết quả:
    Motor nhiều bước → Chạy nhanh
    Motor ít bước → Chạy chậm
    → Cùng hoàn thành sau duration_ms
```

**Ví dụ:**
```
Block: {"t":500, "s":[1000, 500, 300]}
Duration: 500ms

Motor 0: 1000 steps → 2000 Hz
Motor 1:  500 steps → 1000 Hz
Motor 2:  300 steps →  600 Hz

Sau 500ms:
    M0: 2000 × 0.5 = 1000 ✓
    M1: 1000 × 0.5 =  500 ✓
    M2:  600 × 0.5 =  300 ✓
```

---

#### **3.5. Servo & Pump Timing**

**Servo (SG90):**
- **PWM:** TIM4_CH4 (PB9), 50 Hz, 20ms period
- **Pulse:** 500-2500µs (0°-270°)
- **Timing:** Đổi góc NGAY khi bắt đầu di chuyển (cùng lúc với Step)
- **Auto-Detach:** Ngắt PWM sau 2s không hoạt động (giảm nhiệt)

**Bơm hút (Relay/MOSFET):**
- **Pin:** PA9 (GPIO Output)
- **Logic:** HIGH = Bật, LOW = Tắt
- **Timing:** Đổi trạng thái NGAY khi bắt đầu di chuyển

**Đồng bộ:**
```
robot_execute_motion_block():
    Set Servo PWM → Bắt đầu quay ngay
    Set Pump → Bật/tắt ngay
    Bật TIM3 ISR → Bắt đầu Step motors
    
→ Servo + Pump + Motors chạy song song
→ PC tính toán timing sao cho:
    - Servo quay xong trước khi robot đến đích
    - Pump bật/tắt đúng lúc
```

---

## 📊 BẢNG TÓM TẮT CÁC PHẦN

| **Phần** | **File** | **Nội dung chính** | **Bật TIM3?** | **Trạng thái Robot** |
|----------|----------|-------------------|---------------|---------------------|
| 0. Main | `LUUDO_CHUONGTRINH_CHINH.md` | Vòng lặp chính, polling | Không | - |
| 1. Giao tiếp | `LUUDO_NHAN_VA_XU_LY_LENH.md` | USB CDC, Parser, Queue | Không | IDLE |
| 2. Homing | `LUUDO_HOMING_STATE_MACHINE.md` | RAISING → BACKOFF → DONE | ✅ `robot_start_homing()` | HOMING |
| 3. Motion | `LUUDO_PHAT_XUNG_DONG_CO.md` | DDS, ISR, Sync 3 motors | ✅ `robot_execute_motion_block()` | MOVING |

---

## 🔄 LUỒNG XỬ LÝ HOÀN CHỈNH

### **Ví dụ 1: Lệnh HOME**
```
1. PC gửi: "HOME"
2. USB CDC → cdc_handler_get_command() → "HOME"
3. parse_command() → Switch "HOME"
4. robot_start_homing()
    ├─ Set state = HOMING
    ├─ Bật TIM3 ISR ← [Điểm này bật Timer]
    └─ Gửi "ACK:HOME started"
5. ISR TIM3 chạy State Machine:
    ├─ RAISING (nâng lên)
    ├─ BACKOFF (lùi xuống)
    └─ DONE → Tắt TIM3
6. Main loop: Gửi "HOME_DONE"
```

### **Ví dụ 2: Lệnh ADD_BLOCK**
```
1. PC gửi: "ADD_BLOCK:1:{"t":500,"s":[100,200,300],"a":13500,"b":1}"
2. USB CDC → cdc_handler_get_command()
3. parse_command() → handle_add_block()
    ├─ Parse JSON → MotionBlock
    └─ queue_add_block()
4. Main loop: queue_process()
    ├─ Kiểm tra: IDLE? Queue không rỗng? Homed?
    ├─ queue_pop() → Lấy block
    └─ robot_execute_motion_block()
        ├─ Set state = MOVING
        ├─ Tính DDS speed_addend
        ├─ Set Servo + Pump
        ├─ Bật TIM3 ISR ← [Điểm này bật Timer]
        └─ Gửi ACK (optional)
5. ISR TIM3 chạy DDS:
    ├─ Phát xung Step (80 kHz)
    ├─ Đếm steps_to_go
    └─ Khi done → Tắt TIM3, đặt cờ
6. Main loop: queue_handle_done_messages()
    └─ Gửi "DONE:1"
```

---

## 🎯 KẾ HOẠCH TẠO FILE

### ✅ Đã có:
- [x] `LUUDO_CHUONGTRINH_CHINH.md` (Phần 0)
- [x] `LUUDO_HOMING_STATE_MACHINE.md` (Phần 2)

### 📝 Cần tạo:
- [ ] `LUUDO_NHAN_VA_XU_LY_LENH.md` (Phần 1)
  - 1 lưu đồ: Command Parser routing
  - 1 lưu đồ: Command Queue FIFO
  - Text: USB CDC, Conveyor, Flush
  
- [ ] `LUUDO_PHAT_XUNG_DONG_CO.md` (Phần 3)
  - 1 lưu đồ: ISR TIM3 (MOVING state)
  - 1 sơ đồ: DDS Algorithm
  - Text: Servo timing, Pump control, Synchronization

---

## 💡 LƯU Ý KHI VIẾT BÁO CÁO

1. **Phần 1 (Giao tiếp):** Nhấn mạnh kiến trúc phân tầng
   - USB CDC (Vật lý)
   - Parser (Logic)
   - Queue (Điều phối)

2. **Phần 2 (Homing):** Nhấn mạnh Safety & State Machine
   - Debounce 375µs
   - Watchdog timeout
   - Error handling

3. **Phần 3 (Motion):** Nhấn mạnh Real-time & Synchronization
   - DDS algorithm (không chia)
   - 3-motor sync
   - ISR timing (12.5µs)

4. **Liên kết giữa các phần:**
   - Phần 1 routing → Phần 2/3
   - Phần 2/3 đều dùng TIM3 ISR
   - Phần 2/3 đều dùng DDS algorithm
