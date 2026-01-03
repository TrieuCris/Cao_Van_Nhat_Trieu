import time
import math
import sys
import os

# Thêm đường dẫn để import module
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from kinematics import DeltaKinematics, MotionPlannerTrapezoidal
import constants as C

def run_test():
    print("="*60)
    print("   KIỂM TRA ĐỘ CHÍNH XÁC: MÔ PHỎNG vs THỰC TẾ (SCHEDULER)")
    print("="*60)

    # 1. Khởi tạo & Warmup
    print("[1] Khởi tạo Robot Kinematics & Planner...")
    kine = DeltaKinematics()
    planner = MotionPlannerTrapezoidal(kine)
    
    print("[2] Warming up Numba functions...")
    start_warmup = time.perf_counter()
    kine.warmup()
    end_warmup = time.perf_counter()
    print(f"    -> Warmup xong trong {(end_warmup - start_warmup)*1000:.2f} ms")

    # 2. Định nghĩa kịch bản test
    # Kịch bản: Robot đang ở Wait, Vật vừa chạm Trigger Line
    # Robot cần bay từ Wait đến điểm Gắp (Intercept)
    
    P_WAIT = (0.0, -15.0, -390.0)      # Vị trí chờ tiêu chuẩn
    
    # Giả sử vật là "Chuối", trigger tại Y=-240, nhưng robot sẽ đón lõng ở Y=-50 (trong vùng gắp)
    # X=60 (lệch phải), Z=-414 (Hạ thấp gắp)
    P_PICK = (60.0, -50.0, -414.0)    
    
    ALPHA = -90.0 # Góc servo mặc định
    
    print("\n" + "-"*60)
    print(f"KỊCH BẢN TEST:")
    print(f"  Start (Wait): {P_WAIT}")
    print(f"  End   (Pick): {P_PICK}")
    print(f"  Servo Angle : {ALPHA} deg")
    
    dist_3d = math.sqrt((P_PICK[0]-P_WAIT[0])**2 + (P_PICK[1]-P_WAIT[1])**2 + (P_PICK[2]-P_WAIT[2])**2)
    print(f"  Khoảng cách : {dist_3d:.2f} mm")
    print("-"*60)

    # -------------------------------------------------------------------------
    # PHẦN 1: MÔ PHỎNG (SCHEDULER DÙNG ĐỂ TÍNH TOÁN)
    # -------------------------------------------------------------------------
    print("\n[A] CHẠY MÔ PHỎNG (Scheduler Calculation)...")
    t0 = time.perf_counter()
    
    # Gọi hàm Numba mới
    sim_duration = planner.calc_travel_time(P_WAIT, P_PICK, ALPHA)
    
    t1 = time.perf_counter()
    calc_time_ms = (t1 - t0) * 1000
    
    # Quy đổi ra số block (làm tròn lên)
    sim_blocks = math.ceil(sim_duration / 0.020)
    sim_duration_rounded = sim_blocks * 0.020
    
    print(f"    -> Thời gian tính toán (CPU): {calc_time_ms:.3f} ms (Siêu nhanh)")
    print(f"    -> Kết quả Mô phỏng (Raw)   : {sim_duration:.6f} s")
    print(f"    -> Kết quả Mô phỏng (Block) : {sim_blocks} blocks")
    print(f"    -> Thời gian 'Khóa' Robot   : {sim_duration_rounded:.4f} s (Đã làm tròn 20ms)")

    # -------------------------------------------------------------------------
    # PHẦN 2: THỰC TẾ (REAL EXECUTION PLAN GENERATION)
    # -------------------------------------------------------------------------
    print("\n[B] CHẠY THỰC TẾ (Generating Real Trajectory)...")
    
    # Cần tính góc khớp hiện tại cho Planner thực tế
    current_angles = kine.inverse_kinematics_tool(P_WAIT[0], P_WAIT[1], P_WAIT[2], ALPHA)
    
    t2 = time.perf_counter()
    
    # Gọi hàm sinh quỹ đạo chi tiết (Slicing)
    real_plan = planner.plan_cartesian_move_time_sliced(
        P_WAIT, P_PICK, ALPHA, kine, current_angles
    )
    
    t3 = time.perf_counter()
    exec_calc_time_ms = (t3 - t2) * 1000
    
    real_blocks = len(real_plan)
    real_duration = sum(b['t'] for b in real_plan)
    
    print(f"    -> Thời gian sinh quỹ đạo (CPU): {exec_calc_time_ms:.3f} ms")
    print(f"    -> Số lượng Block thực tế      : {real_blocks} blocks")
    print(f"    -> Tổng thời gian thực tế      : {real_duration:.6f} s")

    # -------------------------------------------------------------------------
    # PHẦN 3: SO SÁNH & KẾT LUẬN
    # -------------------------------------------------------------------------
    print("\n" + "="*60)
    print("   KẾT QUẢ SO SÁNH")
    print("="*60)
    print(f"{ 'Tiêu chí':<25} | {'Mô phỏng (Scheduler)':<20} | {'Thực tế (Execution)':<20} | {'Lệch':<10}")
    print("-" * 85)
    
    diff_blocks = sim_blocks - real_blocks
    diff_time = sim_duration_rounded - real_duration
    
    print(f"{ 'Số Block (20ms/blk)':<25} | {sim_blocks:<20} | {real_blocks:<20} | {diff_blocks:+d}")
    print(f"{ 'Thời gian (giây)':<25} | {sim_duration_rounded:<20.4f} | {real_duration:<20.4f} | {diff_time:+.4f}s")
    
    print("-" * 85)
    
    if diff_time >= 0:
        print("\n✅ KẾT LUẬN: AN TOÀN!")
        print(f"   Mô phỏng dự tính DƯ {diff_time*1000:.1f}ms so với thực tế.")
        print("   -> Robot sẽ đến sớm và chờ vật trôi tới.")
        print("   -> Không bị gắp trễ.")
    else:
        print("\n❌ KẾT LUẬN: NGUY HIỂM!")
        print(f"   Mô phỏng dự tính THIẾU {abs(diff_time)*1000:.1f}ms.")
        print("   -> Robot sẽ xuất phát muộn và không đuổi kịp vật.")
        print("   -> Cần tăng hệ số an toàn trong kinematics.py.")

if __name__ == "__main__":
    run_test()
