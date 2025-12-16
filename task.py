#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
task.py
选题6 - 综合任务脚本 (OOP重构版 - 含单次标定方案)
"""

import time
import math
import numpy as np
import cv2

# --- 模块导入 ---
import config  # 导入配置文件
from User_Motor import User_Motor
from cam import Camera
from detect_red_laser import detect_red_laser
from detect_border_points_Gray import detect_border_points

# ================= 辅助逻辑类 =================

class GeometricSolver:
    """几何解算器：负责将物理坐标转换为云台角度"""
    def __init__(self):
        self.dist = config.SCREEN_DISTANCE
        self.gimbal_h = config.GIMBAL_HEIGHT
        half_side = config.SQUARE_SIZE / 2.0
        
        # 屏幕物理坐标系
        self.center_x = 0
        self.center_y = config.PENCIL_HEIGHT_OFFSET + half_side
        
        # 四个物理角点 (cm)
        top_y = config.PENCIL_HEIGHT_OFFSET + config.SQUARE_SIZE
        bottom_y = config.PENCIL_HEIGHT_OFFSET
        left_x = -half_side
        right_x = half_side
        
        self.p_tl = (left_x, top_y)    
        self.p_tr = (right_x, top_y)   
        self.p_br = (right_x, bottom_y)
        self.p_bl = (left_x, bottom_y) 

    def solve_angles(self, screen_x_cm, screen_y_cm):
        """物理坐标(cm) -> 云台角度(deg)"""
        # Yaw: 正=左，屏幕X正=右 -> 取反
        yaw_rad = math.atan2(screen_x_cm, self.dist)
        yaw_deg = -math.degrees(yaw_rad) 
        
        # Pitch: 正=下，屏幕Y正=上 -> 取反(相对于水平面)
        dist_projected = math.hypot(self.dist, screen_x_cm)
        delta_h = screen_y_cm - self.gimbal_h
        pitch_rad = math.atan2(delta_h, dist_projected)
        pitch_deg = -math.degrees(pitch_rad)
        
        return pitch_deg, yaw_deg


class VisualServoController:
    """视觉伺服控制器：负责闭环控制逻辑 (Task 3.1)"""
    def __init__(self, gimbal, camera):
        self.gimbal = gimbal
        self.cam = camera
        self.kp = config.KP
        self.max_rpm = config.MAX_RPM
        self.threshold = config.CLOSE_THRESHOLD
        self.step = config.WAYPOINT_STEP

    def get_laser_pos(self):
        frame = self.cam.read_frame()
        if frame is None: return -1, -1, None
        cx, cy = detect_red_laser(frame)
        return cx, cy, frame

    def move_to_target_pixel(self, target_x, target_y, timeout=5.0):
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                self.gimbal.stop_continuous_move()
                break
            lx, ly, frame = self.get_laser_pos()
            if lx == -1:
                self.gimbal.stop_continuous_move()
                continue
            
            err_x = target_x - lx
            err_y = target_y - ly
            distance = math.hypot(err_x, err_y)
            
            if distance < self.threshold:
                return True

            yaw_rpm = -1 * err_x * self.kp
            pitch_rpm = 1 * err_y * self.kp
            yaw_rpm = max(min(yaw_rpm, self.max_rpm), -self.max_rpm)
            pitch_rpm = max(min(pitch_rpm, self.max_rpm), -self.max_rpm)
            self.gimbal.start_continuous_move(int(pitch_rpm), int(yaw_rpm))

    def follow_line(self, start_pt, end_pt):
        p1 = np.array(start_pt)
        p2 = np.array(end_pt)
        dist = np.linalg.norm(p2 - p1)
        steps = int(dist / self.step)
        if steps < 1: steps = 1
        waypoints = np.linspace(p1, p2, steps + 1)
        for pt in waypoints:
            self.move_to_target_pixel(int(pt[0]), int(pt[1]), timeout=1.5)

# ================= 核心任务控制类 =================

class GimbalTaskSystem:
    def __init__(self, gimbal_instance):
        self.gimbal = gimbal_instance
        self.geo_solver = GeometricSolver()
        self.cam = None
        self.servo = None

    def _ensure_camera(self):
        if self.cam is None:
            print("正在初始化摄像头...")
            self.cam = Camera()
            time.sleep(1.0) # 等待自动曝光稳定
        if self.servo is None:
            self.servo = VisualServoController(self.gimbal, self.cam)

    def _release_camera(self):
        if self.cam is not None:
            self.cam.release()
            self.cam = None
            self.servo = None
            print("摄像头已释放")

    def run_task_1(self):
        print("\n>>> [Task 1] 回到中心")
        tx, ty = self.geo_solver.center_x, self.geo_solver.center_y
        p_angle, y_angle = self.geo_solver.solve_angles(tx, ty)
        self.gimbal.move_angles_sync(p_angle, y_angle)
        self.gimbal.reset()

    def run_task_2(self):
        print("\n>>> [Task 2] 几何跑圈")
        solver = self.geo_solver
        path = [solver.p_tl, solver.p_tr, solver.p_br, solver.p_bl, solver.p_tl]
        
        start_x, start_y = path[0]
        p_curr, y_curr = solver.solve_angles(start_x, start_y)
        self.gimbal.move_angles_sync(p_curr, y_curr)
        time.sleep(0.5)
        
        for tx, ty in path[1:]:
            p_next, y_next = solver.solve_angles(tx, ty)
            self.gimbal.move_angles_sync(p_next - p_curr, y_next - y_curr)
            p_curr, y_curr = p_next, y_next
            time.sleep(0.5)
        self.gimbal.reset()

    def run_task_3(self):
        """任务3：A4靶纸视觉追踪"""
        print("\n>>> [Task 3_1] 闭环A4靶纸视觉追踪")
        
        try:
            self._ensure_camera()
            
            # 1. 检测靶纸
            print("正在检测 A4 靶纸...")
            target_rect = None
            for _ in range(50):
                frame = self.cam.read_frame()
                if frame is None: continue
                
                res = detect_border_points(frame, visualize=False)
                if res is not None and len(res) == 4:
                    target_rect = res
                    print(f"检测成功: {target_rect}")
                    break
                time.sleep(0.03)
                
            if target_rect is None:
                print("错误: 未检测到靶纸")
                return

            p_tl, p_tr, p_br, p_bl = target_rect

            # 2. 前往起点
            print("前往起点 (左上角)...")
            self.servo.move_to_target_pixel(p_tl[0], p_tl[1], timeout=8.0)
            time.sleep(0.5)

            # 3. 跑圈
            segments = [
                (p_tl, p_tr), (p_tr, p_br), (p_br, p_bl), (p_bl, p_tl)
            ]
            
            t_start = time.time()
            for i, (start_pt, end_pt) in enumerate(segments):
                print(f"执行边 {i+1}...")
                self.servo.follow_line(start_pt, end_pt)
                
            print(f"跑圈完成，耗时: {time.time() - t_start:.2f}s")

        except KeyboardInterrupt:
            print("任务中断")
        finally:
            self.gimbal.stop_continuous_move()
            # 注意：如果要在GUI中连续使用，这里可以不release camera
            # 但为了脚本独立运行的安全性，默认最后释放
            self._release_camera()

    # --- Task 3.2: 单次标定 + 开环解算 ---
    def run_task_3_2(self):
        """
        Task 3 新方案：单次标定 + 几何开环
        1. 云台复位 (0,0)
        2. 拍照：获取红色激光点(Origin) 和 A4纸四角
        3. 计算：像素->厘米 映射关系
        4. 生成路径：计算各点相对于 Origin 的角度偏移
        5. 执行：快速移动
        """
        print("\n>>> [Task 3.2] 单次标定跑圈 (Calibrated Open-Loop)")
        
        # A4纸检测尺寸 (内缩 0.9cm 后)
        # 真实A4: 21.0 x 29.7 cm
        # 内缩后: 19.2 x 27.9 cm
        REAL_WIDTH_CM = 19.2
        REAL_HEIGHT_CM = 27.9
        
        try:
            self._ensure_camera()
            
            # 1. 确保云台处于零位
            print("1. 正在复位云台至机械零点...")
            self.gimbal.reset()
            time.sleep(2.0) # 等待完全静止
            
            # 2. 采集图像并识别
            print("2. 正在采集图像并解算坐标...")
            laser_px = None
            rect_pts = None
            
            # 连拍几张取最后一张，确保清晰
            frame = None
            for _ in range(10):
                frame = self.cam.read_frame()
                time.sleep(0.05)
                
            if frame is None:
                print("错误：无法读取图像")
                return

            # 检测激光点 (作为 (0,0) 参考基准)
            lx, ly = detect_red_laser(frame)
            if lx == -1:
                print("错误：未检测到红色激光点 (请确保激光开启且在视野内)")
                return
            laser_px = (lx, ly)
            print(f"   激光原点像素: {laser_px}")

            # 检测 A4 纸
            rect_pts = detect_border_points(frame, visualize=False)
            if rect_pts is None or len(rect_pts) != 4:
                print("错误：未检测到完整的 A4 靶纸")
                return
            print(f"   A4顶点像素: {rect_pts}")
            
            # 3. 建立映射关系 (Pixel -> CM)
            # rect_pts 顺序: [TL, TR, BR, BL] (顺时针)
            p_tl, p_tr, p_br, p_bl = rect_pts
            
            # 计算像素宽和高 (取平均值以减小误差)
            w1 = np.linalg.norm(np.array(p_tl) - np.array(p_tr)) # 上边
            w2 = np.linalg.norm(np.array(p_bl) - np.array(p_br)) # 下边
            h1 = np.linalg.norm(np.array(p_tl) - np.array(p_bl)) # 左边
            h2 = np.linalg.norm(np.array(p_tr) - np.array(p_br)) # 右边
            
            avg_w_px = (w1 + w2) / 2.0
            avg_h_px = (h1 + h2) / 2.0
            
            ratio_x = REAL_WIDTH_CM / avg_w_px   # cm per pixel (width)
            ratio_y = REAL_HEIGHT_CM / avg_h_px  # cm per pixel (height)
            
            # 使用平均比例 (假设像素是正方形)
            cm_per_pixel = (ratio_x + ratio_y) / 2.0
            print(f"   标定结果: 1 Pixel ≈ {cm_per_pixel:.4f} cm")
            
            # 4. 路径规划与角度解算
            # 目标路径: Current(Laser) -> TL -> TR -> BR -> BL -> TL
            target_pixels = [p_tl, p_tr, p_br, p_bl, p_tl]
            
            # 记录当前云台的绝对角度 (初始为 0,0)
            curr_abs_pitch = 0.0
            curr_abs_yaw = 0.0
            
            print("3. 开始执行路径...")
            
            for i, (tx, ty) in enumerate(target_pixels):
                # A. 计算目标点相对于激光原点(Initial Laser)的像素差
                dx_px = tx - lx
                dy_px = ty - ly
                
                # B. 转换为物理距离 (cm)
                # 注意坐标系方向：
                # 图像: X右正, Y下正
                # 物理: 假设激光点正前方为原点
                dx_cm = dx_px * cm_per_pixel
                dy_cm = dy_px * cm_per_pixel
                
                # C. 解算目标绝对角度
                # Yaw: 正=左。图像X向右(dx>0) -> 需要向右转 -> Angle < 0
                target_yaw = -math.degrees(math.atan2(dx_cm, config.SCREEN_DISTANCE))
                
                # Pitch: 正=下。图像Y向下(dy>0) -> 需要向下转 -> Angle > 0
                # 使用投影距离校正Pitch (球面坐标系)
                dist_proj = math.hypot(config.SCREEN_DISTANCE, dx_cm)
                target_pitch = math.degrees(math.atan2(dy_cm, dist_proj))
                
                # D. 计算增量并移动
                delta_pitch = target_pitch - curr_abs_pitch
                delta_yaw = target_yaw - curr_abs_yaw
                
                print(f"   Step {i+1}: 移动增量 P={delta_pitch:.2f}, Y={delta_yaw:.2f}")
                
                # 执行移动 (阻塞等待到位)
                self.gimbal.move_angles_sync(delta_pitch, delta_yaw)
                
                # E. 更新当前状态
                curr_abs_pitch = target_pitch
                curr_abs_yaw = target_yaw
                
                # 稍微停顿，展示效果
                time.sleep(0.2)
                
            print("任务完成，复位中...")
            self.gimbal.reset()

        except Exception as e:
            print(f"执行出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self._release_camera()


# ================= 命令行入口 =================

if __name__ == "__main__":
    print(f"=== 选题6 任务程序 ===")
    
    try:
        with User_Motor() as gimbal_dev:
            app = GimbalTaskSystem(gimbal_dev)
            
            while True:
                print("\n-----------------------------")
                print("1. [Task 1] 几何回中")
                print("2. [Task 2] 几何跑圈 (盲跑)")
                print("3. [Task 3.1] 视觉伺服 (实时闭环 - 旧)")
                print("4. [Task 3.2] 单次标定跑圈 (推荐 - 新)")
                print("q. 退出")
                
                cmd = input("请输入指令: ")
                
                if cmd == '1':
                    app.run_task_1()
                elif cmd == '2':
                    app.run_task_2()
                elif cmd == '3':
                    # 这里调用你原来的闭环逻辑，如果你保留了的话
                    print("请使用 run_task_3_2") 
                elif cmd == '4':
                    app.run_task_3_2()
                elif cmd == 'q':
                    break
    except Exception as e:
        print(f"Main Error: {e}")