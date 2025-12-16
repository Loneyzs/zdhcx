#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
task.py
选题6 - 综合任务脚本 (OOP重构版)
集成功能：
1. 几何解算 (任务1 & 2)
2. 视觉伺服闭环控制 (任务3 - A4靶纸追踪)
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
        
        # 屏幕物理坐标系：中心(0, PENCIL_HEIGHT_OFFSET + half_side)
        self.center_x = 0
        self.center_y = config.PENCIL_HEIGHT_OFFSET + half_side
        
        # 四个物理角点 (cm)
        top_y = config.PENCIL_HEIGHT_OFFSET + config.SQUARE_SIZE
        bottom_y = config.PENCIL_HEIGHT_OFFSET
        left_x = -half_side
        right_x = half_side
        
        self.p_tl = (left_x, top_y)    # Top-Left
        self.p_tr = (right_x, top_y)   # Top-Right
        self.p_br = (right_x, bottom_y)# Bottom-Right
        self.p_bl = (left_x, bottom_y) # Bottom-Left

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
    """视觉伺服控制器：负责闭环控制逻辑"""
    def __init__(self, gimbal, camera):
        self.gimbal = gimbal
        self.cam = camera
        # 载入配置参数
        self.kp = config.KP
        self.max_rpm = config.MAX_RPM
        self.threshold = config.CLOSE_THRESHOLD
        self.step = config.WAYPOINT_STEP

    def get_laser_pos(self):
        """获取当前激光点坐标"""
        frame = self.cam.read_frame()
        if frame is None:
            return -1, -1, None
        
        cx, cy = detect_red_laser(frame)
        return cx, cy, frame

    def move_to_target_pixel(self, target_x, target_y, timeout=5.0):
        """
        [核心] P控制器：驱动云台使激光点到达图像上的像素坐标
        """
        start_time = time.time()
        
        while True:
            # 1. 超时退出
            if time.time() - start_time > timeout:
                self.gimbal.stop_continuous_move()
                break

            # 2. 获取反馈
            lx, ly, frame = self.get_laser_pos()
            
            # 3. 安全保护：如果丢失激光点，暂停运动
            if lx == -1:
                self.gimbal.stop_continuous_move()
                continue
            
            # 4. 计算误差
            err_x = target_x - lx
            err_y = target_y - ly
            distance = math.hypot(err_x, err_y)
            
            # 5. 判定到达
            if distance < self.threshold:
                return True

            # 6. 计算速度 (P控制)
            # Yaw轴: 图像目标在右(Error>0) -> 需要向右转 -> Yaw负方向
            yaw_rpm = -1 * err_x * self.kp
            # Pitch轴: 图像目标在下(Error>0) -> 需要向下转 -> Pitch正方向
            pitch_rpm = 1 * err_y * self.kp
            
            # 7. 速度限幅
            yaw_rpm = max(min(yaw_rpm, self.max_rpm), -self.max_rpm)
            pitch_rpm = max(min(pitch_rpm, self.max_rpm), -self.max_rpm)
            
            # 8. 执行
            self.gimbal.start_continuous_move(int(pitch_rpm), int(yaw_rpm))

    def follow_line(self, start_pt, end_pt):
        """
        [路径规划] 在两点之间进行插值，生成直线路径
        """
        p1 = np.array(start_pt)
        p2 = np.array(end_pt)
        
        dist = np.linalg.norm(p2 - p1)
        steps = int(dist / self.step)
        if steps < 1: steps = 1
        
        waypoints = np.linspace(p1, p2, steps + 1)
        
        for pt in waypoints:
            tx, ty = int(pt[0]), int(pt[1])
            self.move_to_target_pixel(tx, ty, timeout=1.5)

# ================= 核心任务控制类 =================

class GimbalTaskSystem:
    """
    任务控制系统主类
    供 GUI 或 命令行 调用
    """
    def __init__(self, gimbal_instance):
        self.gimbal = gimbal_instance
        self.geo_solver = GeometricSolver()
        self.cam = None
        self.servo = None

    def _ensure_camera(self):
        """确保摄像头已开启"""
        if self.cam is None:
            print("正在初始化摄像头...")
            self.cam = Camera()
            # 预读几帧确保稳定
            for _ in range(5):
                self.cam.read_frame()
                time.sleep(0.1)
        
        if self.servo is None:
            self.servo = VisualServoController(self.gimbal, self.cam)

    def _release_camera(self):
        """释放摄像头资源"""
        if self.cam is not None:
            self.cam.release()
            self.cam = None
            self.servo = None
            print("摄像头已释放")

    # --- 任务接口 ---

    def run_task_1(self):
        """任务1：几何回中"""
        print("\n>>> [Task 1] 回到中心")
        tx, ty = self.geo_solver.center_x, self.geo_solver.center_y
        p_angle, y_angle = self.geo_solver.solve_angles(tx, ty)
        
        self.gimbal.move_angles_sync(p_angle, y_angle)
        time.sleep(0.5)
        print("任务1 完成")
        self.gimbal.reset()

    def run_task_2(self):
        """任务2：几何跑圈 (盲跑)"""
        print("\n>>> [Task 2] 顺时针跑圈 (几何盲跑)")
        
        solver = self.geo_solver
        path_points = [solver.p_tl, solver.p_tr, solver.p_br, solver.p_bl, solver.p_tl]
        
        # 1. 前往起点
        start_x, start_y = path_points[0]
        p_start_abs, y_start_abs = solver.solve_angles(start_x, start_y)
        self.gimbal.move_angles_sync(p_start_abs, y_start_abs)
        
        p_current = p_start_abs
        y_current = y_start_abs
        time.sleep(0.5)
        
        # 2. 跑圈 (增量模式)
        for i, (tx, ty) in enumerate(path_points[1:]):
            p_target_abs, y_target_abs = solver.solve_angles(tx, ty)
            
            p_delta = p_target_abs - p_current
            y_delta = y_target_abs - y_current
            
            self.gimbal.move_angles_sync(p_delta, y_delta)
            
            p_current = p_target_abs
            y_current = y_target_abs
            time.sleep(0.5)
        
        print("任务2 完成")
        self.gimbal.reset()

    def run_task_3(self):
        """任务3：A4靶纸视觉追踪"""
        print("\n>>> [Task 3] A4靶纸视觉追踪")
        
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

    def stop_all(self):
        """紧急停止接口"""
        self.gimbal.stop_all()
        self.gimbal.stop_continuous_move()
        print("已发送停止指令")

# ================= 命令行入口 =================

if __name__ == "__main__":
    print(f"=== 选题6 任务程序 (类封装版) ===")
    print(f"加载配置: KP={config.KP}, MAX_RPM={config.MAX_RPM}")

    try:
        # User_Motor 作为一个上下文管理器，控制底层串口生命周期
        with User_Motor() as gimbal_dev:
            
            # 实例化任务系统
            app = GimbalTaskSystem(gimbal_dev)
            
            # 可选：启动时复位
            # gimbal_dev.reset()
            # time.sleep(2)
            
            while True:
                print("\n-----------------------------")
                print("1. 任务一：几何回中")
                print("2. 任务二：几何跑圈")
                print("3. 任务三：视觉伺服跑圈")
                print("q. 退出")
                
                cmd = input("请输入指令: ")
                
                if cmd == '1':
                    app.run_task_1()
                elif cmd == '2':
                    app.run_task_2()
                elif cmd == '3':
                    app.run_task_3()
                elif cmd == 'q':
                    break
                else:
                    print("无效指令")
                    
    except Exception as e:
        print(f"系统异常: {e}")
        import traceback
        traceback.print_exc()