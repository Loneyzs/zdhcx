#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task_Basic.py
选题6 - 基础要求任务脚本 (点对点简化版)
逻辑：PITCH正=下，YAW正=左
"""

import time
import math
from User_Motor import User_Motor
from config import (
    SCREEN_DISTANCE, GIMBAL_HEIGHT, PENCIL_HEIGHT_OFFSET, 
    SQUARE_SIZE
)

class GeometricSolver:
    """几何解算器：将屏幕上的(x, y)坐标转换为云台(pitch, yaw)角度"""
    def __init__(self):
        self.dist = SCREEN_DISTANCE
        self.gimbal_h = GIMBAL_HEIGHT
        
        half_side = SQUARE_SIZE / 2.0
        
        # --- 关键坐标点 (cm) ---
        # 坐标系：X=0为中心，右+左-；Y=0为地面，上+
        
        # 1. 中心点
        self.center_x = 0
        self.center_y = PENCIL_HEIGHT_OFFSET + half_side
        
        # 2. 四个角点 (用于顺时针移动)
        # 顺序：左上 -> 右上 -> 右下 -> 左下 -> (回到左上)
        top_y = PENCIL_HEIGHT_OFFSET + SQUARE_SIZE
        bottom_y = PENCIL_HEIGHT_OFFSET
        left_x = -half_side
        right_x = half_side
        
        self.p_tl = (left_x, top_y)    # Top-Left
        self.p_tr = (right_x, top_y)   # Top-Right
        self.p_br = (right_x, bottom_y)# Bottom-Right
        self.p_bl = (left_x, bottom_y) # Bottom-Left

    def solve_angles(self, screen_x_cm, screen_y_cm):
        """输入屏幕物理坐标，返回电机角度(取反适配方向)"""
        # Yaw计算 (Yaw正=左，屏幕右X为正 -> 需要取反)
        yaw_rad = math.atan2(screen_x_cm, self.dist)
        yaw_deg = -math.degrees(yaw_rad) 
        
        # Pitch计算 (Pitch正=下，屏幕上Y为正 -> 需要取反)
        dist_projected = math.hypot(self.dist, screen_x_cm)
        delta_h = screen_y_cm - self.gimbal_h # 目标高度 - 云台高度
        pitch_rad = math.atan2(delta_h, dist_projected)
        pitch_deg = -math.degrees(pitch_rad)
        
        return pitch_deg, yaw_deg

# ================= 任务函数定义 =================

def task_1_return_to_center(gimbal, solver):
    """任务1：复位功能 - 回到屏幕中心"""
    print("\n>>> 执行任务1：回到原点 (中心)")
    
    # 1. 获取中心坐标
    tx, ty = solver.center_x, solver.center_y
    
    # 2. 解算角度
    p_angle, y_angle = solver.solve_angles(tx, ty)
    print(f"目标中心: ({tx}, {ty}) -> 角度: P={p_angle:.2f}, Y={y_angle:.2f}")
    
    # 3. 移动并延时
    gimbal.move_angles_sync(p_angle, y_angle)
    time.sleep(0.5)
    print("任务1 完成。")
    gimbal.reset()

def task_2_clockwise_rect(gimbal, solver):
    """
    任务2：沿边线顺时针移动一周 (修正版：适配增量运动接口)
    逻辑：
    1. 计算目标点的绝对角度
    2. 计算 (目标角度 - 当前角度) 得到增量
    3. 调用 move_angles_sync 执行增量移动
    """
    print("\n>>> 执行任务2：顺时针跑圈")
    
    # 1. 定义路径点序列 (顺时针：左上 -> 右上 -> 右下 -> 左下 -> 左上)
    path_points = [
        solver.p_tl,  # 起点 (左上)
        solver.p_tr,  # 右上
        solver.p_br,  # 右下
        solver.p_bl,  # 左下
        solver.p_tl   # 回到起点 (闭环)
    ]
    
    # --- 第一步：前往起点 ---
    start_x, start_y = path_points[0]
    # 计算起点的绝对角度
    p_start_abs, y_start_abs = solver.solve_angles(start_x, start_y)
    
    # 假设系统刚复位，当前绝对角度为 0,0
    # 因此前往起点的增量 = 起点绝对角度 - 0
    gimbal.move_angles_sync(p_start_abs, y_start_abs)
    
    # 更新当前所在的绝对角度
    p_current = p_start_abs
    y_current = y_start_abs
    
    time.sleep(0.5)
    
    # --- 第二步：开始跑圈 ---
    
    # 遍历剩余的点 (从索引1开始)
    for i, (tx, ty) in enumerate(path_points[1:]):
        
        # 1. 解算下一个目标的绝对角度
        p_target_abs, y_target_abs = solver.solve_angles(tx, ty)
        
        # 2. 计算增量 (Delta = Target - Current)
        p_delta = p_target_abs - p_current
        y_delta = y_target_abs - y_current
        
        # 3. 执行增量移动
        # 注意：这里传入的是差值，因为接口是“在此基础上再转多少度”
        gimbal.move_angles_sync(p_delta, y_delta)
        
        # 4. 更新当前角度记录 (电机动完后，我们就到了目标位置)
        p_current = p_target_abs
        y_current = y_target_abs
        
        time.sleep(0.5)
    
    print("任务2 完成。")
    time.sleep(0.5)
    gimbal.reset()

# ================= 主程序入口 =================

if __name__ == "__main__":
    print("=== 选题6 任务程序 (简化版) ===")
    
    solver = GeometricSolver()
    
    try:
        with User_Motor() as gimbal:
            # 上电先复位一次
            print("初始化复位中...")
            gimbal.reset()
            time.sleep(2) # 等待复位完全稳定
            
            while True:
                cmd = input("\n请输入指令 (1=任务一回中, 2=任务二跑圈, q=退出): ")
                
                if cmd == '1':
                    task_1_return_to_center(gimbal, solver)
                elif cmd == '2':
                    task_2_clockwise_rect(gimbal, solver)
                elif cmd == 'q':
                    break
                else:
                    print("未知指令")
                    
    except Exception as e:
        print(f"发生异常: {e}")