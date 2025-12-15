#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步进电机云台控制器 (用户接口层)
======================================
V5.3 完整功能最终版 by Gemini

- [功能恢复] 恢复了在初始化时自动配置电机响应模式的功能，使设计更加健壮。
- [架构] 保持 V5.2 的双API模式：move_pulses_sync (阻塞) 和 
  move_pulses_async (非阻塞)，以适应不同场景的需求。
- 依赖于 Motor.py V5.3 或更高版本。
"""

import time
from typing import Tuple, Optional, List, Dict

try:
    from Motor import StepperMotorController
    from config import (
        GIMBAL_PITCH_ADDR, GIMBAL_YAW_ADDR,
        GIMBAL_DEFAULT_VELOCITY, GIMBAL_DEFAULT_ACCELERATION,ORIGIN_MODE_SINGLE_TURN
    )
except ImportError:
    print("警告: 无法找到 Motor.py 或 config.py，将使用虚拟电机类。")
    class StepperMotorController:
        def enable_control(self, *a, **k): pass
        def configure_driver_response(self, *a, **k): return True
        def position_control(self, *a, **k): pass
        def velocity_control(self, *a, **k): pass
        def synchronous_motion(self, *a, **k): pass
        def stop_now(self, *a, **k): pass
        def _wait_for_positions_settled_polling(self, *a, **k): return True
        def get_motor_pulses(self, *a, **k): return 0
        def close(self): pass
    GIMBAL_PITCH_ADDR, GIMBAL_YAW_ADDR, GIMBAL_DEFAULT_VELOCITY, GIMBAL_DEFAULT_ACCELERATION = 1, 2, 100, 30

class User_Motor:
    def __init__(self):
        """初始化云台控制器"""
        self.controller = StepperMotorController()
        self.pitch_addr = GIMBAL_PITCH_ADDR
        self.yaw_addr = GIMBAL_YAW_ADDR
        
        self.log("云台初始化开始...")
        self._initialize_motors()
        self.log(f"云台初始化完成 - Pitch轴(电机{self.pitch_addr}), Yaw轴(电机{self.yaw_addr})")
        
    def __enter__(self): return self
    def __exit__(self, exc_type, exc_val, exc_tb): self.close()
        
    def close(self):
        """关闭云台控制器"""
        if hasattr(self, 'controller'):
            self.log("关闭云台连接...")
            self.controller.close()
            
    def _initialize_motors(self):
        """内部方法：使能电机并尝试自动配置响应模式。"""
        try:
            self.log("尝试配置Pitch轴为'到位返回'模式...")
            if not self.controller.configure_driver_response(self.pitch_addr, desired_modes=['Reached', 'Both', 'Other']):
                self.log("警告: 配置Pitch轴响应模式失败。将完全依赖轮询模式。")
            
            self.log("尝试配置Yaw轴为'到位返回'模式...")
            if not self.controller.configure_driver_response(self.yaw_addr, desired_modes=['Reached', 'Both', 'Other']):
                self.log("警告: 配置Yaw轴响应模式失败。将完全依赖轮询模式。")

            self.log("使能双轴电机...")
            self.controller.enable_control(self.pitch_addr, state=True, sync_flag=False)
            self.controller.enable_control(self.yaw_addr, state=True, sync_flag=False)
            time.sleep(0.05)
        except Exception as e:
            raise RuntimeError(f"电机初始化失败: {e}")
    
    def _execute_move(self, pitch_pulses: int, yaw_pulses: int):
        """内部核心移动指令发送函数"""
        pitch_dir = 1 if pitch_pulses < 0 else 0
        self.controller.position_control(self.pitch_addr, pitch_dir, GIMBAL_DEFAULT_VELOCITY, 
                                         GIMBAL_DEFAULT_ACCELERATION, abs(pitch_pulses), False, True)
        yaw_dir = 1 if yaw_pulses < 0 else 0
        self.controller.position_control(self.yaw_addr, yaw_dir, GIMBAL_DEFAULT_VELOCITY, 
                                       GIMBAL_DEFAULT_ACCELERATION, abs(yaw_pulses), False, True)
        self.controller.synchronous_motion()

    def reset_1(self):
        """
        pitch轴回零
        """
        self.controller.origin_trigger_return(
            self.pitch_addr, 
            ORIGIN_MODE_SINGLE_TURN, 
            sync_flag=False
        )

    def reset_2(self):
        """
        yaw轴回零
        """
        self.controller.origin_trigger_return(
            self.yaw_addr, 
            ORIGIN_MODE_SINGLE_TURN, 
            sync_flag=False
        )

    def reset(self):
        self.controller.origin_trigger_return(
            self.yaw_addr, 
            ORIGIN_MODE_SINGLE_TURN, 
            sync_flag=False
        )
        self.controller.origin_trigger_return(
            self.pitch_addr, 
            ORIGIN_MODE_SINGLE_TURN, 
            sync_flag=False
        )
    
    def move_pulses_sync(self, pitch_pulses: int, yaw_pulses: int, timeout: float = 10.0):
        """
        同步（阻塞）移动双轴。
        此函数会等待运动完成后再返回。
        """
        if pitch_pulses == 0 and yaw_pulses == 0:
            return

        self.log(f"同步移动: P={pitch_pulses}, Y={yaw_pulses}")
        
        targets_to_poll = {}
        p_start, y_start = self.get_gimbal_pulses()
        if p_start is None or y_start is None:
            self.log("获取初始位置失败，无法执行同步移动。", level="ERROR")
            return
        
        if pitch_pulses != 0:
            targets_to_poll[self.pitch_addr] = p_start + pitch_pulses
        if yaw_pulses != 0:
            targets_to_poll[self.yaw_addr] = y_start + yaw_pulses

        self._execute_move(pitch_pulses, yaw_pulses)
        
        if targets_to_poll:
            if not self.controller._wait_for_positions_settled_polling(targets_to_poll, timeout=timeout):
                self.log("⚠️ 同步移动等待超时或失败", level="ERROR")

    def move_pulses_async(self, pitch_pulses: int, yaw_pulses: int):
        """
        异步（非阻塞）移动双轴。
        此函数发送指令后立即返回。
        """
        if pitch_pulses == 0 and yaw_pulses == 0:
            return
        
        self._execute_move(pitch_pulses, yaw_pulses)

    def get_gimbal_pulses(self) -> Tuple[Optional[int], Optional[int]]:
        """获取云台两轴当前脉冲数"""
        pitch_pulses = self.controller.get_motor_pulses(self.pitch_addr)
        yaw_pulses = self.controller.get_motor_pulses(self.yaw_addr)
        return pitch_pulses, yaw_pulses

    def stop_all(self):
        """立即停止所有电机"""
        self.controller.stop_now(0, sync_flag=False)
        self.log("云台所有轴已发送紧急停止指令")

    def start_continuous_move(self, pitch_rpm: int = 0, yaw_rpm: int = 0):
        """使用速度模式开始连续运动"""
        self.log(f"速度模式启动: Pitch={pitch_rpm} RPM, Yaw={yaw_rpm} RPM")
        pitch_dir = 0 if pitch_rpm >= 0 else 1
        self.controller.velocity_control(self.pitch_addr, pitch_dir, abs(pitch_rpm),
                                         GIMBAL_DEFAULT_ACCELERATION, True)
        yaw_dir = 0 if yaw_rpm >= 0 else 1
        self.controller.velocity_control(self.yaw_addr, yaw_dir, abs(yaw_rpm),
                                       GIMBAL_DEFAULT_ACCELERATION, True)
        self.controller.synchronous_motion()

    def stop_continuous_move(self):
        """停止所有由速度模式启动的运动"""
        self.log("速度模式停止。")
        self.stop_all()
        
    def log(self, message, level="INFO"):
        """统一的日志记录接口"""
        print(f"[{time.strftime('%H:%M:%S')}|User_Motor|{level}] {message}")
    
    def _calculate_shortest_angle_diff(self, current_angle: float, target_angle: float) -> float:
        # 标准化角度到0-360范围
        current_angle = current_angle % 360.0
        target_angle = target_angle % 360.0
        
        # 计算直接差值
        diff = target_angle - current_angle
        
        # 选择最短路径
        if diff > 180.0:
            diff -= 360.0
        elif diff < -180.0:
            diff += 360.0
            
        return diff

    def move_to_angles_sync(self, pitch_angle: float, yaw_angle: float, velocity: int = GIMBAL_DEFAULT_VELOCITY):
        # 获取当前脉冲
        pitch_pulses = self.controller.get_motor_pulses(self.pitch_addr)
        yaw_pulses = self.controller.get_motor_pulses(self.yaw_addr)

        current_pitch = pitch_pulses / 3200.0
        current_yaw = yaw_pulses / 3200.0
        
        # 计算最短角度差
        pitch_diff = self._calculate_shortest_angle_diff(current_pitch, pitch_angle)
        yaw_diff = self._calculate_shortest_angle_diff(current_yaw, yaw_angle)

        # 转換为脉冲数
        pitch_pulses = int(pitch_diff * 3200.0 / 360.0)
        yaw_pulses = int(yaw_diff * 3200.0 / 360.0)
        
        # 执行同步运动
        self.move_pulses_async(pitch_pulses, yaw_pulses)


if __name__ == "__main__":
    """测试例程 - 测试move_pulses_async和move_to_angles_sync方法"""
    print("=== User_Motor 测试例程 ===")
    
    try:
        with User_Motor() as gimbal:
            #gimbal.reset()
            #time.sleep(1)
            gimbal.start_continuous_move(1,1)
            #time.sleep(3)

            #gimbal.reset()
            #time.sleep(3)
            #gimbal.move_to_angles_sync(0,90)
            # gimbal.reset_1()
            # time.sleep(0.7)
            # gimbal.reset_2()
            # time.sleep(0.7)
            # gimbal.move_to_angles_sync(0, -90)
            # time.sleep(1.5)

            # gimbal.reset_2()
            # time.sleep(0.7)
            
    except Exception as e:
        print(f"测试失败: {e}")
    
    print("测试例程结束")
