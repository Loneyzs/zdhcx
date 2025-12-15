# -*- coding: utf-8 -*-
"""
基于Python3 + pyserial的步进电机控制器 - 单串口版 (完整功能最终版)
====================================================================
V5.3 by Gemini

- [功能恢复] 完整恢复了 configure_driver_response 函数及其相关的所有底层功能，
  确保代码的完整性和可扩展性。
- [API重构] 将核心的轮询等待函数 _wait_for_positions_settled_polling 标记为
  内部函数，明确其职责是作为上层业务逻辑的可靠支撑。
- [关注点分离] 此层专注于提供稳定可靠的底层通信和原子操作，将复杂的等待策略
  完全交由上层 User_Motor 模块处理。
"""

import time
import serial
import struct
import threading
from typing import List, Tuple, Optional, Dict

try:
    from config import MOTOR_SERIAL_PORT, MOTOR_BAUDRATE, MOTOR_TIMEOUT
except ImportError:
    MOTOR_SERIAL_PORT, MOTOR_BAUDRATE, MOTOR_TIMEOUT = 'COM9', 115200, 0.1

FUNC_CODES = {'S_Conf': 0x42, 'S_CPOS': 0x36, 'M_Conf': 0x48, 'M_Enable': 0xF8, 
              'M_Vel': 0xF6, 'M_Pos': 0xFD, 'M_Stop': 0xFE, 'M_Sync': 0xFF,
              'SUB_Conf_R': 0x6C, 'SUB_Conf_W': 0xD1, 'SUB_Enable': 0xAB,
              'SUB_Stop': 0x98, 'SUB_Sync': 0x66, 'CHECKSUM': 0x6B}
CONF_RESPONSE_INDEX = 15

class StepperMotorController:
    def __init__(self, port: str = None, baudrate: int = MOTOR_BAUDRATE, timeout: float = MOTOR_TIMEOUT):
        self.port = port or MOTOR_SERIAL_PORT
        self.baudrate = baudrate
        self.timeout = timeout
        self.lock = threading.Lock()
        self.uart = None
        try:
            self.uart = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self.log(f"串口初始化成功: {self.port} @ {self.baudrate} bps")
        except serial.SerialException as e:
            self.log(f"串口初始化失败: {e}", level="ERROR")
            raise
            
    def __enter__(self): return self
    def __exit__(self, exc_type, exc_val, exc_tb): self.close()
        
    def close(self):
        if self.uart and self.uart.is_open:
            self.uart.close()
            self.log("串口已关闭。")
            
    def _send_cmd_nolock(self, cmd: bytearray):
        # self.log(f"TX -> {' '.join(f'{b:02X}' for b in cmd)}", level="DEBUG")
        self.uart.write(cmd)

    def _receive_data_nolock(self) -> Optional[bytearray]:
        response = self.uart.read_until(expected=bytes([FUNC_CODES['CHECKSUM']]))
        # if response:
            # self.log(f"RX <- {' '.join(f'{b:02X}' for b in response)}", level="DEBUG")
        return bytearray(response) if response else None

    def enable_control(self, addr: int, state: bool, sync_flag: bool):
        cmd = bytearray([addr, FUNC_CODES['M_Enable'], FUNC_CODES['SUB_Enable'], 0x01 if state else 0x00, 0x01 if sync_flag else 0x00, FUNC_CODES['CHECKSUM']])
        with self.lock:
            self._send_cmd_nolock(cmd)
            time.sleep(0.01)

    def position_control(self, addr: int, direction: int, velocity: int,
                         acceleration: int, pulses: int, absolute_flag: bool, sync_flag: bool):
        cmd = bytearray([addr, FUNC_CODES['M_Pos'], direction, (velocity >> 8) & 0xFF, velocity & 0xFF, acceleration,
                         (pulses >> 24) & 0xFF, (pulses >> 16) & 0xFF, (pulses >> 8) & 0xFF, pulses & 0xFF,
                         0x01 if absolute_flag else 0x00, 0x01 if sync_flag else 0x00, FUNC_CODES['CHECKSUM']])
        with self.lock:
            self._send_cmd_nolock(cmd)
            time.sleep(0.005)

    def velocity_control(self, addr: int, direction: int, velocity: int,
                         acceleration: int, sync_flag: bool):
        cmd = bytearray([addr, FUNC_CODES['M_Vel'], direction, (velocity >> 8) & 0xFF, velocity & 0xFF,
                         acceleration, 0x01 if sync_flag else 0x00, FUNC_CODES['CHECKSUM']])
        with self.lock:
            self._send_cmd_nolock(cmd)
            time.sleep(0.005)

    def synchronous_motion(self):
        cmd = bytearray([0x00, FUNC_CODES['M_Sync'], FUNC_CODES['SUB_Sync'], FUNC_CODES['CHECKSUM']])
        with self.lock:
            self._send_cmd_nolock(cmd)
            time.sleep(0.005)

    def stop_now(self, addr: int, sync_flag: bool):
        cmd = bytearray([addr, FUNC_CODES['M_Stop'], FUNC_CODES['SUB_Stop'], 0x01 if sync_flag else 0x00, FUNC_CODES['CHECKSUM']])
        with self.lock:
            self._send_cmd_nolock(cmd)
            time.sleep(0.01)

    def get_motor_pulses(self, addr: int) -> Optional[int]:
        """获取指定地址电机的实时位置（单位：脉冲）。"""
        with self.lock:
            self.uart.reset_input_buffer()
            cmd = bytearray([addr, FUNC_CODES['S_CPOS'], FUNC_CODES['CHECKSUM']])
            self._send_cmd_nolock(cmd)
            response = self._receive_data_nolock()
        
        if response and len(response) >= 8 and response[0] == addr and response[1] == FUNC_CODES['S_CPOS']:
            try:
                encoder_val = struct.unpack('>I', response[3:7])[0]
                sign = -1 if response[2] == 0x01 else 1
                pulses = round(encoder_val * 3200.0 / 65536.0)
                return sign * pulses
            except Exception as e:
                self.log(f"解析电机 {addr} 位置数据失败: {e}", level="ERROR")
        return None
    
    def read_driver_config(self, addr: int) -> Optional[bytearray]:
        with self.lock:
            self.uart.reset_input_buffer()
            cmd = bytearray([addr, FUNC_CODES['S_Conf'], FUNC_CODES['SUB_Conf_R'], FUNC_CODES['CHECKSUM']])
            self._send_cmd_nolock(cmd)
            response = self._receive_data_nolock()
        if response and len(response) > 4 and response[0] == addr and response[1] == FUNC_CODES['S_Conf']:
            return response[2:-1] 
        return None

    def write_driver_config(self, addr: int, config_payload: bytearray, save: bool = True) -> bool:
        cmd = bytearray([addr, FUNC_CODES['M_Conf'], FUNC_CODES['SUB_Conf_W'], 0x01 if save else 0x00])
        cmd.extend(config_payload[2:])
        cmd.append(FUNC_CODES['CHECKSUM'])
        with self.lock:
            self.uart.reset_input_buffer()
            self._send_cmd_nolock(cmd)
            response = self._receive_data_nolock()
        return response and len(response) == 4 and response[2] == 0x02

    def configure_driver_response(self, addr: int, desired_modes: List[str] = ['Reached', 'Both']) -> bool:
        """尝试配置驱动器的响应模式，但不作为核心依赖。"""
        self.log(f"正在为电机 {addr} 检查响应模式...")
        config = self.read_driver_config(addr)
        if not config:
            self.log(f"读取电机 {addr} 配置失败，无法自动配置。", level="ERROR")
            return False

        mode_map = {'None': 0x00, 'Receive': 0x01, 'Reached': 0x02, 'Both': 0x03, 'Other': 0x04}
        desired_values = {mode_map.get(m, 0x02) for m in desired_modes}
        param_offset = 2 + CONF_RESPONSE_INDEX

        if len(config) <= param_offset:
            self.log(f"电机 {addr} 配置块长度不足。", level="ERROR")
            return False
            
        current_mode_val = config[param_offset]
        if current_mode_val in desired_values:
            self.log(f"电机 {addr} 响应模式 ({current_mode_val}) 已符合要求。")
            return True
        
        target_mode_val = list(desired_values)[0]
        self.log(f"电机 {addr} 响应模式值为 {current_mode_val:02X}，将尝试修改为 {target_mode_val:02X}。")
        config[param_offset] = target_mode_val
        
        if self.write_driver_config(addr, config, save=True):
            self.log(f"电机 {addr} 响应模式配置成功！")
            return True
        
        self.log(f"写入电机 {addr} 配置失败。", level="ERROR")
        return False

    def _wait_for_positions_settled_polling(self, targets: Dict[int, int], timeout: float = 10.0, tolerance: int = 5, poll_interval: float = 0.02) -> bool:
        """
        内部使用的、可靠的目标轮询等待函数。
        """
        start_time = time.time()
        self.log(f"开始目标轮询: 目标 {targets}, 容差 {tolerance}...")
        
        while time.time() < start_time + timeout:
            remaining_targets = {}
            for addr, target_pos in targets.items():
                current_pos = self.get_motor_pulses(addr)
                if current_pos is None or abs(current_pos - target_pos) > tolerance:
                    remaining_targets[addr] = target_pos
            
            if not remaining_targets:
                self.log(f"✅ 目标轮询成功。所有电机均已到达目标。总耗时: {time.time() - start_time:.3f}s")
                return True
            
            targets = remaining_targets
            time.sleep(poll_interval)
            
        self.log(f"⚠️ 目标轮询等待超时！未到达的目标: {targets}", level="ERROR")
        return False

    def log(self, message: str, level: str = "INFO"):
        """统一的日志记录接口"""
        print(f"[{time.strftime('%H:%M:%S')}|Motor|{level}] {message}")
    
    def origin_trigger_return(self, addr: int, origin_mode: int, sync_flag: bool) -> None:
        cmd = bytearray([addr, 0x9A, origin_mode, 0x01 if sync_flag else 0x00, 0x6B])  
        with self.lock:                       # 🔒 加锁  
            self._send_cmd_nolock(cmd)        # 复用已有安全写  
            time.sleep(0.01) 