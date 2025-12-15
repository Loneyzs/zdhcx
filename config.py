import platform
import numpy as np

# !!! force close pipeline !!!
FORCE_NO_PIPELINE = True

# ---------- 运行平台判断 ---------- #
is_windows = platform.system().lower().startswith("win")

# ---------- 步进电机串口参数 ---------- #
MOTOR_SERIAL_PORT = "COM5" if is_windows else "/dev/ttyUSB0"
MOTOR_BAUDRATE = 115200
MOTOR_TIMEOUT = 0.1

# ---------- 相机参数 ---------- #
CAM_WIDTH  = 1920
CAM_HEIGHT = 1080
#CAM_WIDTH  = 1152
#CAM_HEIGHT = 648
CAM_FPS    = 30
CAMERA_INDEX = 0 if is_windows else "/dev/video0"

# ---------- 云台控制默认参数 ---------- #
GIMBAL_PITCH_ADDR = 1          # Pitch轴电机地址
GIMBAL_YAW_ADDR = 2            # Yaw轴电机地址
GIMBAL_DEFAULT_DIRECTION = 0   # 默认方向：0为顺时针，1为逆时针
GIMBAL_DEFAULT_VELOCITY = 100  # 默认速度(RPM)
GIMBAL_DEFAULT_ACCELERATION = 30  # 默认加速度
GIMBAL_DEFAULT_SYNC = True    # 默认多机同步标志

# ---------- 回零模式参数 ---------- #
ORIGIN_MODE_SINGLE_TURN = 0    # 单圈回零模式

# ---------- 选题6 几何参数配置 ---------- #
# 单位: cm
SCREEN_DISTANCE = 100.0       # 激光笔距离屏幕的水平距离
GIMBAL_HEIGHT = 15.0          # 云台激光笔中心的离地高度
PENCIL_HEIGHT_OFFSET = 15.0   # 铅笔画的正方形底边离地面的高度 (根据你的描述调整)
SQUARE_SIZE = 50.0            # 正方形边长