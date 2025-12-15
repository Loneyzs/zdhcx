# -*- coding: utf-8 -*-
import sys
import signal
import datetime
import cv2
from cam import Camera

def cleanup(cam):
    """
    释放摄像头和销毁所有窗口，确保 GStreamer 元素被置为 NULL。
    """
    if cam and hasattr(cam, 'cap'):
        try:
            cam.cap.release()
            # 强制一次小延时，确保管线状态切换完成
            cv2.waitKey(1)
        except Exception as e:
            print(f"释放摄像头时出错：{e}")
    cv2.destroyAllWindows()
    print("已释放资源，程序退出。")
    sys.exit(0)

def signal_handler(sig, frame):
    # 捕获 Ctrl+C 信号
    cleanup(cam)

if __name__ == "__main__":
    # 全局 cam，用于 signal handler
    cam = None

    # 注册 SIGINT（Ctrl+C）信号处理
    signal.signal(signal.SIGINT, signal_handler)

    # 1. 初始化摄像头
    cam = Camera()
    if not cam.cap.isOpened():
        print("错误：无法打开摄像头管线，请检查设备连接和 GStreamer 管线配置。")
        cleanup(cam)

    print("摄像头管线已打开，开始捕获新帧。按 Ctrl+C 退出。")

    # 2. 循环读取新帧并输出时间戳
    try:
        last_frame_id = -1
        while True:
            # 如果 Camera.read_frame() 直接返回图像：
            frame = cam.read_frame()

            # （可选）若 Camera 有帧 ID 属性，可在此判断新帧
            # frame_id = cam.get_frame_id()
            # if frame_id == last_frame_id:
            #     continue
            # last_frame_id = frame_id

            # 获取当前时间戳，精确到毫秒
            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            print(f"[{ts}] 获取到新帧")

            # （可选）显示画面
            # cv2.imshow("Live", frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    except Exception as e:
        print(f"运行时出错：{e}")
    finally:
        cleanup(cam)
