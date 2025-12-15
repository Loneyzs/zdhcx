# -*- coding: utf-8 -*-

import cv2
import os, sys
import threading
import time
from config import *

class Camera:
    """
    摄像头采集类：
    """
    def __init__(self):
        self._last_idx   = -1              # 上次读出的帧序号（软件计数）
        if FORCE_NO_PIPELINE or is_windows:
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, CAM_FPS)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            pipeline = (
                "v4l2src device=/dev/video0 io-mode=4 do-timestamp=true ! "
                "image/jpeg, width={CAM_WIDTH}, height={CAM_HEIGHT}, framerate={CAM_FPS}/1 ! "
                "jpegparse ! jpegdec ! "
                "videoconvert ! video/x-raw,format=BGR,width={CAM_WIDTH}, height={CAM_HEIGHT} ! "
                "queue max-size-buffers=1 leaky=downstream ! "
                "appsink drop=true max-buffers=1 sync=false enable-last-sample=false emit-signals=false"
            )
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 采集线程共享状态
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest = None                # 最新帧
        self._seq = -1                     # 采集线程递增的软件帧号
        self._running = True

        # 启动采集线程：持续读取并覆盖最新帧，只保留 1 帧
        self._th = threading.Thread(target=self._capture_loop, daemon=True)
        self._th.start()

    def _capture_loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                # 读取失败时短暂休眠避免空转
                time.sleep(0.001)
                continue
            with self._cond:
                self._latest = frame
                self._seq += 1
                self._cond.notify_all()

    def read_frame(self):
        """
        阻塞直到有“新帧”（软件帧号 _seq 严格大于 _last_idx）再返回。
        这样可保证下游绝不会两次读到同一帧；如果调用过快，会等待下一帧。
        """
        with self._cond:
            while self._running and self._seq == self._last_idx:
                self._cond.wait()
            if not self._running:
                return None
            # 返回时更新已读序号，确保同一帧不会被重复返回
            self._last_idx = self._seq
            return self._latest

    def read_zoomed_frame(self):
        """
        获取数码变焦帧：
        - 宽度：裁切掉左右各1/3，保留中心1/3
        - 高度：裁切掉上下各1/4，保留中心1/2
        基于read_frame()实现，确保帧同步特性
        """
        frame = self.read_frame()
        if frame is None:
            return None
        
        height, width = frame.shape[:2]
        
        left_crop = width // 5
        right_crop = width - width // 5

        top_crop = height // 5      # 上边裁切掉1/4
        # bottom_crop = height - height // 5  # 下边裁切掉1/4
        bottom_crop = height
        
        # 双向裁切：保留宽度中心1/3，高度中心1/2
        zoomed_frame = frame[top_crop:bottom_crop, left_crop:right_crop]
        return zoomed_frame

    def release(self):
        self._running = False
        try:
            self.cap.release()
        except Exception:
            pass
        with self._cond:
            self._cond.notify_all()
        if hasattr(self, "_th") and self._th.is_alive():
            try:
                self._th.join(timeout=1.0)
            except Exception:
                pass


if __name__ == '__main__':
    cam = Camera()
    try:
        while True:
            frame = cam.read_frame()
            if frame is None:
                break
            cv2.imshow("Camera Test", frame)
            
            # 按 q 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.release()
        cv2.destroyAllWindows()
