import cv2
import numpy as np

def nothing(x):
    pass

def sort_points(pts):
    pts = np.array(pts, dtype=np.float32)
    cx = np.mean(pts[:, 0])
    cy = np.mean(pts[:, 1])

    angles = []
    for p in pts:
        dx = p[0] - cx
        dy = p[1] - cy
        angle = np.arctan2(dy, dx)
        angles.append(angle)

    sorted_p = np.argsort(angles)
    pts_sorted = pts[sorted_p]
    return pts_sorted

# 对结果输出顶点排序，并返回数组，方便进行舵机移动
def sort_points_clockwise(pts):
    pts = np.array(pts, dtype=np.float32)
    center = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:,1] - center[1], pts[:,0] - center[0])
    sorted_indices = np.argsort(angles)
    pts_sorted = pts[sorted_indices]

    # 确保左上角在第一个
    # 左上角应该是x+y最小的点
    idx_topleft = np.argmin(pts_sorted[:, 0] + pts_sorted[:, 1])
    pts_ordered = np.roll(pts_sorted, -idx_topleft, axis=0)

    return pts_ordered.tolist()

def detect_border_points(frame, gray_thresh=55, epsilon_slider=10, dist_thresh_slider=15,
                         canny_thresh1=50, canny_thresh2=150,
                         adaptive_block_size=11, adaptive_c=2,
                         gaussian_kernel=5, visualize=False):
    """
    检测边界点
    参数:
        frame: 输入图像
        gray_thresh: 灰度阈值（保留参数以兼容旧代码，但不再使用）
        epsilon_slider: 多边形拟合精度
        dist_thresh_slider: 距离变换阈值
        canny_thresh1: Canny低阈值
        canny_thresh2: Canny高阈值
        adaptive_block_size: 自适应二值化的块大小（必须为奇数）
        adaptive_c: 自适应二值化的常数C
        gaussian_kernel: 高斯滤波核大小（必须为奇数）
        visualize: 是否返回可视化图像
    返回:
        如果 visualize=False: 返回四个顶点坐标列表或None
        如果 visualize=True: 返回 (顶点列表, 二值化图, 边沿检测图, 结果图)
    """
    kernel = np.ones((7,7), np.uint8)
    results = []
    dist_thresh = float(dist_thresh_slider)

    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 自适应二值化（替代固定阈值二值化）
    # 确保 block_size 为奇数
    if adaptive_block_size % 2 == 0:
        adaptive_block_size += 1
    binary_mask = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,  # 使用高斯权重
        cv2.THRESH_BINARY_INV,            # 反转二值化
        adaptive_block_size,              # 块大小
        adaptive_c                         # 常数C
    )
    # Canny边沿检测
    edges = cv2.Canny(gray, canny_thresh1, canny_thresh2)

    # 形态学闭运算（使用边沿检测结果）
    closed_mask = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=1)

    # 轮廓检测
    contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 创建结果图像
    result_img = frame.copy() if visualize else None

    if len(contours) > 0:
        outer_contour = max(contours, key=cv2.contourArea)

        C_outer = cv2.arcLength(outer_contour, True)
        epsilon_outer = (epsilon_slider / 100.0) * C_outer
        P_outer = cv2.approxPolyDP(outer_contour, epsilon_outer, True)

        if len(P_outer) == 4:

            mask_for_dist = np.zeros_like(closed_mask)
            cv2.fillPoly(mask_for_dist, [P_outer], 255)
            dist_map = cv2.distanceTransform(mask_for_dist, cv2.DIST_L2, 3)
            _, inside_mask = cv2.threshold(dist_map, dist_thresh, 255, cv2.THRESH_BINARY)
            inside_mask = inside_mask.astype(np.uint8)*255
            inner_contours, _ = cv2.findContours(inside_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(inner_contours) > 0:
                inner_contour = max(inner_contours, key=cv2.contourArea)

                C_inner = cv2.arcLength(inner_contour, True)
                epsilon_inner = (epsilon_slider / 100.0) * C_inner
                P_inner = cv2.approxPolyDP(inner_contour, epsilon_inner, True)

                if len(P_inner) == 4:
                    outer_pts = [p[0] for p in P_outer]
                    inner_pts = [p[0] for p in P_inner]
                    outer_pts_sorted = sort_points(outer_pts)
                    inner_pts_sorted = sort_points(inner_pts)
                    for p_out, p_in in zip(outer_pts_sorted, inner_pts_sorted):
                        mx = int((p_out[0] + p_in[0]) / 2)
                        my = int((p_out[1] + p_in[1]) / 2)
                        results.append((mx, my))

                    # 在结果图上绘制
                    if visualize and result_img is not None:
                        # 绘制外轮廓
                        cv2.polylines(result_img, [np.int32(outer_pts_sorted)], True, (0, 255, 0), 2)
                        # 绘制内轮廓
                        cv2.polylines(result_img, [np.int32(inner_pts_sorted)], True, (255, 0, 0), 2)
                        # 绘制四个顶点
                        for i, mp in enumerate(results):
                            cv2.circle(result_img, mp, 8, (0, 0, 255), -1)
                            cv2.putText(result_img, f"P{i}: {mp}", (mp[0]+10, mp[1]-10),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    ordered_results = None
    if len(results) == 4:
        ordered_results = sort_points_clockwise(results)

    if visualize:
        return ordered_results, binary_mask, edges, result_img
    else:
        return ordered_results


# 主程序：用于测试和可视化
if __name__ == "__main__":
    from cam import Camera

    # 初始化摄像头
    cam = Camera()

    # 创建窗口
    cv2.namedWindow("1. Binary Threshold", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("2. Edge Detection", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("3. Final Result", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("Control", cv2.WINDOW_AUTOSIZE)

    # 创建滑块
    cv2.createTrackbar("AdaptBlockSize", "Control", 40, 99, nothing)   # 自适应块大小 (3-99)
    cv2.createTrackbar("AdaptC", "Control", 10, 20, nothing)            # 自适应常数C (0-20)
    cv2.createTrackbar("GaussianKernel", "Control", 3, 15, nothing)    # 高斯滤波核 (1-15)
    cv2.createTrackbar("Canny_Thresh1", "Control", 150, 255, nothing)
    cv2.createTrackbar("Canny_Thresh2", "Control", 255, 255, nothing)
    cv2.createTrackbar("Epsilon", "Control", 1, 50, nothing)
    cv2.createTrackbar("DistThresh", "Control", 15, 50, nothing)

    print("边界点检测程序已启动")
    print("按 'q' 键退出")
    print("\n窗口说明:")
    print("  1. Binary Threshold - 自适应二值化 + 高斯滤波结果")
    print("  2. Edge Detection   - Canny边沿检测结果")
    print("  3. Final Result     - 最终多边形拟合结果（标注四个顶点）")
    print("\n参数说明:")
    print("  AdaptBlockSize - 自适应二值化块大小（建议奇数，如11, 15, 21）")
    print("  AdaptC         - 自适应二值化常数C（建议0-10）")
    print("  GaussianKernel - 高斯滤波核大小（建议奇数，如3, 5, 7）")

    while True:
        # 读取帧
        frame = cam.read_frame()
        if frame is None:
            print("无法读取摄像头画面")
            break

        # 获取滑块参数
        adaptive_block_size = cv2.getTrackbarPos("AdaptBlockSize", "Control")
        adaptive_c = cv2.getTrackbarPos("AdaptC", "Control")
        gaussian_kernel = cv2.getTrackbarPos("GaussianKernel", "Control")
        canny_thresh1 = cv2.getTrackbarPos("Canny_Thresh1", "Control")
        canny_thresh2 = cv2.getTrackbarPos("Canny_Thresh2", "Control")
        epsilon_slider = cv2.getTrackbarPos("Epsilon", "Control")
        dist_thresh_slider = cv2.getTrackbarPos("DistThresh", "Control")

        # 确保参���有效（避免为0导致错误）
        if adaptive_block_size < 3:
            adaptive_block_size = 3
        if gaussian_kernel < 1:
            gaussian_kernel = 1

        # 调用检测函数（可视化模式）
        results, binary_img, edge_img, result_img = detect_border_points(
            frame,
            epsilon_slider=epsilon_slider,
            dist_thresh_slider=dist_thresh_slider,
            canny_thresh1=canny_thresh1,
            canny_thresh2=canny_thresh2,
            adaptive_block_size=adaptive_block_size,
            adaptive_c=adaptive_c,
            gaussian_kernel=gaussian_kernel,
            visualize=True
        )

        # 显示结果
        cv2.imshow("1. Binary Threshold", binary_img)
        cv2.imshow("2. Edge Detection", edge_img)
        cv2.imshow("3. Final Result", result_img)

        # 如果检测到四个顶点，打印坐标
        if results is not None:
            print(f"\r检测到四个顶点: {results}", end="", flush=True)
        else:
            print("\r未检测到有效的四边形", end="", flush=True)

        # 按键检测
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    print("\n程序退出")
    cv2.destroyAllWindows()
    cam.release()