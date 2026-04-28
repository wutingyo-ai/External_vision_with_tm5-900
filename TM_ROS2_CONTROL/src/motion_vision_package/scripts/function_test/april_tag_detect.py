#!/home/lips/TM_ROS2_CONTROL/venv/bin/python3
import cv2
import numpy as np
from apriltag import Detector, DetectorOptions
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def draw_tag_axes(overlay, camera_params, tag_size, pose):
    """
    overlay: BGR image
    camera_params: fx, fy, cx, cy
    tag_size: tag邊長 (m)
    pose: 4x4 位姿矩陣
    """

    fx, fy, cx, cy = camera_params
    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0, 0, 1]])

    # 原點
    origin = np.array([[0,0,0]], dtype=np.float32)

    # XYZ 軸向量
    axes = np.array([
        [tag_size, 0, 0],  # X
        [0, tag_size, 0],  # Y
        [0, 0, -tag_size]  # Z (camera看向Z正向為遠離鏡頭)
    ], dtype=np.float32)

    # 旋轉向量 + 平移向量
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]

    # 投影
    img_pts, _ = cv2.projectPoints(np.vstack((origin, axes)), rvec, tvec, K, None)
    img_pts = img_pts.reshape(-1, 2).astype(int)

    origin_pt = tuple(img_pts[0])
    x_pt = tuple(img_pts[1])
    y_pt = tuple(img_pts[2])
    z_pt = tuple(img_pts[3])

    # 畫線
    cv2.line(overlay, origin_pt, x_pt, (0,0,255), 2)  # X 紅色
    cv2.line(overlay, origin_pt, y_pt, (0,255,0), 2)  # Y 綠色
    cv2.line(overlay, origin_pt, z_pt, (255,0,0), 2)  # Z 藍色
    
# 可視化 tag 的 3D pose (立方體)
def draw_tag_pose_cubic(overlay, camera_params, tag_size, pose):
    fx, fy, cx, cy = camera_params
    K = np.array([[fx, 0, cx],
                    [0, fy, cy],
                    [0, 0, 1]])
    
    # 立方體頂點
    cube_pts = np.array([
        [-0.5, -0.5, 0],
        [ 0.5, -0.5, 0],
        [ 0.5,  0.5, 0],
        [-0.5,  0.5, 0],
        [-0.5, -0.5, -1],
        [ 0.5, -0.5, -1],
        [ 0.5,  0.5, -1],
        [-0.5,  0.5, -1],
    ]) * tag_size

    # 旋轉向量 + 平移向量
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]

    # 投影到影像平面
    img_pts, _ = cv2.projectPoints(cube_pts, rvec, tvec, K, None)
    img_pts = img_pts.reshape(-1, 2).astype(int)

    # 立方體邊
    edges = [
        (0,1),(1,2),(2,3),(3,0),
        (0,4),(1,5),(2,6),(3,7),
        (4,5),(5,6),(6,7),(7,4)
    ]

    for i,j in edges:
        cv2.line(overlay, tuple(img_pts[i]), tuple(img_pts[j]), (0,255,0), 2)

def draw_coordinate_frame(ax, T=np.eye(4), axis_length=0.03, label=None):
    """
    在 matplotlib 3D 中畫出座標系
    """
    origin = T[:3, 3]
    R = T[:3, :3]

    x_dir = R[:, 0] * axis_length
    y_dir = R[:, 1] * axis_length
    z_dir = R[:, 2] * axis_length

    ax.quiver(origin[0], origin[1], origin[2],
              x_dir[0], x_dir[1], x_dir[2], color='r')

    ax.quiver(origin[0], origin[1], origin[2],
              y_dir[0], y_dir[1], y_dir[2], color='g')

    ax.quiver(origin[0], origin[1], origin[2],
              z_dir[0], z_dir[1], z_dir[2], color='b')

    ax.scatter(origin[0], origin[1], origin[2], c='k', s=30)

    if label is not None:
        ax.text(origin[0], origin[1], origin[2], label)

def draw_tag_plane(ax, T, tag_size=0.05):
    """
    在 matplotlib 3D 中畫出 tag 平面
    """
    half = tag_size / 2.0

    corners_local = np.array([
        [-half, -half, 0, 1],
        [ half, -half, 0, 1],
        [ half,  half, 0, 1],
        [-half,  half, 0, 1],
    ]).T

    corners_world = T @ corners_local
    corners_world = corners_world[:3, :].T

    corners_closed = np.vstack([corners_world, corners_world[0]])

    ax.plot(corners_closed[:, 0],
            corners_closed[:, 1],
            corners_closed[:, 2], 'k--', linewidth=1)




def main():
    
    imagepath = '/home/lips/TM_ROS2_CONTROL/src/Motion_vision_package/tag_detect_image/rgb_image.png'
    img_gray = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
    img_color = cv2.imread(imagepath, cv2.IMREAD_COLOR)

    if img_gray is None or img_color is None:
        print("圖片讀取失敗")
        return

    # -----------------------------
    # Camera intrinsics
    fx, fy, cx, cy = 385.6169128417969, 385.35711669921875, 330.66339111328125, 244.81097412109375
    camera_params = (fx, fy, cx, cy)

    # -----------------------------
    # 設定 Detector 與 family
    options = DetectorOptions()
    options.families = "tag16h5"
    detector = Detector(options)

    # Tag 尺寸 (單位: m)
    tag_size = 0.05  # 50 mm

    # -----------------------------
    # 偵測 tags
    detections, dimg = detector.detect(img_gray, return_image=True)

    print(f"Detected {len(detections)} tags")

    # 儲存所有 tag pose，後面給 matplotlib 用
    tag_poses = []

    # -----------------------------
    # Overlay: 原圖 + 偵測結果
    if len(img_color.shape) == 3:
        overlay = (
            0.5 * img_color.astype(np.float32) +
            0.5 * dimg[:, :, None].astype(np.float32)
        ).astype(np.uint8)
    else:
        overlay = (
            0.5 * img_gray.astype(np.float32) +
            0.5 * dimg.astype(np.float32)
        ).astype(np.uint8)

    # -----------------------------
    # 處理每個 tag
    for det in detections:
        print(f"\nTag ID: {det.tag_id}")

        # 計算 4x4 位姿矩陣
        pose, init_error, final_error = detector.detection_pose(
            det, camera_params, tag_size=tag_size
        )

        print("4x4 Transformation Matrix:\n", pose)
        print("InitError:", init_error, "FinalError:", final_error)
        #  如果是 ID=4，就額外保存
        if det.tag_id == 4:
            saved_pose_id4 = pose.copy()
            print(">>> 已保存 Tag ID 4 的 pose")
        # 存起來給 matplotlib 用
        tag_poses.append((det.tag_id, pose))

        # 在 OpenCV 影像上畫 XYZ 軸
        draw_tag_axes(overlay, camera_params, tag_size, pose)

        # 顯示 Tag ID
        center = tuple(det.center.astype(int))
        cv2.putText(overlay, f"ID:{det.tag_id}", center,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # -----------------------------
    # 顯示 OpenCV 結果
    cv2.imshow("Apriltag Detection + Pose", overlay)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # -----------------------------
    # Matplotlib 3D 視覺化
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Camera frame（相機自己在原點）
    T_camera = np.eye(4)
    draw_coordinate_frame(ax, T_camera, axis_length=0.05, label="Camera")

    # 每個 tag frame + tag plane
    for tag_id, pose in tag_poses:
        draw_coordinate_frame(ax, pose, axis_length=0.03, label=f"Tag {tag_id}")
        draw_tag_plane(ax, pose, tag_size=tag_size)

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("AprilTag Poses in Camera Coordinate System")

    # 很重要：避免 3D 顯示比例怪掉
    ax.set_box_aspect([1, 1, 1])

    # 你可以依照實際距離微調
    ax.set_xlim(-0.2, 0.2)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(0, 0.5)

    plt.show()   

if __name__ == "__main__":
    main()
    
    
    
# for det in detections:
#     print("ID:", det.tag_id)
#     print("Rotation:\n", det.pose_R)
#     print("Translation:\n", det.pose_t)

#     # 生成 4x4 轉移矩陣
#     T = np.eye(4)
#     T[:3, :3] = det.pose_R
#     T[:3, 3] = det.pose_t.flatten()
#     print("4x4 Transformation Matrix:\n", T)
    
    