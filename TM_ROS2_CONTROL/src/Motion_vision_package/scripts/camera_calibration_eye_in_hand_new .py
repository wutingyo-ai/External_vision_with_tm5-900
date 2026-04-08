#!/usr/bin/env python3.8
import re

# from flask.config import T
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from scipy.spatial.transform import Rotation as R_scipy
import cv2 as cv
import glob
import os
from apriltag import Detector, DetectorOptions






lips_camera_intrinsic=np.array(
                            [[385.6169128417969,  0,         330.66339111328125 ],
                            [  0,       385.35711669921875 ,244.81097412109375],
                            [  0,           0,           1        ]])

# 定義轉移矩陣的函數
def R_and_t_to_T(R, t):
    T = np.hstack((R, t.reshape(-1, 1)))  # 將平移向量轉換為列向量
    T = np.vstack((T, [0, 0, 0, 1]))  # 添加最後一行
    return T
# 定義轉移矩陣的函數
def T_to_R_and_t(T):
    R = T[:3, :3]
    t = T[:3, 3].reshape(-1, 1)  # 確保 t 是列向量
    return R, t

def april_tag_detect(index_first=0,index_last=5):
    saved_pose_id4=[]
    R_cam_list=[]
    t_cam_list=[]
    
    imagepath = '/home/lips/TM_ROS2_CONTROL/src/Motion_vision_package/tag_detect_image/'
    # images = sorted(glob.glob(os.path.join(path, '*.png')), key=lambda x: int(os.path.basename(x).split('.')[0]))
    
    select_order=range(index_first,index_last)
    # select_order=[5,9,10]
    # 使用 os.path.join() 手動生成檔案名稱
    images = [os.path.join(imagepath, f"{i}.png") for i in select_order[:] ]  # 假設有 10 張影像
    # images = [os.path.join(imagepath, f"{i}.png") for i in list(range(0, 3)) + list(range(3, 11))]
    for fname in images:
        img_gray = cv.imread(fname, cv.IMREAD_GRAYSCALE)
        img_color = cv.imread(fname, cv.IMREAD_COLOR)

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
                print(pose.copy())
                saved_pose_id4.append(pose.copy())
                R_cam_list.append(pose[:3, :3])
                t_cam_list.append(pose[:3, 3].reshape(-1, 1))  # 確保 t 是列向量

    return R_cam_list,t_cam_list

def get_robot_pose_list(index_first=0,index_last=5):
    # 讀取 CSV 檔案
    data = pd.read_csv('/home/lips/TM_ROS2_CONTROL/src/Motion_vision_package/eye_hand_calibration_data/flange_pose_values.csv')
    # 只選擇第 1 至第 5 行（Python 的索引從 0 開始，因此使用 0:5）在 camera_calibration_eye_to_hand_new.py 程式中，data.iloc[0:6] 會取出 CSV 檔案中的前六行數據，這包括標題行和接下來的五行數據。
    data_subset = data.iloc[index_first:index_last]
    # data_subset = data.iloc[[5,9,10]]

    # 存儲轉移矩陣的列表
    flange_to_robot_base_T_matrix_list = []
    flange_to_robot_base_R_matrix_list = []
    flange_to_robot_base_t_matrix_list = []

    # 計算轉移矩陣
    for index, column in data_subset.iterrows():
        # 提取位置和旋轉數據
        X, Y, Z = column['x'], column['y'], column['z']
        Rx, Ry, Rz = column['rx'], column['ry'], column['rz']
        # print(Rx,Ry,Rz)
        # 計算旋轉矩陣
        print(X,Y,Z,Rx,Ry,Rz)
        rotation = R_scipy.from_euler('xyz', [Rx, Ry, Rz], degrees=True).as_matrix()
        
        # 創建平移向量
        translation = np.array([[X/1000], [Y/1000], [Z/1000]])
        
        
        # 計算 base 到 flange 的轉移矩陣
        rotation_inv = rotation.T  # 旋轉矩陣的逆是其轉置
        translation_inv = -rotation_inv @ translation  # 平移向量的逆


        # 使用自定義函數生成轉移矩陣
        T = R_and_t_to_T(rotation, translation)
        
        # 將轉移矩陣添加到列表中
        flange_to_robot_base_R_matrix_list.append(rotation)
        flange_to_robot_base_t_matrix_list.append(translation) 
        flange_to_robot_base_T_matrix_list.append(T)
    return flange_to_robot_base_R_matrix_list,flange_to_robot_base_t_matrix_list

def get_cam_to_flange_matrix(flange_to_robot_base_R_matrix_list,flange_to_robot_base_t_matrix_list,R_cam_matrix_list,t_cam_matrix_list):
    ######################################################################################計算相機RGB座標系至手臂基座座標系關係
    R_cam2flange,t_cam2flange=cv.calibrateHandEye(
                                                    R_gripper2base = flange_to_robot_base_R_matrix_list,
                                                    t_gripper2base = flange_to_robot_base_t_matrix_list,
                                                    R_target2cam   = R_cam_matrix_list,
                                                    t_target2cam   = t_cam_matrix_list,
                                                    method= cv.CALIB_HAND_EYE_TSAI)

    print(f'R_cam2base=np.array(\n[[{R_cam2flange[0][0]:.3f},{R_cam2flange[0][1]:.3f},{R_cam2flange[0][2]:.3f}],\n' 
                                f'[{R_cam2flange[1][0]:.3f},{R_cam2flange[1][1]:.3f},{R_cam2flange[1][2]:.3f}],\n'
                                f'[{R_cam2flange[2][0]:.3f},{R_cam2flange[2][1]:.3f},{R_cam2flange[2][2]:.3f}]])\n')

    print(f't_cam2base=np.array(\n[[{t_cam2flange[0][0]:.3f}],\n[{t_cam2flange[1][0]:.3f}],\n[{t_cam2flange[2][0]:.3f}]])\n')

    T_cam2flange=R_and_t_to_T(R_cam2flange,t_cam2flange)
    print(f'T_cam2base=np.array(\n[[{T_cam2flange[0][0]:.3f},{T_cam2flange[0][1]:.3f},{T_cam2flange[0][2]:.3f},{T_cam2flange[0][3]:.3f}],\n' 
                                f'[{T_cam2flange[1][0]:.3f},{T_cam2flange[1][1]:.3f},{T_cam2flange[1][2]:.3f},{T_cam2flange[1][3]:.3f}],\n'
                                f'[{T_cam2flange[2][0]:.3f},{T_cam2flange[2][1]:.3f},{T_cam2flange[2][2]:.3f},{T_cam2flange[2][3]:.3f}],\n'
                                f'[{T_cam2flange[3][0]:.3f},{T_cam2flange[3][1]:.3f},{T_cam2flange[3][2]:.3f},{T_cam2flange[3][3]:.3f}]])\n')
    return T_cam2flange
    
def draw_coordinate(T):
    # # 定義旋轉矩陣和位移向量
    # R_cam2base = np.array([[ 0.98892985, -0.09406213, -0.11476094],
    #                        [ 0.0535957 ,  0.94762488, -0.31485645],
    #                        [ 0.13836639,  0.30522025,  0.94217585]])

    # t_cam2base = np.array([-0.08155043, -0.40121386, 0.21272556])
    R_cam2flange,t_cam2flange=T_to_R_and_t(T)
    
    # 將單位從米轉換為毫米
    scale = 1000
    # 基座座標系的原點
    flange_origin = np.array([0, 0, 0])*scale
    cam_origin = t_cam2flange.flatten() * scale
    # 繪製座標系
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 繪製 flange 座標系 
    ax.quiver(flange_origin[0], flange_origin[1], flange_origin[2], 
            1, 0, 0, color='r', label='Flange X', length=0.2*scale)
    ax.quiver(flange_origin[0], flange_origin[1], flange_origin[2], 
            0, 1, 0, color='g', label='Flange Y', length=0.2*scale)
    ax.quiver(flange_origin[0], flange_origin[1], flange_origin[2], 
            0, 0, 1, color='b', label='Flange Z', length=0.2*scale)

    # 繪製 cam 座標系
    # cam_origin = t_cam2flange
    ax.quiver(cam_origin[0], cam_origin[1], cam_origin[2], 
            R_cam2flange[0, 0], R_cam2flange[1, 0], R_cam2flange[2, 0], 
            color='c', label='Cam X', length=0.2*scale)
    ax.quiver(cam_origin[0], cam_origin[1], cam_origin[2], 
            R_cam2flange[0, 1], R_cam2flange[1, 1], R_cam2flange[2, 1], 
            color='m', label='Cam Y', length=0.2*scale)
    ax.quiver(cam_origin[0], cam_origin[1], cam_origin[2], 
            R_cam2flange[0, 2], R_cam2flange[1, 2], R_cam2flange[2, 2], 
            color='y', label='Cam Z', length=0.2*scale)

    # 設定圖形屬性
    ax.set_xlabel('X axis(mm)')
    ax.set_ylabel('Y axis(mm)')
    ax.set_zlabel('Z axis(mm)')
    ax.set_title('Base and Camera Coordinate Systems')
    ax.legend()
    ax.set_xlim([-500, 500])
    ax.set_ylim([-500, 500])
    ax.set_zlim([-500, 500])

    plt.show()


if __name__ == "__main__":
    index_first=0
    index_last=6
    R_cam_matrix_list,t_cam_matrix_list = april_tag_detect(index_first,index_last)
    R_robot_matrix_list,t_robot_matrix_list = get_robot_pose_list(index_first,index_last)
    print(">>> R_robot_matrix_list:", len(R_robot_matrix_list), "R_cam_matrix_list:", len(R_cam_matrix_list))
    print(">>> t_robot_matrix_list:", len(t_robot_matrix_list), "t_cam_matrix_list:", len(t_cam_matrix_list))
    T_cam_to_base = get_cam_to_flange_matrix(R_robot_matrix_list,t_robot_matrix_list,R_cam_matrix_list,t_cam_matrix_list)
    draw_coordinate(T_cam_to_base)
    
#2026/04/08
""" T_cam2base=np.array(
[[0.999,0.028,-0.021,0.017],
[-0.028,1.000,-0.002,-0.045],
[0.021,0.003,1.000,0.089],
[0.000,0.000,0.000,1.000]]) """










   

######################################################################################
# # 檢查轉移矩陣的乘積是否為單位矩陣
# for i in range(len(transformation_matrices_robot_flange)):
#     T_robot_flange = transformation_matrices_robot_flange[i]
#     T_base_flange = transformation_matrices_base_to_flange[i]
    
#     # 計算乘積
#     product = T_robot_flange @ T_base_flange

#     # 檢查是否接近單位矩陣
#     identity_matrix = np.eye(4)  # 4x4 單位矩陣
#     if np.allclose(product, identity_matrix):
#         print(f"Transformation Matrices {i} multiply to identity matrix:\n{product}\n")
#     else:
#         print(f"Transformation Matrices {i} do NOT multiply to identity matrix:\n{product}\n")


# # 輸出轉移矩陣
# for i, T in enumerate(transformation_matrices_robot_flange):
#     print(f"Transformation Matrix {i}:\n{T}\n")







""" 平移向量 (XYZ): [-0.331928 -0.893492  0.119226]
旋轉角度 (RX, RY, RZ): [ -3.63431022 -11.30655766  69.75749912] """


""" 2025/03/28
R_cam2base=
np.array(
[[-0.020,0.905,-0.425],
[0.995,0.059,0.078],
[0.096,-0.422,-0.902]])

t_cam2base=
np.array(
[[0.940],
[-0.139],
[0.623]])

T_cam2base=
np.array(
[[-0.020,0.905,-0.425,0.940],
[0.995,0.059,0.078,-0.139],
[0.096,-0.422,-0.902,0.623],
[0.000,0.000,0.000,1.000]]) """