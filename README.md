# 機械臂手眼配合功能包 motion_vision_package

## 手眼校正程式目錄 motion_vision_package/scripts/eye_hand_calibration/..
> 取像與紀錄手臂姿態程式
>- lips_camera_calibration_hand_eye.py:取apyriltag影像，此程式以下代號為A
>- Get_Flange_and_Tool_Pose.py:取機械手臂法蘭面姿態 6dof xyz單位mm，rxryrz單位deg，此程式以下代號為B
>-  Send_simple_msg.py:發送字串命令用的節點，此程式以下代號為C
>- camera_calibration_eye_in_hand_new.py:求解法蘭面座標系到相機座標系轉移矩陣關係，此程式以下代號為D

### 啟用方式 
>1. 確認網路有pin到達明手臂就可，不需要進listen node
>2. 編譯完整個motion_vision_package後使用ros2 run 同時開啟A、B、C。
>3. C程式會需要輸入字串，擺好apriltag移動手臂使影像映入相機後，就可輸入"ok"並按enter發送，A、B程式便會紀錄在motion_vision_package/tag_detect_image、motion_vision_package/eye_hand_calibration_data之中。

### !注意
>- april tag的姿態要有適當平移與旋轉，但也不可過於奇異，求出之解才會準，否則誤差會很大。
>- eye to hand 部分只有流程邏輯，細部細節尚未修改
>- AE相機連線部分需設定

# 機械手臂伺服運動方法
> 透過達明腳本語言 Script Language 的PVT(Position-Velocity-Time)模式控制路徑軌跡
> ros2中使用send script來做發送