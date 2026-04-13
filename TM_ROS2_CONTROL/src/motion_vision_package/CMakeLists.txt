cmake_minimum_required(VERSION 3.8)
project(motion_vision_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED) # 這是我們之前聊過的數學庫
find_package(control_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



# 1. node1: TM_control_node (C++)
add_executable(TM_control_node src/TM_control_node.cpp)
target_include_directories(TM_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(TM_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# 2. node2: moveit_cpp_tutorial (C++)
add_executable(moveit_cpp_tutorial src/moveit_cpp_tutorial.cpp)
# 將所有 ROS 2 相關的功能包一次性掛載
ament_target_dependencies(
  moveit_cpp_tutorial
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
)

add_executable(keyboard_input src/keyboard_input.cpp)
# 將所有 ROS 2 相關的功能包一次性掛載
ament_target_dependencies(
  keyboard_input
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  control_msgs
)


# 1. 整合 C++ 執行檔 (Targets)
install(TARGETS 
  TM_control_node
  moveit_cpp_tutorial
  keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)

# 2. 整合 Python 腳本 (Programs)
install(PROGRAMS
  scripts/robot_move.py
  scripts/lips_camera_calibration_hand_eye.py
  scripts/Send_simple_msg.py
  scripts/Get_Flange_and_Tool_Pose.py
  DESTINATION lib/${PROJECT_NAME}
)
# 安裝 launch 檔
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# # 4. 安裝設定 (讓 ros2 run 能找到它)

# install(TARGETS TM_control_node
#   DESTINATION lib/${PROJECT_NAME})
# install(TARGETS
#   moveit_cpp_tutorial
#   DESTINATION lib/${PROJECT_NAME}
# )


# # ===== Python node =====
# install(PROGRAMS
#   scripts/robot_move.py
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(PROGRAMS
#   scripts/lips_camera_calibration_hand_eye.py
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(PROGRAMS
#   scripts/Send_simple_msg.py
#   DESTINATION lib/${PROJECT_NAME}
# )
# install(PROGRAMS
#   scripts/Get_Flange_and_Tool_Pose.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
