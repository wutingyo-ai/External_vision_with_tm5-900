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
find_package(std_srvs REQUIRED)
find_package(tm_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# find_package(tm_driver REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


include_directories(include
  # ${CMAKE_CURRENT_SOURCE_DIR}
  /home/dennis_wu/my_workspace/src/tmr_ros2-humble/tm_driver/include
)

#
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
  std_srvs
  tm_msgs
)

add_executable(tm_pvt_bridge src/tm_pvt_bridge.cpp)
# 將所有 ROS 2 相關的功能包一次性掛載
ament_target_dependencies(
  tm_pvt_bridge
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  control_msgs
  std_srvs
  tm_msgs
  trajectory_msgs
)

add_executable(Joint_jog_test src/Joint_jog_test.cpp)
# 將所有 ROS 2 相關的功能包一次性掛載
ament_target_dependencies(
  Joint_jog_test
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  control_msgs
  std_srvs
  tm_msgs
)

add_executable(manual_send_script src/manual_send_script.cpp)
# 將所有 ROS 2 相關的功能包一次性掛載
ament_target_dependencies(
  manual_send_script
  rclcpp
  geometry_msgs
  std_srvs
  tm_msgs
)

add_executable(pvt_synchronization src/pvt_synchronization.cpp
/home/dennis_wu/my_workspace/src/tmr_ros2-humble/tm_driver/src/tm_command.cpp)
ament_target_dependencies(
  pvt_synchronization
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2_eigen
  control_msgs
  std_srvs
  tm_msgs
  # tm_driver
)




# 1. 整合 C++ 執行檔 (Targets)
install(TARGETS 
  moveit_cpp_tutorial
  keyboard_input
  tm_pvt_bridge
  Joint_jog_test
  manual_send_script
  pvt_synchronization
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

ament_package()
