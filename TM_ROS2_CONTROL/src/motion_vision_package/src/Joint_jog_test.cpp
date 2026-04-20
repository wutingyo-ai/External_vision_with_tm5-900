#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include <chrono>
#include <thread>
#include "std_srvs/srv/trigger.hpp" // Servo 的 Start/Stop 使用 Trigger 類型
#include "tm_msgs/srv/send_script.hpp"

std::string Enter_Exit(bool exit=false)
{
  if (!exit)
  {
    return "PVTEnter(0)";
  }
  else
  {
    return "PVTExit()";
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_jog_test_node");
  
  // 注意：Topic 通常是 /servo_node/delta_joint_cmds
  auto publisher = node->create_publisher<control_msgs::msg::JointJog>("servo_node/delta_joint_cmds", 10);
  auto start_client = node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  auto stop_client = node->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
  auto tm_script_client_ = node->create_client<tm_msgs::srv::SendScript>("send_script");
  double current_v = 0.0;
  double target_v = 1.0;  // 目標速度 1.0 rad/s
  double accel = 0.5;     // 加速度 0.5 rad/s^2
  double dt = 0.02;       // 20ms 週期
  double step = accel * dt;

  // 初始化訊息
  auto msg = control_msgs::msg::JointJog();
  msg.joint_names.push_back("joint_1"); // 測試一號關節
  msg.velocities.push_back(0.0);

  RCLCPP_INFO(node->get_logger(), "開始 Joint 1 平滑測試...");
  // 建立 Service Client

  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "PVTEnterExit";

    // 根據 mode 發送指令：0 -> Joint, 1 -> Cartesian
  request->script=Enter_Exit();

  
  tm_script_client_->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Sent TM Script: %s", request->script.c_str());
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // 等待 Service 上線
  while (!start_client->wait_for_service(std::chrono::seconds(1))) {
  if (!rclcpp::ok()) return 0;
  RCLCPP_INFO(node->get_logger(), "等待 Servo Start 服務中...");
  }

  

  // --- 啟動 Servo ---
  auto request_servo = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto result_future = start_client->async_send_request(request_servo);

  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Servo 啟動成功！");
  } else {
      RCLCPP_ERROR(node->get_logger(), "無法啟動 Servo，測試終止。");
      return 0;
  }

  int count = 0;
  // 1. 加速段
  while (current_v < target_v && rclcpp::ok()) {
    current_v = std::min(current_v + step, target_v);
    msg.header.stamp = node->now();
    msg.velocities[0] = current_v;
    publisher->publish(msg);
    std::cout << "加速中: " << count++ << " - 速度: " << current_v << " rad/s" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // 2. 恆速段 (維持 1 秒)
  for (int i = 0; i < 50; ++i) {
    msg.header.stamp = node->now();
    msg.velocities[0] = target_v;
    publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // 3. 減速段
  while (current_v > 0.0 && rclcpp::ok()) {
    current_v = std::max(current_v - step, 0.0);
    msg.header.stamp = node->now();
    msg.velocities[0] = current_v;
    publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(node->get_logger(), "測試完成，發送停止指令");
  
  // 最後發送一筆 0 速度確保停止
  msg.velocities[0] = 0.0;
  publisher->publish(msg);

    request->id = "PVTEnterExit";



  request->script=Enter_Exit(true);
  tm_script_client_->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Sent TM Script: %s", request->script.c_str());

  // --- 停止 Servo ---
  RCLCPP_INFO(node->get_logger(), "正在停止 Servo...");
  auto stop_result_future = stop_client->async_send_request(request_servo);
  rclcpp::spin_until_future_complete(node, stop_result_future);
  RCLCPP_INFO(node->get_logger(), "Servo 已安全停止。");
  rclcpp::shutdown();
  return 0;
}