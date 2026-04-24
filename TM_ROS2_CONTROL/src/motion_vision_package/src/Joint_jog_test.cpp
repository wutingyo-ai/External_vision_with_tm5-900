#include "rclcpp/rclcpp.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include <chrono>
#include <thread>
#include "std_srvs/srv/trigger.hpp" // Servo 的 Start/Stop 使用 Trigger 類型
#include "tm_msgs/srv/send_script.hpp"

#include <cstdlib>
#include <memory>
using namespace std::chrono_literals;
std::string Enter_Exit(bool exit = false)
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

double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
} 
double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}


bool send_cmd(std::string cmd, std::shared_ptr<rclcpp::Node> node, rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client)
{
  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "demo";
  request->script = cmd;
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->ok)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "OK");
    }
    else
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "not OK");
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  return true;
}

std::string format_pvt_point(double j1_pos, double j1_vel, double time_from_start)
{
  
  // 格式化 PVTPoint 指令: PVTPoint(pos_deg, vel_deg, time)
  // 注意：TM 的 Script 需要角度 (Degree)，但 ROS 是弧度 (Radian)
  std::stringstream ss;
  ss << std::fixed << std::setprecision(5);
  ss << "PVTPoint(";
  ss << j1_pos << "," << "0.0" << "," << "90.0" << "," << "0.0" << "," << "90.0" << "," << "0.0" << ",";
  ss << j1_vel << "," << "0.0" << "," << "0.0" << "," << "0.0" << "," << "0.0" << "," << "0.0" << ",";
  ss << time_from_start << ")";
  return ss.str();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_jog_test_node");

  // 注意：Topic 通常是 /servo_node/delta_joint_cmds
  auto publisher = node->create_publisher<control_msgs::msg::JointJog>("servo_node/delta_joint_cmds", 10);
  auto start_client = node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  auto stop_client = node->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");
  auto tm_script_client_ = node->create_client<tm_msgs::srv::SendScript>("send_script");
  
  double current_v = 0.0;
  double target_v = deg2rad(20); // 20 deg/s 轉換為 rad/s
  double accel = deg2rad(10);     
  double dt = 0.1;       
  double step = accel * dt;

  // 初始化訊息
  auto msg = control_msgs::msg::JointJog();
  msg.joint_names.push_back("joint_1"); // 測試一號關節
  msg.velocities.push_back(0.0);

  RCLCPP_INFO(node->get_logger(), "開始 Joint 1 平滑測試...");
  // 建立 Service Client

  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "PVTmode";

  // 根據 mode 發送指令：0 -> Joint, 1 -> Cartesian
  request->script = Enter_Exit();
  while (!tm_script_client_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto future_result = tm_script_client_->async_send_request(request);
  // 這裡的 shared_from_this() 就是指定「自己」這個 Node 來處理等待過程中的通訊
  if (rclcpp::spin_until_future_complete(node->shared_from_this(), future_result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = future_result.get();
    if (response->ok)
    {
      RCLCPP_INFO(node->get_logger(), "Successfully executed: %s", request->script.c_str());
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "TM Robot rejected the script: %s", request->script.c_str());
    }
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Service call failed or timed out.");
  }


  // double current_pos = -45.0;  // 累積位移 (deg)
  // double current_v = 0.0;   // 當前速度 (rad/s)
  // double target_v = 20;    // 目標速度
  // double accel = 10;       // 加速度
  // double dt = 0.2;          // 週期 (100ms)
  // int mode = 0;             // 0: Joint, 1: Cartesian
  // std::string zero_pv = "PVTPoint(-45,0.0,90.0,0.0,90.0,0.0,0.0,0.0,0,0,0,0,0.2)";
  // // 1. 進入 PVT 模式
  // send_cmd("PVTEnter(0)", node, tm_script_client_);

  // // 2. 預留 5 組初始速度為 0 的 buffer (維持在原位)
  // RCLCPP_INFO(node->get_logger(), "初始化 5 組 PVT Buffer...");
  // for (int i = 0; i < 5; ++i)
  // {
  //   send_cmd(zero_pv, node, tm_script_client_);
  //   std::this_thread::sleep_for(200ms);
  // }

  // // 3. 加速段 (使用梯形累加位移)
  // RCLCPP_INFO(node->get_logger(), "開始加速...");
  // while (current_v < target_v)
  // {
  //   double last_v = current_v;
  //   current_v = std::min(current_v + (accel * dt), target_v);

  //   // 梯形公式：s = s_old + (v_old + v_new) / 2 * dt
  //   current_pos += (last_v + current_v) * 0.5 * dt;
   
  //   send_cmd( 
  //   format_pvt_point(current_pos , current_v , dt), 
  //   node,
  //   tm_script_client_);

  //   std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
  // }

  // // 4. 恆速段
  // RCLCPP_INFO(node->get_logger(), "恆速運作...");
  // for (int i = 0; i < 20; ++i)
  // {
  //   double last_v = current_v;            // 雖然 v 不變，但維持公式一致性
  //   current_pos += (last_v + current_v) * 0.5 * dt; // 恆速下 (v+v)/2 仍為 v

  //   send_cmd(
  //     format_pvt_point(current_pos , current_v , dt),
  //     node, 
  //     tm_script_client_);

  //   std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
  // }

  // // 5. 減速段
  // RCLCPP_INFO(node->get_logger(), "減速停止中...");
  // while (current_v > 0.0)
  // {
  //   double last_v = current_v;
  //   current_v = std::max(current_v - (accel * dt), 0.0);

  //   // 梯形公式
  //   current_pos += (last_v + current_v) * 0.5 * dt;

  //   send_cmd(format_pvt_point(current_pos , current_v , dt), node, tm_script_client_);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt * 1000)));
  // }

  // // 6. 離開 PVT 模式
  // send_cmd("PVTExit()",node,tm_script_client_);
  // RCLCPP_INFO(node->get_logger(), "測試完成");

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
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
    std::cout << "加速中: " << count++ << " - 速度: " << current_v << " deg/s" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 2. 恆速段 (維持 1 秒)
  for (int i = 0; i < 50; ++i) {
    msg.header.stamp = node->now();
    msg.velocities[0] = target_v;
    publisher->publish(msg);
    std::cout << "恆速中: " << i << " - 速度: " << target_v << " rad/s" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 3. 減速段
  while (current_v > 0.0 && rclcpp::ok()) {
    current_v = std::max(current_v - step, 0.0);
    msg.header.stamp = node->now();
    msg.velocities[0] = current_v;
    publisher->publish(msg);
    std::cout << "減速中: " << count++ << " - 速度: " << current_v << " rad/s" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(node->get_logger(), "測試完成，發送停止指令");

  // 最後發送一筆 0 速度確保停止
  msg.velocities[0] = 0.0;
  publisher->publish(msg);

  request->id = "PVTEnterExit";

  request->script = Enter_Exit(true);
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