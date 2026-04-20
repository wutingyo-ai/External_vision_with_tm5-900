#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tm_msgs/srv/send_script.hpp>
#include <iomanip>

class TmPvtBridge : public rclcpp::Node {
public:
  TmPvtBridge() : Node("tm_pvt_bridge") {
    // 1. 訂閱 Servo Node 的輸出
    sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "tmr_arm_controller/follow_joint_trajectory", 10,
      std::bind(&TmPvtBridge::trajectory_callback, this, std::placeholders::_1));

    // 2. 建立發送 Script 的 Client
    client_ = this->create_client<tm_msgs::srv::SendScript>("send_script");
  }

private:
  int time_counter = 0;
  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (msg->points.empty()) return;

    // 取出最新的點位 (通常 Servo 只會發送一個 point)
    auto& point = msg->points[0];
    
    // 格式化 PVTPoint 指令: PVTPoint(pos_deg, vel_deg, time)
    // 注意：TM 的 Script 需要角度 (Degree)，但 ROS 是弧度 (Radian)
    std::stringstream ss;
    ss << std::fixed << std::setprecision(5);
    ss << "PVTPoint(";
    
    // 填入 6 個關節的位置 (rad -> deg)
    for (double pos : point.positions) { ss << pos * 180.0 / M_PI << ","; }
    // 填入 6 個關節的速度 (rad/s -> deg/s)
    for (double vel : point.velocities) { ss << vel * 180.0 / M_PI << ","; }
    
    // 填入時間 (從訊息中取得 duration)
    double t = rclcpp::Duration(point.time_from_start).seconds();
    if (t <= 0) t = 0.01; // 確保時間不為 0
    ss << t << ")";

    // 發送給 TM Driver
    send_to_tm(ss.str());
    puts((ss.str()).c_str());
    // 在類別成員函式中這樣寫：
    RCLCPP_INFO(this->get_logger(), "Count: %d, Data: %s", time_counter++, ss.str().c_str());
  }

  void send_to_tm(const std::string& script) {
    auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
    request->id = "PVTStream";
    request->script = script;
    // 使用非同步發送以確保不會卡住訂閱回呼
    client_->async_send_request(request);
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  // 1. 建立物件實例 (建構)
  auto node = std::make_shared<TmPvtBridge>();
  
  // 2. 開始運行 (呼叫)
  // spin 會讓節點開始監聽 Topic 並在有訊息時執行回呼函式
  puts("TmPvtBridge node started. Listening for trajectory commands...");
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}