#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

bool send_cmd(std::string cmd, std::shared_ptr<rclcpp::Node> node, rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client) {
  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "demo";
  request->script = cmd;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return false;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->ok) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "OK");
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "not OK");
    }
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_send_script");
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client =
    node->create_client<tm_msgs::srv::SendScript>("send_script");
  
  std::string cmd = "PTP(\"JPP\",0,0,90,0,90,0,35,200,0,false)\r\n"
                    "QueueTag(1)\r\n"
                    "PTP(\"JPP\",0,0,70,0,90,0,35,200,0,false)\r\n"
                    "QueueTag(2)\r\n";
  // double s0=-45.0 ,v0=0.0, a1=20.0,t=2.0,dt=0.2;

  


// std::string newcmd = 
//     "PVTEnter(1)\r\n"
//     "PVTPoint(467.5,-122.2,359.7,180,0,90,50,50,0,0,0,0,0.5)\r\n"
//     "QueueTag(1,0)\r\n"
//     "PVTPoint(467.5,-72.2,359.7,180,0,90,-50,50,0,0,0,0,0.5)\r\n"
//     "QueueTag(2,0)\r\n"
//     "PVTPoint(417.5,-72.2,359.7,180,0,90,0,0,0,0,0,0,0.5)\r\n"
//     "QueueTag(3,0)\r\n"
//     "PVTPoint(417.5,-122.2,359.7,180,0,90,50,50,0,0,0,0,0.5)\r\n"
//     "QueueTag(4,0)\r\n"
//     "PVTPoint(417.5,-122.2,359.7,180,0,60,50,50,0,0,0,0,3.0)\r\n"
//     "QueueTag(5,0)\r\n"
//     "PVTPoint(417.5,-122.2,359.7,180,0,90,50,50,0,0,0,0,3.0)\r\n"
//     "QueueTag(6,0)\r\n"
//     "PVTExit()";

std::string newcmd1 = 
    "PVTEnter(0)\r\n"
    "PVTPoint(-45,0,90.0,0,90,0,0,0,0,0,0,0,0.2)\r\n"
    "QueueTag(1,1)\r\n"
    "PVTPoint(-44.6,0,90,0,90,0,4,0,0,0,0,0,0.2)\r\n"
    "QueueTag(2,1)\r\n";
std::string newcmd2=
     "PVTPoint(-43.4,0,90,0,90,0,8,0,0,0,0,0,0.2)\r\n"
      "QueueTag(3,1)\r\n"
      "PVTPoint(-41.4,0,90,0,90,0,12,0,0,0,0,0,0.2)\r\n"
      "QueueTag(4,1)\r\n";

std::string newcmd3=
      "PVTPoint(-38.6,0,90,0,90,0,16,0,0,0,0,0,0.2)\r\n"
      "QueueTag(5,1)\r\n"
      "PVTPoint(-35,0,90,0,90,0,20,0,0,0,0,0,0.2)\r\n"
      "QueueTag(6,1)\r\n";     



std::string newcmd4=
    "PVTPoint(-31.4,0,90,0,90,0,16,0,0,0,0,0,0.2)\r\n"
    "QueueTag(7,1)\r\n"
    "PVTPoint(-28.6,0,90,0,90,0,12,0,0,0,0,0,0.2)\r\n"
    "QueueTag(8,1)\r\n";
    
std::string newcmd5=
    "PVTPoint(-26.6,0,90,0,90,0,8,0,0,0,0,0,0.2)\r\n"
    "QueueTag(9,1)\r\n"
    "PVTPoint(-25.4,0,90,0,90,0,4,0,0,0,0,0,0.2)\r\n"
    "QueueTag(10,1)\r\n";

std::string newcmd6=
    "PVTPoint(-25,0,90,0,90,0,4,0,0,0,0,0,0.2)\r\n"
    "QueueTag(11,1)\r\n"
    "PVTExit()";


  send_cmd(newcmd1, node, client);
  // std::this_thread::sleep_for(400ms); // 等待 TM Robot 處理指令，確保緩衝區有序執行
  send_cmd(newcmd2, node, client);
  // std::this_thread::sleep_for(400ms); // 等待 TM Robot 處理指令，確保緩衝區有序執行
  send_cmd(newcmd3, node, client);
  // std::this_thread::sleep_for(400ms); // 等待 TM Robot 處理指令，確保緩衝區有序執行
  send_cmd(newcmd4, node, client);
  // std::this_thread::sleep_for(400ms); // 等待 TM Robot 處理指令，確保緩衝區有序執行
  send_cmd(newcmd5, node, client);
  // std::this_thread::sleep_for(400ms); // 等待 TM Robot 處理指令，確保緩衝區有序執行
  send_cmd(newcmd6, node, client);
  // std::this_thread::sleep_for(400ms); // 等待 TM Robot 處理指令，確保緩衝區有序執行

  rclcpp::shutdown();
  return 0;
}
