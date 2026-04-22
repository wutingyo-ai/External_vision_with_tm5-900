#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>
#include <string>

using namespace std::chrono_literals;

// 發送 Script 的函式
bool send_cmd(std::string cmd, std::shared_ptr<rclcpp::Node> node, rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client) {
    auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
    request->id = "DemoManual";
    request->script = cmd;

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "在等待 Service 時被中斷。");
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "等待 Service 中...");
    }

    auto result = client->async_send_request(request);
    
    // 等待回傳結果
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->ok) {
            RCLCPP_INFO(node->get_logger(), "Script 發送成功 [OK]");
            return true;
        } else {
            RCLCPP_WARN(node->get_logger(), "機器人回傳 [Not OK]");
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "呼叫 Service 失敗");
    }
    return false;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("manual_script_sender");
    auto client = node->create_client<tm_msgs::srv::SendScript>("send_script");

    std::cout << "========================================" << std::endl;
    std::cout << " TM Robot Script 手動發送工具" << std::endl;
    std::cout << " 提示: 貼上指令後，在最後一行輸入 '#' 並按 Enter 發送" << std::endl;
    std::cout << " 提示: 直接輸入 'exit' 可結束程式" << std::endl;
    std::cout << "========================================" << std::endl;

    while (rclcpp::ok()) {
        std::string input_script;
        std::string line;
        
        std::cout << "\n請輸入 Script 指令 (結尾輸入 #):\n";
        
        // 讀取多行直到遇到 #
        std::getline(std::cin, input_script, '#');
        
        // 清除緩衝區中的換行符，避免影響下一次讀取
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        // 去除前後空白（選擇性）
        if (input_script.find("exit") != std::string::npos) {
            break;
        }

        if (!input_script.empty()) {
            RCLCPP_INFO(node->get_logger(), "正在發送指令...");
            send_cmd(input_script, node, client);
        }
    }

    RCLCPP_INFO(node->get_logger(), "程式結束。");
    rclcpp::shutdown();
    return 0;
}