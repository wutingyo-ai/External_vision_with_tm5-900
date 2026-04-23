#include "rclcpp/rclcpp.hpp"
#include "tm_msgs/srv/send_script.hpp"
#include "tm_msgs/msg/feedback_state.hpp"
#include <mutex>
#include <string>
#include "tm_driver/tm_command.h"
using namespace std::chrono_literals;



class TMPVTStreamer : public rclcpp::Node {
public:
    TMPVTStreamer() : Node("tm_pvt_streamer") {
        client_ = this->create_client<tm_msgs::srv::SendScript>("send_script");
        sub_ = this->create_subscription<tm_msgs::msg::FeedbackState>(
            "/feedback_states", 10, std::bind(&TMPVTStreamer::feedback_callback, this, std::placeholders::_1));

        // 參數初始化
        current_pos_ = -45.0;
        current_v_ = 0.0;
        target_v_ = 20.0;
        accel_ = 10.0;
        dt_ = 0.1; // 發送週期 200ms
        tag_counter_ = 1;
        state_ = State::START;
        start_time_ = this->get_clock()->now();
        // 定時器：每 200ms 觸發一次
        timer_ = this->create_wall_timer(100ms, std::bind(&TMPVTStreamer::control_loop, this));
    }

private:
    enum class State { START, RUNNING, ENDING, FINISHED };
    
    void control_loop() {
        std::string cmd = "";

        switch (state_) {
            case State::START:
                // 第一步：發送 Enter 並帶入第一個點位
                cmd = "PVTEnter(0)\r\n";
                cmd += calculate_next_step();
                state_ = State::RUNNING;
                break;

            case State::RUNNING:
                // 持續加速或等速段
                cmd = calculate_next_step();
                if (current_v_ >= target_v_) {
                    // 這裡可以加入維持等速的邏輯，範例為達到後準備減速
                    state_ = State::ENDING;
                }
                break;

            case State::ENDING:
                // 減速段
                cmd = calculate_next_step();
                if (current_v_ <= 0.0) {
                    // 最後一步：補上 Exit
                    cmd += "PVTExit()";
                    state_ = State::FINISHED;
                }
                rclcpp::shutdown();
                break;

            case State::FINISHED:
                RCLCPP_INFO(this->get_logger(), "所有指令已完成，手臂應在目標點停止。");
                timer_->cancel();
                return;
        }

        if (!cmd.empty()) {
            send_script(cmd);
        }
    }

    std::string calculate_next_step() {
        double last_v = current_v_;
        
        if(state_==State::START) {
            current_v_ = 0.0; // 從靜止開始
        }
        
        // 簡單梯形加減速邏輯
        if (state_ != State::ENDING) {
            current_v_ = std::min(current_v_ + (accel_ * dt_), target_v_);
        } else {
            current_v_ = std::max(current_v_ - (accel_ * dt_), 0.0);
        }
        
        current_pos_ += (last_v + current_v_) * 0.5 * dt_;
        
        // 紀錄預期時間點
        {
            std::lock_guard<std::mutex> lock(mtx_);
            last_planned_time_ = this->get_clock()->now();
            last_planned_pos_ = current_pos_;
            is_planned_ = true; // 標記已經有規劃值了
        }

        
        char buf[256];
        snprintf(buf, sizeof(buf), "PVTPoint(%.3f,0,90,0,90,0,%.3f,0,0,0,0,0,%.2f)\r\nQueueTag(%d,0)\r\n",
         current_pos_, current_v_, dt_, tag_counter_++);
        if (tag_counter_ > 15) tag_counter_ = 1;

        
        return std::string(buf);
        
    }

    void feedback_callback(const tm_msgs::msg::FeedbackState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        // 這裡就是判斷「程式運行速度與手臂有無對齊」的核心
        rclcpp::Time actual_time = msg->header.stamp;
        double actual_pos = TmCommand::deg(msg->joint_pos[0]);

        // 如果還沒開始規劃，只更新 actual_pos，不印 Log 也不算誤差
        if (!is_planned_) {return; }

        RCLCPP_INFO_THROTTLE(
        this->get_logger(),*this->get_clock(), 200,
        "規劃位置:%.3f | 實際位置:%.3f",last_planned_pos_, actual_pos);

        // 計算時間戳差值 (電腦規劃時間 vs 手步回傳物理時間)
        double time_diff = (this->get_clock()->now() - actual_time).seconds();
        double pos_error = last_planned_pos_ - actual_pos;
        RCLCPP_INFO_THROTTLE(
        this->get_logger(),*this->get_clock(), 200,
        "程式運行經過時間:%.3fs | 手臂位置回傳時間:%.3f",
        (this->get_clock()->now() - start_time_).seconds(), 
        (actual_time-start_time_).seconds()
        );



        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "對齊檢查 -> 延遲: %.3fs | 位置誤差: %.4f", time_diff, pos_error);
    }

    void send_script(const std::string& s) {
        auto req = std::make_shared<tm_msgs::srv::SendScript::Request>();
        req->script = s;
        client_->async_send_request(req);
    }

    // 變數區域
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr client_;
    rclcpp::Subscription<tm_msgs::msg::FeedbackState>::SharedPtr sub_;
    
    State state_;
    double current_pos_, current_v_, target_v_, accel_, dt_;
    int tag_counter_;
    rclcpp::Time start_time_;
    std::mutex mtx_;
    rclcpp::Time last_planned_time_;
    double last_planned_pos_=-45.0;
    bool is_planned_ = false;
};

int main(int argc, char **argv) {
    
    
    rclcpp::init(argc, argv);
    // std::cout<<TmCommand::deg(3.14159)<<'\n';
    auto node = std::make_shared<TMPVTStreamer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}