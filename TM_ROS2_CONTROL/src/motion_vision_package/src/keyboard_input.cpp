/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, PickNik LLC
 * All rights reserved.
 *********************************************************************/

/* Title     : servo_keyboard_input.cpp
 * Project   : moveit2_tutorials
 * Created   : 05/31/2021
 * Author    : Adam Pettinger
 * Modified  : Added Start/Stop Service Clients for TM5-900
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "tm_msgs/srv/send_script.hpp"
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_S 0x73  // Start Servo
#define KEYCODE_T 0x74  // Stop Servo

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "flange";
const std::string BASE_FRAME_ID = "base";

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();
  
private:
  void spin();
  void callServoControl(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client);
  void callTmPvtEnter_or_Exit(int mode,bool exit);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  
  // Service Clients
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;

  std::string frame_to_publish_;
  double joint_vel_cmd_;

  // 在 KeyboardServo 類別的 private 區段新增
  rclcpp::Client<tm_msgs::srv::SendScript>::SharedPtr tm_script_client_;
  int current_mode_; // 0 for Joint, 1 for Cartesian
};





KeyboardServo::KeyboardServo() : frame_to_publish_(BASE_FRAME_ID), joint_vel_cmd_(1.0)
{
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");
  
  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);

  start_client_ = nh_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  stop_client_ = nh_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");

    // 在 KeyboardServo 建構子中新增
  tm_script_client_ = nh_->create_client<tm_msgs::srv::SendScript>("send_script");
  current_mode_ = 0; // 預設為 Joint
}

void KeyboardServo::callServoControl(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client)
{
  if (!client->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Servo service not available");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(request);
}

// 新增此成員函式
void KeyboardServo::callTmPvtEnter_or_Exit(int mode,bool exit=false)
{

  if (!tm_script_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(nh_->get_logger(), "TM Script service not available");
    return;
  }
  auto request = std::make_shared<tm_msgs::srv::SendScript::Request>();
  request->id = "PVT_INIT";

    // 根據 mode 發送指令：0 -> Joint, 1 -> Cartesian
  if (!exit)
  {
    request->script = (mode == 0) ? "PVTEnter(0)" : "PVTEnter(1)";
  }
  else
  {    
    request->script = "PVTExit()";
  }

  
  tm_script_client_->async_send_request(request);
  RCLCPP_INFO(nh_->get_logger(), "Sent TM Script: %s", request->script.c_str());
}




KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;
  signal(SIGINT, quit);
  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();
  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{ std::bind(&KeyboardServo::spin, this) }.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
  puts("Use 1|2|3|4|5|6 keys to joint jog. 'R' to reverse the direction of jogging.");
  puts("Use 'S' to START Servo, 'T' to STOP Servo.");
  puts("'Q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Use read key-press
    switch (c)
    { 
      
      case KEYCODE_LEFT:
        current_mode_ = 1;
        twist_msg->twist.linear.y = -1.0;
        publish_twist = true;
        puts("LEFT");
        break;
      case KEYCODE_RIGHT:
        current_mode_ = 1;
        twist_msg->twist.linear.y = 1.0;
        publish_twist = true;
        puts("RIGHT");
        break;
      case KEYCODE_UP:
        current_mode_ = 1;
        twist_msg->twist.linear.x = 1.0;
        publish_twist = true;
        puts("UP");
        break;
      case KEYCODE_DOWN:
        current_mode_ = 1;
        twist_msg->twist.linear.x = -1.0;
        publish_twist = true;
        puts("DOWN");
        break;
      case KEYCODE_PERIOD:
        current_mode_ = 1;
        twist_msg->twist.linear.z = -1.0;
        publish_twist = true;
        puts("PERIOD");
        break;
      case KEYCODE_SEMICOLON:
        current_mode_ = 1;
        twist_msg->twist.linear.z = 1.0;
        publish_twist = true;
        puts("SEMICOLON");
        break;
      case KEYCODE_E:
        frame_to_publish_ = EEF_FRAME_ID;
        puts("EEF_FRAME (flange)");
        break;
      case KEYCODE_W:
        frame_to_publish_ = BASE_FRAME_ID;
        puts("WORLD_FRAME (base)");
        break;
      case KEYCODE_1:
        current_mode_ = 0;
        joint_msg->joint_names.push_back("joint_1");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        puts("JOINT 1");
        break;
      case KEYCODE_2:
        current_mode_ = 0;
        joint_msg->joint_names.push_back("joint_2");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        puts("JOINT 2");
        break;
      case KEYCODE_3:
        current_mode_ = 0;
        joint_msg->joint_names.push_back("joint_3");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        puts("JOINT 3");
        break;
      case KEYCODE_4:
        current_mode_ = 0;
        joint_msg->joint_names.push_back("joint_4");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        puts("JOINT 4");
        break;
      case KEYCODE_5:
        current_mode_ = 0;
        joint_msg->joint_names.push_back("joint_5");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        puts("JOINT 5");
        break;
      case KEYCODE_6:
        current_mode_ = 0;
        joint_msg->joint_names.push_back("joint_6");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        puts("JOINT 6");
        break;
      case KEYCODE_R:
        joint_vel_cmd_ *= -1;
        puts("JOINT VELOCITY REVERSED");
        break;
      case KEYCODE_S:
        callServoControl(start_client_);
        callTmPvtEnter_or_Exit(current_mode_);
        puts(current_mode_ == 0 ? "SENT: SERVO START (JOINT PVT)" : "SENT: SERVO START (CARTESIAN PVT)");
        puts("SENT: SERVO START");
        break;
      case KEYCODE_T:
        callServoControl(stop_client_);
        callTmPvtEnter_or_Exit(current_mode_, true);
        puts("SENT: SERVO STOP");
        break;
      case KEYCODE_Q:
        puts("QUIT");
        return 0;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = frame_to_publish_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }
  return 0;
}