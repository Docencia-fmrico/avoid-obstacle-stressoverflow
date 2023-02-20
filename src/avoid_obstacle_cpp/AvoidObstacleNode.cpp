// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <utility>
#include <chrono>
#include "avoid_obstacle_cpp/AvoidObstacleNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki_ros_interfaces/msg/button_event.hpp"
#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
// #include "kobuki_ros_interfaces/msg/wheel_drop_event.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "rclcpp/rclcpp.hpp"

namespace avoid_obstacle_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

AvoidObstacleNode::AvoidObstacleNode()
: Node("avoid_obstacle"),
  state_(INNIT)
{
  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::button_callback, this, _1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::scan_callback, this, _1));

  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "input_bumper", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::bumper_callback, this, _1));

  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("status_led", 10);
  sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(25ms, std::bind(&AvoidObstacleNode::control_cycle, this));

  state_ts_ = now();

  declare_parameter<double>("yaw_time", 7.5);
  declare_parameter<double>("back_time", 2.5);
  declare_parameter<double>("dodge_time", 20);
  declare_parameter<double>("scan_timeout", 1);

  declare_parameter<float>("speed_linear", 0.1f);
  declare_parameter<float>("speed_angular", 0.3f);
  declare_parameter<float>("obstacle_distance", 1.0f);

  get_parameter("yaw_time", yaw_time_);
  get_parameter("back_time", back_time_);
  get_parameter("dodge_time", dodge_time_);
  get_parameter("scan_timeout", scan_timeout_);

  get_parameter("speed_linear", speed_linear_);
  get_parameter("speed_angular", speed_angular_);
  get_parameter("obstacle_distance", obstacle_distance_);
}

void
AvoidObstacleNode::button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg)
{
  last_button_event_ = std::move(msg);
}

void
AvoidObstacleNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  last_scan_ = std::move(msg);
}

void
AvoidObstacleNode::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  last_bumper_event_ = std::move(msg);
}

void
AvoidObstacleNode::control_cycle()
{
  /*
   * Do nothing until the first sensor read
   */
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case INNIT:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      RCLCPP_INFO(get_logger(), "INNIT -> READY");
      go_state(READY);
      break;
    case READY:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      if (check_ready_2_forward()) {
        last_button_event_ = nullptr;
        RCLCPP_INFO(get_logger(), "READY -> FORWARD");
        go_state(FORWARD);
      }
      break;
    case FORWARD:
      out_vel.linear.x = speed_linear_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_forward_2_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> STOP");
        go_state(STOP);
      }
      if (check_forward_2_yaw_turn_in()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> YAW TURN IN");
        go_state(YAW_TURN_IN);
      }
      break;
    case BACK:
      out_vel.linear.x = -speed_linear_;

      if (check_back_2_yaw_turn_in()) {
        RCLCPP_INFO(get_logger(), "BACK -> YAW TURN IN");
        go_state(YAW_TURN_IN);
      }
      break;
    case YAW_TURN_IN:
      out_vel.angular.z = speed_angular_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "YAW TURN IN -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_yaw_turn_in_2_dodge_turn()) {
        RCLCPP_INFO(get_logger(), "YAW TURN IN -> DODGE TURN");
        go_state(DODGE_TURN);
      }
      break;
    case DODGE_TURN:
      out_vel.angular.z = -speed_angular_*0.75;
      out_vel.linear.x = speed_linear_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_dodge_2_yaw_new_obstacle()) {
        RCLCPP_INFO(get_logger(), "DODGE TURN -> YAW TURN IN");
        go_state(YAW_TURN_IN);
      }
      if (check_dodge_2_yaw_turn_out()) {
        RCLCPP_INFO(get_logger(), "DODGE TURN -> YAW TURN OUT");
        go_state(YAW_TURN_OUT);
      }
      break;
    case YAW_TURN_OUT:
      out_vel.angular.z = speed_angular_;

      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "YAW TURN OUT -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      if (check_yaw_turn_out_2_forward()) {
        RCLCPP_INFO(get_logger(), "YAW TURN OUT -> FORWARD");
        go_state(FORWARD);
      }
      break;
    case STOP:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      if (check_stop_2_forward()) {
        RCLCPP_INFO(get_logger(), "STOP -> FORWARD");
        go_state(FORWARD);
      }
      break;
    case EMERGENCY_STOP:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;

      if (check_emergency_2_back()) {
        last_bumper_event_ = nullptr;
        RCLCPP_INFO(get_logger(), "EMERGENCY STOP -> BACK");
        go_state(BACK);
      }
      break;
  }

  vel_pub_->publish(out_vel);
}

void
AvoidObstacleNode::go_state(int new_state)
{
  state_ = new_state;
  manage_user_feedback(new_state);
  state_ts_ = now();
  RCLCPP_INFO(get_logger(), "CHANGED STATE");
}

void
AvoidObstacleNode::manage_user_feedback(int new_state)
{
  change_status_led(new_state);
  change_status_sound(new_state);
}

void
AvoidObstacleNode::change_status_led(int new_state)
{
  kobuki_ros_interfaces::msg::Led out_led;

  switch (new_state)
  {
    case READY:
      out_led.value = kobuki_ros_interfaces::msg::Led::ORANGE;
      break;
    case BACK:
    case STOP:
    case EMERGENCY_STOP:
      out_led.value = kobuki_ros_interfaces::msg::Led::RED;
      break;
    case FORWARD:
    case YAW_TURN_IN:
    case YAW_TURN_OUT:
    case DODGE_TURN:
      out_led.value = kobuki_ros_interfaces::msg::Led::GREEN;
      break;
    
    default:
      return;
  }

  led_pub_->publish(out_led);
}

void
AvoidObstacleNode::change_status_sound(int new_state)
{
  kobuki_ros_interfaces::msg::Sound out_sound;

  switch (new_state)
  {
    case READY:
    case FORWARD:
    case YAW_TURN_IN:
    case YAW_TURN_OUT:
    case DODGE_TURN:
      out_sound.value = kobuki_ros_interfaces::msg::Sound::ON;
      break;
    case BACK:
    case STOP:
    case EMERGENCY_STOP:
      out_sound.value = kobuki_ros_interfaces::msg::Sound::RECHARGE;
      break;
    
    default:
      return;
  }

  sound_pub_->publish(out_sound);
}

bool
AvoidObstacleNode::check_ready_2_forward()
{
  return last_button_event_ != nullptr;
}

bool
AvoidObstacleNode::check_forward_2_yaw_turn_in()
{
  size_t pos = 0;
  return last_scan_->ranges[pos] < obstacle_distance_;
}

bool
AvoidObstacleNode::check_forward_2_stop()
{
  /* 
   * Stop if no sensor readings for 1 second
   */
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > rclcpp::Duration::from_seconds(scan_timeout_);
}

bool
AvoidObstacleNode::check_stop_2_forward()
{
  /* 
   * Going forward if sensor readings are available again
   */
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < rclcpp::Duration::from_seconds(scan_timeout_);
}

bool
AvoidObstacleNode::check_yaw_turn_in_2_dodge_turn()
{
  return (now() - state_ts_ ) > rclcpp::Duration::from_seconds(yaw_time_);
}

bool
AvoidObstacleNode::check_dodge_2_yaw_new_obstacle()
{
  return check_forward_2_yaw_turn_in();
}

bool
AvoidObstacleNode::check_dodge_2_yaw_turn_out()
{
  return (now() - state_ts_ ) > rclcpp::Duration::from_seconds(dodge_time_);
}

bool
AvoidObstacleNode::check_yaw_turn_out_2_forward()
{
  return check_yaw_turn_in_2_dodge_turn();
}

bool
AvoidObstacleNode::check_any_2_emergency_stop()
{
  return last_bumper_event_ != nullptr;
}

bool
AvoidObstacleNode::check_emergency_2_back()
{
  return last_bumper_event_ == nullptr;
}

bool
AvoidObstacleNode::check_back_2_yaw_turn_in()
{
  return (now() - state_ts_ ) > rclcpp::Duration::from_seconds(back_time_);
}

} // namespace avoid_obstacle_cpp