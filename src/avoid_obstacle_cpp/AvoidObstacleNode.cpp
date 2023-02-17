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
#include "kobuki_ros_interfaces/msg/wheel_drop_event.hpp"
#include "kobuki_ros_interfaces/msg/led.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "rclcpp/rclcpp.hpp"

namespace avoid_obstacle_cpp
{

using namespace std::chrono_literals;
using std::placeholders::_1;

AvoidObstacleNode::AvoidObstacleNode()
: Node("avoid_obstacle"),
  state_(READY)
{
  button_sub_ = create_subscription<kobuki_ros_interfaces::msg::ButtonEvent>(
    "input_button", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::button_callback, this, _1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "input_scan", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::scan_callback, this, _1));

  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>( // comprobar que sea el mensaje
    "input_bumper", rclcpp::SensorDataQoS(),
    std::bind(&AvoidObstacleNode::bumper_callback, this, _1));

  led_pub_ = create_publisher<kobuki_ros_interfaces::msg::Led>("status_led", 10);
  sound_pub_ = create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  timer_ = create_wall_timer(25ms, std::bind(&AvoidObstacleNode::control_cycle, this));

  state_ts_ = now();
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
AvoidObstacleNode::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg) // comprobar que sea asi
{
  RCLCPP_INFO(get_logger(), "CRASH!");
  last_bumper_event_ = std::move(msg);
}

void
AvoidObstacleNode::control_cycle()
{
  // Do nothing until the first sensor read
  if (last_scan_ == nullptr) {
    return;
  }

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case READY:
      out_led.value = kobuki_ros_interfaces::msg::Led::ORANGE;
      led_pub_->publish(out_led);

      if (!check_ready_2_forward()) {
        break;
      }

      RCLCPP_INFO(get_logger(), "STARTING...");

      out_sound.value = kobuki_ros_interfaces::msg::Sound::ON;
      out_led.value = kobuki_ros_interfaces::msg::Led::GREEN;

      sound_pub_->publish(out_sound);
      led_pub_->publish(out_led);

      RCLCPP_INFO(get_logger(), "READY -> FORWARD");
      go_state(FORWARD);

      break;
    case FORWARD:
      out_vel.linear.x = SPEED_LINEAR;

      if (check_forward_2_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> STOP");
        go_state(STOP);
      }

      if (check_forward_2_yaw()) {
        out_sound.value = kobuki_ros_interfaces::msg::Sound::ON;
        sound_pub_->publish(out_sound);
        RCLCPP_INFO(get_logger(), "FORWARD -> YAW TURN");
        go_state(YAW_TURN_IN);
      }
      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      break;
    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_yaw_turn_in()) {
        out_led.value = kobuki_ros_interfaces::msg::Led::GREEN;
        led_pub_->publish(out_led);
        go_state(YAW_TURN_IN);
      }
      break;
    case YAW_TURN_IN:
      out_vel.angular.z = SPEED_ANGULAR;
      if (check_yaw_2_dodge()) {
        RCLCPP_INFO(get_logger(), "YAW TURN -> DODGE TURN");
        go_state(DODGE_TURN);
      }
      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      break;
    case DODGE_TURN:
      out_vel.angular.z = -SPEED_ANGULAR*0.75;
      out_vel.linear.x = SPEED_LINEAR;

      if (check_dodge_2_yaw_new_obstacle()) {
        out_sound.value = kobuki_ros_interfaces::msg::Sound::RECHARGE;
        sound_pub_->publish(out_sound);
        RCLCPP_INFO(get_logger(), "DODGE TURN -> YAW TURN IN");
        go_state(YAW_TURN_IN);
      }

      if (check_dodge_2_yaw_out()) {
        out_sound.value = kobuki_ros_interfaces::msg::Sound::ON;
        sound_pub_->publish(out_sound);
        RCLCPP_INFO(get_logger(), "DODGE TURN -> YAW TURN OUT");
        go_state(YAW_TURN_OUT);
      }

      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }

      break;
    case YAW_TURN_OUT:
      out_vel.angular.z = SPEED_ANGULAR;
      if (check_yaw_out_2_forward()) {
        RCLCPP_INFO(get_logger(), "YAW TURN OUT -> FORWARD");
        go_state(FORWARD);
      }

      if (check_any_2_emergency_stop()) {
        RCLCPP_INFO(get_logger(), "FORWARD -> EMERGENCY STOP");
        go_state(EMERGENCY_STOP);
      }
      break;
    case STOP:
      if (check_stop_2_forward()) {
        RCLCPP_INFO(get_logger(), "STOP -> FORWARD");
        go_state(FORWARD);
      }
      break;
    case EMERGENCY_STOP:
      out_vel.linear.x = 0;
      out_vel.angular.z = 0;
      if (check_emergency_2_back()) {
        out_led.value = kobuki_ros_interfaces::msg::Led::RED;
        led_pub_->publish(out_led);
        out_sound.value = kobuki_ros_interfaces::msg::Sound::RECHARGE;
        sound_pub_->publish(out_sound);
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
  state_ts_ = now();
  RCLCPP_INFO(get_logger(), "CHANGED STATE");
}

bool
AvoidObstacleNode::check_ready_2_forward()
{
  return last_button_event_ != nullptr;
}

bool
AvoidObstacleNode::check_forward_2_yaw()
{
  // going forward when deteting an obstacle
  // at 1 meters with the front laser read
  size_t pos = 0;
  return last_scan_->ranges[pos] < OBSTACLE_DISTANCE;
}

bool
AvoidObstacleNode::check_forward_2_stop()
{
  // Stop if no sensor readings for 1 second
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
}

bool
AvoidObstacleNode::check_stop_2_forward()
{
  // Going forward if sensor readings are available
  // again
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}

bool
AvoidObstacleNode::check_yaw_2_dodge()
{
  // Turning for 7.5 seconds
  return (now() - state_ts_ ) > YAW_TIME;
}

bool
AvoidObstacleNode::check_dodge_2_yaw_new_obstacle()
{
  return check_forward_2_yaw();
}

bool
AvoidObstacleNode::check_dodge_2_yaw_out()
{
  // Turning for 10 seconds
  return (now() - state_ts_ ) > DODGE_TIME;
}

bool
AvoidObstacleNode::check_yaw_out_2_forward()
{
  return check_yaw_2_dodge();
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
  // Turning for 10 seconds
  return (now() - state_ts_ ) > BACK_TIME;
}

} // namespace avoid_obstacle_cpp