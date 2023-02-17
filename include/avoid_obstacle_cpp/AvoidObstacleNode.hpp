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

#ifndef AVOID_OBSTACLE_CPP__AVOIDOBSTACLENODE_HPP_
#define AVOID_OBSTACLE_CPP__AVOIDOBSTACLENODE_HPP_

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

using namespace std::chrono_literals;  // NOLINT

class AvoidObstacleNode : public rclcpp::Node
{
public:
  AvoidObstacleNode();

private:
  void button_callback(kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr msg);
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);
  void control_cycle();

  static const int READY = 1;
  static const int FORWARD = 2;
  static const int BACK = 3;
  static const int YAW_TURN_IN = 4;
  static const int YAW_TURN_OUT = 5;
  static const int DODGE_TURN = 6;
  static const int STOP = 7;
  static const int EMERGENCY_STOP = 8;
  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_ready_2_forward();
  bool check_forward_2_yaw();
  bool check_forward_2_stop();
  bool check_stop_2_forward();
  bool check_yaw_2_dodge();
  bool check_dodge_2_yaw_new_obstacle();
  bool check_dodge_2_yaw_out();
  bool check_yaw_out_2_forward();
  bool check_any_2_emergency_stop();
  bool check_emergency_2_back();
  bool check_back_2_yaw_turn_in();

  const rclcpp::Duration YAW_TIME {7.5s};
  const rclcpp::Duration BACK_TIME {2.5s};
  const rclcpp::Duration DODGE_TIME {20s};
  const rclcpp::Duration SCAN_TIMEOUT {1s};

  kobuki_ros_interfaces::msg::Sound out_sound;
  kobuki_ros_interfaces::msg::Led out_led;

  static constexpr float SPEED_LINEAR = 0.1f;
  static constexpr float SPEED_ANGULAR = 0.3f;
  static constexpr float OBSTACLE_DISTANCE = 1.0f;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Led>::SharedPtr led_pub_;
  rclcpp::Publisher<kobuki_ros_interfaces::msg::Sound>::SharedPtr sound_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::ButtonEvent>::SharedPtr button_sub_;
  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
  kobuki_ros_interfaces::msg::ButtonEvent::UniquePtr last_button_event_;
  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bumper_event_;
};

}  // namespace avoid_obstacle_cpp

#endif  // AVOID_OBSTACLE_CPP__AVOIDOBSTACLENODE_HPP_
