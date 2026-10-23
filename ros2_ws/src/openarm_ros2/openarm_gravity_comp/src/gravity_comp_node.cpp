// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "openarm_gravity_comp/arm_dynamics.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace openarm_gravity_comp
{
namespace
{

constexpr std::size_t kArmDof = 7;
constexpr double kMaxGravityScale = 1.2;
const std::array<double, kArmDof> kDefaultTorqueLimits = {
  20.0, 20.0, 7.0, 7.0, 2.0, 2.0, 2.0};

}  // namespace

class GravityCompNode : public rclcpp::Node
{
public:
  GravityCompNode()
  : Node("openarm_gravity_comp")
  {
    declare_parameter<std::string>("robot_description", "");
    declare_parameter<bool>("enabled", false);
    declare_parameter<bool>("apply_effort", false);
    declare_parameter<bool>("enable_left", true);
    declare_parameter<bool>("enable_right", true);
    declare_parameter<std::string>("left_base_link", "openarm_body_link0");
    declare_parameter<std::string>("right_base_link", "openarm_body_link0");
    declare_parameter<bool>("verbose", false);
    declare_parameter<double>("gravity_scale", 0.2);
    declare_parameter<double>("ramp_duration_s", 3.0);
    declare_parameter<double>("state_timeout_s", 0.1);
    declare_parameter<double>("publish_rate_hz", 100.0);
    declare_parameter<std::vector<double>>(
      "torque_limits", std::vector<double>(
        kDefaultTorqueLimits.begin(),
        kDefaultTorqueLimits.end()));

    enabled_ = get_parameter("enabled").as_bool();
    apply_effort_ = get_parameter("apply_effort").as_bool();
    enable_left_ = get_parameter("enable_left").as_bool();
    enable_right_ = get_parameter("enable_right").as_bool();
    left_base_link_ = get_parameter("left_base_link").as_string();
    right_base_link_ = get_parameter("right_base_link").as_string();
    verbose_ = get_parameter("verbose").as_bool();
    gravity_scale_ = get_parameter("gravity_scale").as_double();
    ramp_duration_s_ = get_parameter("ramp_duration_s").as_double();
    state_timeout_s_ = get_parameter("state_timeout_s").as_double();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    torque_limits_ = get_parameter("torque_limits").as_double_array();

    validate_startup_parameters();

    const auto robot_description =
      get_parameter("robot_description").as_string();
    if (robot_description.empty()) {
      throw std::runtime_error("robot_description parameter is empty");
    }

    if (enable_left_) {
      left_dynamics_ = std::make_unique<ArmDynamics>(
        robot_description, left_base_link_, "openarm_left_hand_tcp");
      initialize_arm(
        *left_dynamics_, "left", left_base_link_, "openarm_left_hand_tcp");
    }
    if (enable_right_) {
      right_dynamics_ = std::make_unique<ArmDynamics>(
        robot_description, right_base_link_, "openarm_right_hand_tcp");
      initialize_arm(
        *right_dynamics_, "right", right_base_link_, "openarm_right_hand_tcp");
    }

    left_preview_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "gravity_compensation/left_effort", 10);
    right_preview_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "gravity_compensation/right_effort", 10);
    if (apply_effort_) {
      left_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "left_forward_effort_controller/commands", 10);
      right_effort_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "right_forward_effort_controller/commands", 10);
    }

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      std::bind(
        &GravityCompNode::joint_state_callback, this,
        std::placeholders::_1));

    parameter_callback_ = add_on_set_parameters_callback(
      std::bind(
        &GravityCompNode::parameters_callback, this,
        std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GravityCompNode::update, this));
    ramp_started_at_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "OpenArm gravity compensation ready: enabled=%s apply_effort=%s "
      "scale=%.3f ramp=%.2fs rate=%.1fHz",
      enabled_ ? "true" : "false", apply_effort_ ? "true" : "false",
      gravity_scale_, ramp_duration_s_, publish_rate_hz_);
    if (!apply_effort_) {
      RCLCPP_WARN(
        get_logger(),
        "Dry-run mode: efforts are published for inspection but are "
        "not sent to the robot");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "LIVE gravity feedforward is enabled; torque ramps from zero");
    }
  }

private:
  void validate_startup_parameters()
  {
    if (gravity_scale_ < 0.0 || gravity_scale_ > kMaxGravityScale) {
      throw std::runtime_error("gravity_scale must be in [0.0, 1.2]");
    }
    if (ramp_duration_s_ < 0.0) {
      throw std::runtime_error("ramp_duration_s must be non-negative");
    }
    if (state_timeout_s_ <= 0.0) {
      throw std::runtime_error("state_timeout_s must be positive");
    }
    if (publish_rate_hz_ <= 0.0 || publish_rate_hz_ > 500.0) {
      throw std::runtime_error("publish_rate_hz must be in (0, 500]");
    }
    if (torque_limits_.size() != kArmDof ||
      std::any_of(
        torque_limits_.begin(), torque_limits_.end(),
        [](double value) {
          return !std::isfinite(value) || value <= 0.0;
        }))
    {
      throw std::runtime_error(
              "torque_limits must contain seven positive finite values");
    }
  }

  void initialize_arm(
    ArmDynamics & dynamics, const std::string & side,
    const std::string & base_link, const std::string & tip_link)
  {
    std::string error;
    if (!dynamics.initialize(error)) {
      throw std::runtime_error(
              side + " arm dynamics initialization failed: " +
              error);
    }
    RCLCPP_INFO(
      get_logger(), "%s KDL chain: %s -> %s", side.c_str(),
      base_link.c_str(), tip_link.c_str());
  }

  void joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr message)
  {
    if (message->position.size() < message->name.size()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "joint_states has fewer positions than names");
      return;
    }

    std::unordered_map<std::string, double> positions;
    positions.reserve(message->name.size());
    for (std::size_t i = 0; i < message->name.size(); ++i) {
      positions.emplace(message->name[i], message->position[i]);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    left_state_valid_ =
      !enable_left_ || extract_positions(positions, *left_dynamics_, left_q_);
    right_state_valid_ = !enable_right_ ||
      extract_positions(positions, *right_dynamics_, right_q_);
    if (left_state_valid_ && right_state_valid_) {
      last_joint_state_ = std::chrono::steady_clock::now();
      received_complete_state_ = true;
    }
  }

  static bool extract_positions(
    const std::unordered_map<std::string, double> & positions,
    const ArmDynamics & dynamics, std::vector<double> & output)
  {
    output.clear();
    output.reserve(dynamics.joint_names().size());
    for (const auto & name : dynamics.joint_names()) {
      const auto position = positions.find(name);
      if (position == positions.end() || !std::isfinite(position->second)) {
        output.clear();
        return false;
      }
      output.push_back(position->second);
    }
    return true;
  }

  void update()
  {
    const auto now = std::chrono::steady_clock::now();

    std::vector<double> left_q;
    std::vector<double> right_q;
    bool state_fresh = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const double state_age = received_complete_state_ ?
        std::chrono::duration<double>(
        now - last_joint_state_)
        .count() :
        std::numeric_limits<double>::infinity();
      state_fresh = received_complete_state_ &&
        state_age <= state_timeout_s_ && left_state_valid_ &&
        right_state_valid_;
      left_q = left_q_;
      right_q = right_q_;
    }

    const double requested_scale =
      enabled_ && state_fresh ? gravity_scale_ : 0.0;
    if (requested_scale != ramp_target_scale_) {
      ramp_start_scale_ = current_scale_;
      ramp_target_scale_ = requested_scale;
      ramp_started_at_ = now;
    }
    if (ramp_duration_s_ == 0.0) {
      current_scale_ = ramp_target_scale_;
    } else {
      const double elapsed =
        std::chrono::duration<double>(now - ramp_started_at_).count();
      const double progress = std::clamp(elapsed / ramp_duration_s_, 0.0, 1.0);
      current_scale_ = ramp_start_scale_ +
        progress * (ramp_target_scale_ - ramp_start_scale_);
    }

    if (!state_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No complete fresh joint state; gravity effort is ramping to zero");
    }

    publish_arm(
      left_dynamics_.get(), left_q, current_scale_,
      left_preview_pub_, left_effort_pub_, "left");
    publish_arm(
      right_dynamics_.get(), right_q, current_scale_,
      right_preview_pub_, right_effort_pub_, "right");
  }

  void publish_arm(
    ArmDynamics * dynamics, const std::vector<double> & positions, double scale,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &
    preview_publisher,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &
    effort_publisher,
    const char * side)
  {
    auto output = std_msgs::msg::Float64MultiArray();
    output.data.assign(kArmDof, 0.0);

    if (dynamics != nullptr && positions.size() == kArmDof && scale > 0.0) {
      std::vector<double> gravity;
      std::string error;
      if (!dynamics->compute_gravity(positions, gravity, error)) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "%s gravity calculation failed: %s", side,
          error.c_str());
      } else {
        for (std::size_t i = 0; i < kArmDof; ++i) {
          output.data[i] = std::clamp(
            scale * gravity[i], -torque_limits_[i],
            torque_limits_[i]);
        }
      }
    }

    preview_publisher->publish(output);
    if (apply_effort_ && effort_publisher) {
      effort_publisher->publish(output);
    }
    if (verbose_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "%s scale=%.3f tau=[%.2f %.2f %.2f %.2f %.2f %.2f %.2f]",
        side, scale, output.data[0], output.data[1], output.data[2],
        output.data[3], output.data[4], output.data[5], output.data[6]);
    }
  }

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "gravity_scale") {
        const double value = parameter.as_double();
        if (value < 0.0 || value > kMaxGravityScale) {
          result.successful = false;
          result.reason = "gravity_scale must be in [0.0, 1.2]";
          return result;
        }
        gravity_scale_ = value;
      } else if (parameter.get_name() == "ramp_duration_s") {
        const double value = parameter.as_double();
        if (!std::isfinite(value) || value < 0.0) {
          result.successful = false;
          result.reason = "ramp_duration_s must be non-negative and finite";
          return result;
        }
        ramp_start_scale_ = current_scale_;
        ramp_started_at_ = std::chrono::steady_clock::now();
        ramp_duration_s_ = value;
      } else if (parameter.get_name() == "enabled") {
        enabled_ = parameter.as_bool();
      } else if (parameter.get_name() == "verbose") {
        verbose_ = parameter.as_bool();
      } else if (parameter.get_name() == "apply_effort") {
        result.successful = false;
        result.reason = "apply_effort cannot be changed while running";
        return result;
      }
    }
    return result;
  }

  bool enabled_ = false;
  bool apply_effort_ = false;
  bool enable_left_ = true;
  bool enable_right_ = true;
  std::string left_base_link_ = "openarm_body_link0";
  std::string right_base_link_ = "openarm_body_link0";
  bool verbose_ = false;
  double gravity_scale_ = 0.2;
  double current_scale_ = 0.0;
  double ramp_start_scale_ = 0.0;
  double ramp_target_scale_ = 0.0;
  double ramp_duration_s_ = 3.0;
  double state_timeout_s_ = 0.1;
  double publish_rate_hz_ = 100.0;
  std::vector<double> torque_limits_;

  std::unique_ptr<ArmDynamics> left_dynamics_;
  std::unique_ptr<ArmDynamics> right_dynamics_;
  std::vector<double> left_q_;
  std::vector<double> right_q_;
  bool left_state_valid_ = false;
  bool right_state_valid_ = false;
  bool received_complete_state_ = false;
  std::mutex state_mutex_;
  std::chrono::steady_clock::time_point last_joint_state_;
  std::chrono::steady_clock::time_point ramp_started_at_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    left_preview_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    right_preview_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    left_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    right_effort_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace openarm_gravity_comp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<openarm_gravity_comp::GravityCompNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("openarm_gravity_comp"), "%s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
