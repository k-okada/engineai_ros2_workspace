// Copyright 2026 Kei Okada
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

#include "engineai_ros2/engineai_hardware_interface.hpp"
#include "engineai_ros2/engineai_util.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "interface_protocol/msg/parallel_parser_type.hpp"

#include <sstream>
#include <vector>

#include <sstream>
#include <string>
#include <vector>

namespace engineai_hardware_interface
{
hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
#if ENGINEAI_NEEDS_LOCAL_GETSET
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.EngineAI"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
#endif

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
#if ENGINEAI_NEEDS_LOCAL_GETSET
  RCLCPP_INFO(get_logger(), "Robot hardware_component initialized");
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  joint_index_.clear();
  joint_index_.reserve(info_.joints.size());
  joint_state_interfaces_.clear();
  joint_state_interfaces_.reserve(info_.joints.size());
  joint_command_interfaces_.clear();
  joint_command_interfaces_.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;
    joint_index_[name] = i;
    joint_state_interfaces_.emplace_back(name + "/position", i);
    joint_command_interfaces_.emplace_back(name + "/position", i);
  }
#else
  RCLCPP_INFO(get_logger(), "Robot hardware_component update_rate is %dHz", info_.rw_rate);
#endif

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // EngineAI_SystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // create node
  node_ = std::make_shared<rclcpp::Node>("engineai_hardware_interface");

  // initialize joint_names
  joint_names_.clear();
  joint_names_.reserve(info_.joints.size());
  for (const auto &j : info_.joints) {
    joint_names_.push_back(j.name);
  }

  //
  const size_t n = joint_names_.size();

  // Initialize per-joint vectors with defaults
  stiffness_per_joint_.assign(n, default_stiffness_);
  damping_per_joint_.assign(n, default_damping_);

  // Parse overrides
  const auto stiff_map = parse_name_value_csv(
    get_string_param(info_.hardware_parameters, "stiffness_by_joint", ""));
  const auto damp_map  = parse_name_value_csv(
    get_string_param(info_.hardware_parameters, "damping_by_joint", ""));

  // Apply overrides by joint name
  for (size_t i = 0; i < n; ++i) {
    const auto& name = joint_names_[i];

    auto itS = stiff_map.find(name);
    if (itS != stiff_map.end()) stiffness_per_joint_[i] = itS->second;

    auto itD = damp_map.find(name);
    if (itD != damp_map.end()) damping_per_joint_[i] = itD->second;
  }

  RCLCPP_INFO(get_logger(),
              "Loaded per-joint stiffness/damping (n=%zu). Defaults: k=%.2f d=%.2f",
              n, default_stiffness_, default_damping_);

  // publisher
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(10));

  joint_pub_ = node_->create_publisher<interface_protocol::msg::JointCommand>
    ("/hardware/joint_command", rclcpp::QoS(10));

  // subscriber
  joint_sub_ = node_->create_subscription<interface_protocol::msg::JointState>(
    "/hardware/joint_state", 10,
    [this](interface_protocol::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const auto noj = std::min(joint_names_.size(), msg->position.size());
      for(size_t i = 0; i < noj; ++i) {
        latest_pos_[joint_names_[i]] = msg->position[i];
        latest_vel_[joint_names_[i]] = msg->velocity[i];
        latest_tor_[joint_names_[i]] = msg->torque[i];
      }
    });

  imu_sub_ = node_->create_subscription<interface_protocol::msg::ImuInfo>(
    "/hardware/imu_info", 10,
    [this](interface_protocol::msg::ImuInfo::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      latest_imu_.header.stamp = node_->now();
      latest_imu_.header.frame_id = "LINK_BASE";
      // orientation
      latest_imu_.orientation.x = msg->quaternion.x;
      latest_imu_.orientation.y = msg->quaternion.y;
      latest_imu_.orientation.z = msg->quaternion.z;
      latest_imu_.orientation.w = msg->quaternion.w;
      // linear
      latest_imu_.linear_acceleration.x = msg->linear_acceleration.x;
      latest_imu_.linear_acceleration.y = msg->linear_acceleration.y;
      latest_imu_.linear_acceleration.z = msg->linear_acceleration.z;
      // angular
      latest_imu_.angular_velocity.x = msg->angular_velocity.x;
      latest_imu_.angular_velocity.y = msg->angular_velocity.y;
      latest_imu_.angular_velocity.z = msg->angular_velocity.z;
      // orientation covariance [rad^2]
      latest_imu_.orientation_covariance[0] = 1e-3;
      latest_imu_.orientation_covariance[4] = 1e-3;
      latest_imu_.orientation_covariance[8] = 1e-3;
      // angular velocity covariance [(rad/s)^2]
      latest_imu_.angular_velocity_covariance[0] = 1e-3;
      latest_imu_.angular_velocity_covariance[4] = 1e-3;
      latest_imu_.angular_velocity_covariance[8] = 1e-3;
      // linear acceleration covariance [(m/s^2)^2]
      latest_imu_.linear_acceleration_covariance[0] = 1e-2;
      latest_imu_.linear_acceleration_covariance[4] = 1e-2;
      latest_imu_.linear_acceleration_covariance[8] = 1e-2;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "publish latest imu: %7.3f %7.3f %7.3f", latest_imu_.linear_acceleration.x, latest_imu_.linear_acceleration.y, latest_imu_.linear_acceleration.z);
      // publish
      imu_pub_->publish(latest_imu_);
    });

  RCLCPP_INFO(get_logger(), "info_.joints.size()=%zu", info_.joints.size());
  for (const auto& j : info_.joints) {
    RCLCPP_INFO(get_logger(), "info joint: %s", j.name.c_str());
  }
  RCLCPP_INFO(get_logger(), "EngineAI topic-bridge hardware initialized");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }
  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

#if ENGINEAI_NEEDS_LOCAL_GETSET
bool EngineAI_SystemPositionOnlyHardware::set_command(const std::string & key, double value)
{
  auto idx_opt = joint_index(joint_names_, key);
  if (!idx_opt) {
    RCLCPP_WARN_THROTTLE(*logger_, *clock_, 2000,
      "set_command: unknown key='%s' (normalized='%s')",
      key.c_str(), normalize_joint_key(key).c_str());
    return false;
  }
  const size_t idx = *idx_opt;
  if (idx >= hw_commands_.size()) {
    const std::string joint = normalize_joint_key(key);
    RCLCPP_ERROR(*logger_,
      "set_command: index out of range idx=%zu size=%zu key='%s'",
      idx, hw_commands_.size(), joint.c_str());
    return false;
  }
  hw_commands_[idx] = value;
  return true;
}

double EngineAI_SystemPositionOnlyHardware::get_command(const std::string & key) const
{
  auto idx_opt = joint_index(joint_names_, key);
  if (!idx_opt) {
    RCLCPP_WARN_THROTTLE(*logger_, *clock_, 2000,
      "get_command: unknown key='%s' (normalized='%s')",
      key.c_str(), normalize_joint_key(key).c_str());
    return 0.0;
  }
  const size_t idx = *idx_opt;
  if (idx >= hw_commands_.size()) {
    const std::string joint = normalize_joint_key(key);
    RCLCPP_ERROR(*logger_,
      "get_command: index out of range idx=%zu size=%zu key='%s'",
      idx, hw_commands_.size(), joint.c_str());
    return 0.0;
  }
  return hw_commands_[idx];
}

bool EngineAI_SystemPositionOnlyHardware::set_state(const std::string & key, double value)
{
  auto idx_opt = joint_index(joint_names_, key);
  if (!idx_opt) {
    RCLCPP_WARN_THROTTLE(*logger_, *clock_, 2000,
      "set_state: unknown key='%s' (normalized='%s')",
      key.c_str(), normalize_joint_key(key).c_str());
    return false;
  }
  const size_t idx = *idx_opt;
  if (idx >= hw_states_.size()) {  // idxもsize_tなので -Wsign-compare 回避
    const std::string joint = normalize_joint_key(key);
    RCLCPP_ERROR(*logger_,
      "set_state: index out of range idx=%zu size=%zu key='%s'",
      idx, hw_states_.size(), joint.c_str());
    return false;
  }
  hw_states_[idx] = value;
  return true;
}

double EngineAI_SystemPositionOnlyHardware::get_state(const std::string & key) const
{
  auto idx_opt = joint_index(joint_names_, key);
  if (!idx_opt) {
    RCLCPP_WARN_THROTTLE(*logger_, *clock_, 2000,
      "get_state: unknown key='%s' (normalized='%s')",
      key.c_str(), normalize_joint_key(key).c_str());
    return 0.0;
  }
  const size_t idx = *idx_opt;
  if (idx >= hw_states_.size()) {
    const std::string joint = normalize_joint_key(key);
    RCLCPP_ERROR(*logger_,
      "get_state: index out of range idx=%zu size=%zu key='%s'",
      idx, hw_states_.size(), joint.c_str());
    return 0.0;
  }
  return hw_states_[idx];
}

std::vector<hardware_interface::StateInterface>
EngineAI_SystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size());

  std::fprintf(stderr, "### export_state_interfaces CALLED ### joints=%zu\n", info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & jn = info_.joints[i].name;
    std::fprintf(stderr, "export about to add [%zu] joint='%s' len=%zu\n", i, jn.c_str(), jn.size());

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        jn,
        hardware_interface::HW_IF_POSITION,
        &hw_states_[i]
      )
    );

    // 返す実体からも表示
    std::fprintf(stderr, "export added [%zu] name='%s' if='%s'\n",
      i,
      state_interfaces.back().get_name().c_str(),
      state_interfaces.back().get_interface_name().c_str()
    );
  }

  std::fprintf(stderr, "export_state_interfaces returning %zu interfaces\n", state_interfaces.size());
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
EngineAI_SystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}
#endif

hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EngineAI_SystemPositionOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EngineAI_SystemPositionOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  rclcpp::spin_some(node_);

  std::lock_guard<std::mutex> lk(mtx_);
  for (const auto & joint : joint_names_) {
    auto it = latest_pos_.find(joint);
    if (it != latest_pos_.end()) {
      set_state(joint + "/position", it->second);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EngineAI_SystemPositionOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Build header line
  const auto header = make_joint_header_line(joint_names_);
  // Build position line
  const auto pos_line = make_joint_value_line(joint_names_,
					      [this](const std::string& key) {
						return get_command(key);
					      },
					      "/position");
  // Print both lines with throttling
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s\n%s",
		       header.c_str(), pos_line.c_str());

  // publish joint_command_ros
  interface_protocol::msg::JointCommand cmd;

  // header
  cmd.header.stamp = get_clock()->now();
  cmd.header.frame_id = "";  // 必要なら robot 名など

  const size_t n = joint_command_interfaces_.size();

  cmd.position.reserve(n);
  cmd.velocity.assign(n, 0.0);
  cmd.feed_forward_torque.assign(n, 0.0);
  cmd.torque.assign(n, 0.0);
  cmd.stiffness = stiffness_per_joint_;
  cmd.damping = damping_per_joint_;
  for (const auto & name : joint_names_)
  {
    cmd.position.push_back(get_command(name + "/position"));  // position command
  }
  cmd.parallel_parser_type = interface_protocol::msg::ParallelParserType::RL_PARSER;

  joint_pub_->publish(cmd);;

  return hardware_interface::return_type::OK;
}

}  // namespace engineai_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  engineai_hardware_interface::EngineAI_SystemPositionOnlyHardware, hardware_interface::SystemInterface)
