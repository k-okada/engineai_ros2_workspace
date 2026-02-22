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

#ifndef ENGINEAI_HARDWARE_INTERFACE_HPP_
#define ENGINEAI_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "interface_protocol/msg/joint_state.hpp"
#include "interface_protocol/msg/imu_info.hpp"
#include "interface_protocol/msg/joint_command.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "hardware_interface/version.h"
#if defined(HARDWARE_INTERFACE_VERSION_MAJOR)
// use get/set for 4.18.0-
  #if (HARDWARE_INTERFACE_VERSION_MAJOR < 4) || \
      (HARDWARE_INTERFACE_VERSION_MAJOR == 4 && HARDWARE_INTERFACE_VERSION_MINOR < 18)
    #define ENGINEAI_NEEDS_LOCAL_GETSET 1
  #else
    #define ENGINEAI_NEEDS_LOCAL_GETSET 0
  #endif
#else
  // use old interface when version.h is not found
  #define ENGINEAI_NEEDS_LOCAL_GETSET 0
#endif

namespace engineai_hardware_interface
{
class EngineAI_SystemPositionOnlyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EngineAI_SystemPositionOnlyHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

#if ENGINEAI_NEEDS_LOCAL_GETSET
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

  bool set_command(const std::string& key, double value);
  double get_command(const std::string& key) const;
  bool set_state(const std::string & key, double value);
  double get_state(const std::string & key) const;

  std::unordered_map<std::string, size_t> joint_index_;
  std::vector<std::pair<std::string, size_t>> joint_state_interfaces_;
  std::vector<std::pair<std::string, size_t>> joint_command_interfaces_;
#endif

private:
  // Parameters for the engineai simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

#if ENGINEAI_NEEDS_LOCAL_GETSET
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
#endif

  double default_stiffness_{20.0};
  double default_damping_{1.0};
  std::vector<double> stiffness_per_joint_;
  std::vector<double> damping_per_joint_;

  // Publish & Subscribe
  rclcpp::Node::SharedPtr node_;

  rclcpp::Subscription<interface_protocol::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<interface_protocol::msg::ImuInfo>::SharedPtr imu_sub_;
  rclcpp::Publisher<interface_protocol::msg::JointCommand>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::mutex mtx_;
  std::unordered_map<std::string, double> latest_pos_;
  std::unordered_map<std::string, double> latest_vel_;
  std::unordered_map<std::string, double> latest_tor_;
  sensor_msgs::msg::Imu latest_imu_;
  std::vector<std::string> joint_names_;
};

}  // namespace engineai_hardware_interface

#endif  // ENGINEAI_HARDWARE_INTERFACE_HPP_
