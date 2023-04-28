#ifndef LITTLEBOT_BASE__LITTLEBOT_HARDWARE_HPP_
#define LITTLEBOT_BASE__LITTLEBOT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include <libserial/serial.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "littlebot_base/visibility_control.h"

using namespace std::chrono_literals;

namespace littlebot_base
{
class LittlebotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LittlebotHardware)

  LITTLEBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  LITTLEBOT_BASE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  LITTLEBOT_BASE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  LITTLEBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  LITTLEBOT_BASE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  LITTLEBOT_BASE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  LITTLEBOT_BASE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the wheeled robot position
  double base_x_;
  double base_y_;
  double base_theta_;
};

}  // namespace littlebot_base

#endif  // LITTLEBOT_BASE__LITTLEBOT_HARDWARE_HPP_