#ifndef HARDWARE_INTERFACES_ROOMBA_SYSTEM_HPP_
#define HARDWARE_INTERFACES_ROOMBA_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

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

// roomba stuff
#include "create/create.h"

namespace hardware_interfaces
{
class RoombaSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoombaSystemHardware);

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

private:
   // robot itself
   std::unique_ptr<create::Create> robot_;

  // parameters from the hardware interface `ros2_control.xacro`
  create::RobotModel model_ = create::RobotModel::CREATE_2;
  std::string serial_port_;
  int serial_baud_;

  // buttons
  bool buttons_[8] = {false};

  // sensors
  //std::mutex bumper_mtx_;
  bool static_bumpers_[2] = {false};
  bool light_bumpers_[6] = {false};
  int light_signals_[6] = {0};

  // battery
  float battery_capacity_;
  float battery_charge_;
  float battery_percent_;
  float battery_voltage_;
  float battery_current_;
  int battery_temperature_;

  // functions
  void get_bumper_readings();
  void get_battery_status();
  void get_button_status();
};

}  // namespace hardware_interfaces

#endif
