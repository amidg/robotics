#ifndef HARDWARE_INTERFACES_ROOMBA_SYSTEM_HPP_
#define HARDWARE_INTERFACES_ROOMBA_SYSTEM_HPP_

#include <memory>
#include <string>
#include <unordered_map>

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

  // drive motors [left, right]
  std::unordered_map<std::string, float> drive_motors_;

  /*
   * GPIOs from the hardware interface description:
   * - cleaning motors [main, side, vacuum]
   * - buttons [clean, clock, schedule, day, hour, minute, dock, spot]
   * - status_leds [debris, spot, dock, check, power]
   */
  std::unordered_map<std::string, float> cleaning_motors_;
  std::unordered_map<std::string, bool> buttons_;
  std::unordered_map<std::string, bool> status_leds_;
  create::CreateMode system_mode_ = create::MODE_FULL;

  /*
   * Sensors from the hardware interface description:
   * - light_bumpers_ (left, front_left, center_left, center_right, front_right, right)
   * - light_signals_ (left, front_left, center_left, center_right, front_right, right)
   * - static_bumpers_ (left, right)
   * - battery_ (capacity, charge, state, voltage, current, temperature)
   * - cliff_sensors_ (left, front_left, right, front_right)
   * - wheel_drop_ (left, right)
   */
  std::unordered_map<std::string, bool> light_bumpers_;
  std::unordered_map<std::string, int> light_signals_;
  std::unordered_map<std::string, bool> static_bumpers_;
  std::unordered_map<std::string, float> battery_;
  std::unordered_map<std::string, bool> cliff_;
  std::unordered_map<std::string, bool> wheeldrop_;

  /*
   * Sensors functions
   */
  void get_bumper_readings();
  void get_battery_status();
  void get_button_status();
  void get_cliff_status();
  void get_wheeldrop_status();
}; // class RoombaSystemHardware

}  // namespace hardware_interfaces

#endif
