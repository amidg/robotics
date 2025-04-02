#include "hardware_interfaces/roomba_control.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware_interfaces {
hardware_interface::CallbackReturn RoombaSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  //// BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //hw_start_sec_ =
  //  hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  //hw_stop_sec_ =
  //  hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  //// END: This part here is for exemplary purposes - Please do not copy to your production code

  // CONFIGURE JOINTS
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // CONFIGURE GPIO
  RCLCPP_INFO(get_logger(), "HAS '%ld' GPIO components with '%ld' command interfaces",
    info_.gpios.size(),
    info_.gpios[0].command_interfaces.size()
  );

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  //for (int i = 0; i < hw_start_sec_; i++)
  //{
  //  rclcpp::sleep_for(std::chrono::seconds(1));
  //  RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  //}
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(get_logger(), "Connecting to the Roomba");
  //create::Create robot(model);
  if (robot.connect(port, baud)) {
    RCLCPP_INFO(get_logger(), "Connected to the robot!");
    robot.setMode(create::MODE_FULL);
    RCLCPP_INFO(get_logger(), "Robot mode is set");
  } else {
    RCLCPP_INFO(get_logger(), "Failed to connect to the robot!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  // gpio configure
  for (const auto & [name, descr] : gpio_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  //for (auto i = 0; i < hw_start_sec_; i++)
  //{
  //  rclcpp::sleep_for(std::chrono::seconds(1));
  //  RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  //}
  //// END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  //for (const auto & [name, descr] : gpio_command_interfaces_)
  //{
  //  set_command(name, get_state(name));
  //}

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  //for (auto i = 0; i < hw_stop_sec_; i++)
  //{
  //  rclcpp::sleep_for(std::chrono::seconds(1));
  //  RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  //}
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoombaSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  //ss << "Reading states:";
  //ss << std::fixed << std::setprecision(2);
  for (const auto & [name, descr] : joint_state_interfaces_)
  {
    if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
    {
      // Simulate DiffBot wheels's movement as a first-order system
      // Update the joint status: this is a revolute joint without any limit.
      // Simply integrates
      auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
      set_state(name, get_state(name) + period.seconds() * velo);

      //ss << std::endl
      //   << "\t position " << get_state(name) << " and velocity " << velo << " for '" << name
      //   << "'!";
    }
  }

  // led states
  //for (const auto & [name, descr] : gpio_state_interfaces_)
  //{
  //  ss << std::fixed << std::setprecision(2) << std::endl
  //     << "\t" << get_state(name) << " from GPIO input '" << name << "'";
  //}
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hardware_interfaces::RoombaSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    set_state(name, get_command(name));

    //ss << std::fixed << std::setprecision(2) << std::endl
    //   << "\t" << "command " << get_command(name) << " for '" << name << "'!";
  }

  // LED controls
  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    std::string command = std::to_string(get_command(name));
    ss << "LED " << name << " command " << command << std::endl;

    if (name == "status_leds/power_led")
        robot.setPowerLED((get_command(name)>0));
    else if (name == "status_leds/dock_led")
        robot.enableDockLED((get_command(name)>0));
    else if (name == "status_leds/debris_led")
        robot.enableDebrisLED((get_command(name)>0));
    else if (name == "status_leds/check_led")
        robot.enableCheckRobotLED((get_command(name)>0));
    else if (name == "status_leds/spot_led")
        robot.enableSpotLED((get_command(name)>0));
    //RCLCPP_INFO(get_logger(), command.c_str());
    // Simulate sending commands to the hardware
    //ss << std::fixed << std::setprecision(2) << std::endl
    //   << "\t" << get_command(name) << " for GPIO output '" << name << "'";
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn set_status_leds(const bool (&led_commands)[5]) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hardware_interfaces::RoombaSystemHardware,
  hardware_interface::SystemInterface
)
