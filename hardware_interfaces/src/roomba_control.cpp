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

  // get info from the xacro parameters
  auto roomba_model = info_.hardware_parameters["roomba_model"];
  if (roomba_model == "roomba_400")
      model_ = create::RobotModel::ROOMBA_400;
  else if (roomba_model == "create_1")
      model_ = create::RobotModel::CREATE_1;
  else if (roomba_model == "create_2")
      model_ = create::RobotModel::CREATE_2;
  serial_port_ = info_.hardware_parameters["roomba_serial_port"];
  serial_baud_ = hardware_interface::stod(info_.hardware_parameters["roomba_serial_baud"]);

  // create robot
  robot_ = std::make_unique<create::Create>(model_);

  // configure bumper sensors


  // CONFIGURE JOINTS
  //for (const hardware_interface::ComponentInfo & joint : info_.joints)
  //{
  //  // DiffBotSystem has exactly two states and one command interface on each joint
  //  if (joint.command_interfaces.size() != 1)
  //  {
  //    RCLCPP_FATAL(
  //      get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
  //      joint.name.c_str(), joint.command_interfaces.size());
  //    return hardware_interface::CallbackReturn::ERROR;
  //  }

  //  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  //  {
  //    RCLCPP_FATAL(
  //      get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
  //      joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
  //      hardware_interface::HW_IF_VELOCITY);
  //    return hardware_interface::CallbackReturn::ERROR;
  //  }

  //  if (joint.state_interfaces.size() != 2)
  //  {
  //    RCLCPP_FATAL(
  //      get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
  //      joint.state_interfaces.size());
  //    return hardware_interface::CallbackReturn::ERROR;
  //  }

  //  if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  //  {
  //    RCLCPP_FATAL(
  //      get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
  //      joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
  //      hardware_interface::HW_IF_POSITION);
  //    return hardware_interface::CallbackReturn::ERROR;
  //  }

  //  if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //  {
  //    RCLCPP_FATAL(
  //      get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
  //      joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
  //      hardware_interface::HW_IF_VELOCITY);
  //    return hardware_interface::CallbackReturn::ERROR;
  //  }
  //}

  //// report GPIO
  //RCLCPP_INFO(get_logger(), "HAS '%ld' GPIO components with '%ld' command interfaces",
  //  info_.gpios.size(),
  //  info_.gpios[0].command_interfaces.size()
  //);

  //// report sensors
  //RCLCPP_INFO(get_logger(), "HAS '%ld' senor components with '%ld' state interfaces",
  //  info_.sensors.size(),
  //  info_.sensors[0].state_interfaces.size()
  //);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    RCLCPP_INFO(get_logger(), "Connecting to the Roomba");
    if (robot_->connect(serial_port_, serial_baud_)) {
      RCLCPP_INFO(get_logger(), "Connected to the robot!");
      robot_->setMode(create::MODE_FULL);
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
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoombaSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  //std::stringstream ss;
  //ss << "Reading states:";

  // Read sensors
  get_battery_status();
  get_bumper_readings();
  get_button_status();

  // report GPIOs
  for (const auto & [name, descr] : gpio_state_interfaces_) {
      std::string sensor_name{name};
      // Buttons
      if (sensor_name.find("buttons") != std::string::npos) {
          if (sensor_name.find("clean") != std::string::npos)
              set_state(name, buttons_[0]);
          else if (sensor_name.find("clock") != std::string::npos)
              set_state(name, buttons_[1]);
          else if (sensor_name.find("schedule") != std::string::npos)
              set_state(name, buttons_[2]);
          else if (sensor_name.find("day") != std::string::npos)
              set_state(name, buttons_[3]);
          else if (sensor_name.find("hour") != std::string::npos)
              set_state(name, buttons_[4]);
          else if (sensor_name.find("minute") != std::string::npos)
              set_state(name, buttons_[5]);
          else if (sensor_name.find("dock") != std::string::npos)
              set_state(name, buttons_[6]);
          else if (sensor_name.find("spot") != std::string::npos)
              set_state(name, buttons_[7]);
      }
  }

  // report all sensors
  for (const auto & [name, descr] : sensor_state_interfaces_) {
      std::string sensor_name{name};
      // Battery
      if (sensor_name.find("battery") != std::string::npos) {
          if (sensor_name.find("capacity") != std::string::npos)
              set_state(name, battery_capacity_);
          else if (sensor_name.find("charge") != std::string::npos)
              set_state(name, battery_charge_);
          else if (sensor_name.find("state") != std::string::npos)
              set_state(name, battery_percent_);
          else if (sensor_name.find("voltage") != std::string::npos)
              set_state(name, battery_voltage_);
          else if (sensor_name.find("current") != std::string::npos)
              set_state(name, battery_current_);
          else if (sensor_name.find("temperature") != std::string::npos)
              set_state(name, battery_temperature_);
      }

      // Bumper Light Sensors
      if (sensor_name.find("bool") != std::string::npos) {
          if (sensor_name.find("light_left") != std::string::npos)
              set_state(name, light_bumpers_[0]);
          else if (sensor_name.find("front_left") != std::string::npos)
              set_state(name, light_bumpers_[1]);
          else if (sensor_name.find("center_left") != std::string::npos)
              set_state(name, light_bumpers_[2]);
          else if (sensor_name.find("center_right") != std::string::npos)
              set_state(name, light_bumpers_[3]);
          else if (sensor_name.find("front_right") != std::string::npos)
              set_state(name, light_bumpers_[4]);
          else if (sensor_name.find("light_right") != std::string::npos)
              set_state(name, light_bumpers_[5]);
      } else if (sensor_name.find("raw") != std::string::npos) {
          if (sensor_name.find("light_left") != std::string::npos)
              set_state(name, light_signals_[0]);
          else if (sensor_name.find("front_left") != std::string::npos)
              set_state(name, light_signals_[1]);
          else if (sensor_name.find("center_left") != std::string::npos)
              set_state(name, light_signals_[2]);
          else if (sensor_name.find("center_right") != std::string::npos)
              set_state(name, light_signals_[3]);
          else if (sensor_name.find("front_right") != std::string::npos)
              set_state(name, light_signals_[4]);
          else if (sensor_name.find("light_right") != std::string::npos)
              set_state(name, light_signals_[5]);
      } else if (sensor_name.find("static") != std::string::npos) {
          if (sensor_name.find("left") != std::string::npos)
              set_state(name, static_bumpers_[0]);
          else if (sensor_name.find("right") != std::string::npos)
              set_state(name, static_bumpers_[1]);
      }
  }

  //ss << std::fixed << std::setprecision(2);
  //for (const auto & [name, descr] : joint_state_interfaces_)
  //{
  //  if (descr.get_interface_name() == hardware_interface::HW_IF_POSITION)
  //  {
  //    // Simulate DiffBot wheels's movement as a first-order system
  //    // Update the joint status: this is a revolute joint without any limit.
  //    // Simply integrates
  //    auto velo = get_command(descr.get_prefix_name() + "/" + hardware_interface::HW_IF_VELOCITY);
  //    set_state(name, get_state(name) + period.seconds() * velo);

  //    //ss << std::endl
  //    //   << "\t position " << get_state(name) << " and velocity " << velo << " for '" << name
  //    //   << "'!";
  //  }
  //}
  
  //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type hardware_interfaces::RoombaSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::stringstream ss;
  //ss << "Writing commands:";

  // LEDs
  for (const auto & [name, descr] : gpio_command_interfaces_)
  {
    //std::string command = std::to_string(get_command(name));
    //ss << "LED " << name << " command " << command << std::endl;

    if (name == "status_leds/power_led")
        robot_->setPowerLED((get_command(name)>0));
    else if (name == "status_leds/dock_led")
        robot_->enableDockLED((get_command(name)>0));
    else if (name == "status_leds/debris_led")
        robot_->enableDebrisLED((get_command(name)>0));
    else if (name == "status_leds/check_led")
        robot_->enableCheckRobotLED((get_command(name)>0));
    else if (name == "status_leds/spot_led")
        robot_->enableSpotLED((get_command(name)>0));
  }

  //for (const auto & [name, descr] : joint_command_interfaces_)
  //{
  //  // Simulate sending commands to the hardware
  //  set_state(name, get_command(name));

  //  //ss << std::fixed << std::setprecision(2) << std::endl
  //  //   << "\t" << "command " << get_command(name) << " for '" << name << "'!";
  //}

  //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

void hardware_interfaces::RoombaSystemHardware::get_bumper_readings()
{
    // static bumpers
    static_bumpers_[0] = robot_->isLeftBumper();
    static_bumpers_[1] = robot_->isRightBumper();

    if (model_ == create::RobotModel::CREATE_2) {
        // boolean bumpers
        light_bumpers_[0] = robot_->isLightBumperLeft();
        light_bumpers_[1] = robot_->isLightBumperFrontLeft();
        light_bumpers_[2] = robot_->isLightBumperCenterLeft();
        light_bumpers_[3] = robot_->isLightBumperCenterRight();
        light_bumpers_[4] = robot_->isLightBumperFrontRight();
        light_bumpers_[5] = robot_->isLightBumperRight();

        // raw signals
        light_signals_[0] = robot_->getLightSignalLeft();
        light_signals_[1] = robot_->getLightSignalFrontLeft();
        light_signals_[2] = robot_->getLightSignalCenterLeft();
        light_signals_[3] = robot_->getLightSignalCenterRight();
        light_signals_[4] = robot_->getLightSignalFrontRight();
        light_signals_[5] = robot_->getLightSignalRight();
    }
}

void hardware_interfaces::RoombaSystemHardware::get_battery_status()
{
    battery_capacity_ = robot_->getBatteryCapacity();
    battery_charge_ = robot_->getBatteryCharge();
    battery_percent_ = (battery_charge_ / battery_capacity_) * 100.0;
    battery_voltage_ = robot_->getVoltage();
    battery_current_ = robot_->getCurrent();
    battery_temperature_ = robot_->getTemperature();
}

void hardware_interfaces::RoombaSystemHardware::get_button_status()
{
    buttons_[0] = robot_->isCleanButtonPressed();
    if (model_ != create::RobotModel::CREATE_2) {
        buttons_[1] = robot_->isClockButtonPressed();
        buttons_[2] = robot_->isScheduleButtonPressed();
    }
    buttons_[3] = robot_->isDayButtonPressed();
    buttons_[4] = robot_->isHourButtonPressed();
    buttons_[5] = robot_->isMinButtonPressed();
    buttons_[6] = robot_->isDockButtonPressed();
    buttons_[7] = robot_->isSpotButtonPressed();
}

} // end of hardware_interfaces::RoombaSystemHardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hardware_interfaces::RoombaSystemHardware,
  hardware_interface::SystemInterface
)
