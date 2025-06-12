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

#include "roomba_hardware_interface/roomba_control.hpp"

namespace hardware_interfaces {
hardware_interface::CallbackReturn RoombaSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
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
    
    // initialize all std::unordered_map
    // https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp
    // https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp#L197
    // Alternatively, joint_state_interfaces_, etc can be used
    // I wrote it this way for learning purposes
    std::string interface_name;

    // joints
    for (const hardware_interface::ComponentInfo& joint : info_.joints) {
        interface_name = joint.name;
        // commands
        for (const hardware_interface::InterfaceInfo& interface : joint.command_interfaces) {
            interface_name += ("/" + interface.name);
            std::cout << interface_name << std::endl;
            drive_motors_[interface_name] = {};
        }
    }

    // gpios
    for (const hardware_interface::ComponentInfo& gpio : info_.gpios) {
        interface_name = gpio.name;
        if (interface_name == "cleaning_motors") {
            for (const hardware_interface::InterfaceInfo& motor : gpio.command_interfaces) {
                interface_name += ("/" + motor.name);
                status_leds_[interface_name] = {};
            }
        } else if (interface_name == "status_leds") {
            for (const hardware_interface::InterfaceInfo& led : gpio.command_interfaces) {
                interface_name += ("/" + led.name);
                status_leds_[interface_name] = {};
            }
        } else if (interface_name == "buttons") {
            for (const hardware_interface::InterfaceInfo& btn : gpio.state_interfaces) {
                interface_name += ("/" + btn.name);
                buttons_[interface_name] = {};
            }
        }
    }

    // sensors
    for (const hardware_interface::ComponentInfo& sensor : info_.sensors) {
        interface_name = sensor.name;
        if (interface_name == "bumper_light_sensors_bool") {
            for (const hardware_interface::InterfaceInfo& input : sensor.state_interfaces) {
                interface_name += ("/" + input.name);
                light_bumpers_[interface_name] = {};
            }
        } else if (interface_name == "bumper_light_sensors_raw") {
            for (const hardware_interface::InterfaceInfo& input : sensor.state_interfaces) {
                interface_name += ("/" + input.name);
                light_signals_[interface_name] = {};
            }
        } else if (interface_name == "bumper_light_sensors_static") {
            for (const hardware_interface::InterfaceInfo& input : sensor.state_interfaces) {
                interface_name += ("/" + input.name);
                static_bumpers_[interface_name] = {};
            }
        } else if (interface_name == "battery") {
            for (const hardware_interface::InterfaceInfo& input : sensor.state_interfaces) {
                interface_name += ("/" + input.name);
                battery_[interface_name] = {};
            }
        } else if (interface_name == "cliff") {
            for (const hardware_interface::InterfaceInfo& input : sensor.state_interfaces) {
                interface_name += ("/" + input.name);
                cliff_[interface_name] = {};
            }
        } else if (interface_name == "wheeldrop") {
            for (const hardware_interface::InterfaceInfo& input : sensor.state_interfaces) {
                interface_name += ("/" + input.name);
                wheeldrop_[interface_name] = {};
            }
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
    RCLCPP_INFO(get_logger(), "Connecting to the Roomba");
    if (robot_->connect(serial_port_, serial_baud_)) {
      RCLCPP_INFO(get_logger(), "Connected to the robot!");
      robot_->setMode(system_mode_);
      RCLCPP_INFO(get_logger(), "Robot mode is set");
    } else {
      RCLCPP_INFO(get_logger(), "Failed to connect to the robot!");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // reset values always when configuring hardware
    for (const auto & [name, descr] : joint_state_interfaces_)
        set_state(name, 0.0);
    for (const auto & [name, descr] : joint_command_interfaces_)
        set_command(name, 0.0);

    // gpio configure
    for (const auto & [name, descr] : gpio_state_interfaces_)
        set_state(name, 0.0);
    for (const auto & [name, descr] : gpio_command_interfaces_)
        set_command(name, 0.0);

    // sensors configure
    for (const auto & [name, descr] : sensor_state_interfaces_)
        set_state(name, 0.0);

    RCLCPP_INFO(get_logger(), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // in order to prevent robot motion, set all motors to be 0
  robot_->driveWheels(0.0, 0.0);
  robot_->setAllMotors(0.0, 0.0, 0.0);

  // command and state should be equal when starting
  for (const auto & [name, descr] : joint_command_interfaces_)
    set_command(name, get_state(name));

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // when deactivating, stop joints
  robot_->driveWheels(0.0, 0.0);
  robot_->setAllMotors(0.0, 0.0, 0.0);

  // set all commands to be 0
  for (const auto & [name, descr] : joint_command_interfaces_)
    set_command(name, 0.0);

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoombaSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period) {
    (void)period;
    // Read sensors
    get_battery_status();
    get_bumper_readings();
    get_button_status();
    get_cliff_status();
    get_wheeldrop_status();

    // Wheel states
    for (const auto & [name, descr] : joint_state_interfaces_) {
        //ss << descr.get_prefix_name() << "/" << descr.get_interface_name() << std::endl;
        if (name.find("left") != std::string::npos) {
            if (name.find("position") != std::string::npos)
                set_state(name, static_cast<double>(robot_->getLeftWheelDistance()));
            else if (name.find("velocity") != std::string::npos)
                set_state(name, static_cast<double>(robot_->getMeasuredLeftWheelVel()));
        } else if (name.find("right") != std::string::npos) {
            if (name.find("position") != std::string::npos)
                set_state(name, static_cast<double>(robot_->getRightWheelDistance()));
            else if (name.find("velocity") != std::string::npos)
                set_state(name, static_cast<double>(robot_->getMeasuredRightWheelVel()));
        }
    }

    // GPIOs
    for (const auto & [name, descr] : gpio_state_interfaces_) {
        // Buttons
        if (descr.get_prefix_name() == "buttons")
            set_state(name, buttons_[name]);

        // mode
        if (descr.get_prefix_name() == "docking")
            set_state(name, static_cast<double>(system_mode_));
    }

    // Sensors
    for (const auto & [name, descr] : sensor_state_interfaces_) {
        // Wheel Drop
        if (descr.get_prefix_name() == "wheeldrop")
            set_state(name, wheeldrop_[name]);

        // Cliff
        if (descr.get_prefix_name() == "cliff")
            set_state(name, cliff_[name]);

        // Battery
        if (descr.get_prefix_name() == "battery")
            set_state(name, static_cast<double>(battery_[name]));

        // Bumper Light Sensors
        if (descr.get_prefix_name() == "bumper_light_sensors_bool")
            set_state(name, light_bumpers_[name]);
        else if (descr.get_prefix_name() == "bumper_light_sensors_raw")
            set_state(name, static_cast<double>(light_signals_[name]));
        else if (descr.get_prefix_name() == "bumper_light_sensors_static")
            set_state(name, static_bumpers_[name]);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type hardware_interfaces::RoombaSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // write values for the joints
  static float left_wheel_vel = 0.0;
  static float right_wheel_vel = 0.0;
  for (const auto & [name, descr] : joint_command_interfaces_) {
      drive_motors_[name] = get_command(name);
      if (name.find("left") != std::string::npos)
          left_wheel_vel = drive_motors_[name];
      else if (name.find("right") != std::string::npos)
          right_wheel_vel = drive_motors_[name];
  }
  robot_->driveWheels(left_wheel_vel, right_wheel_vel);

  // write values for GPIO
  for (const auto & [name, descr] : gpio_command_interfaces_) {
    // system mode
    if (descr.get_prefix_name() == "docking") {
        create::CreateMode new_mode = static_cast<create::CreateMode>(get_command(name));
        // cannot allow to turn off robot with 0
        if (system_mode_ != new_mode && new_mode != 0) {
            system_mode_ = new_mode;
            robot_->setMode(system_mode_);
            if (system_mode_ == create::MODE_PASSIVE)
                robot_->dock();
        }
    }

    // Cleaning Motors
    if (descr.get_prefix_name() == "cleaning_motors") {
        robot_->setAllMotors(
            // main [-1, 1]
            (name.find("main") != std::string::npos) ? get_command(name) : 0.0,
            // side [-1, 1]
            (name.find("side") != std::string::npos) ? get_command(name) : 0.0,
            // vacuum [0, 1]
            (name.find("vacuum") != std::string::npos) ? get_command(name) : 0.0
        );
    }

    // LEDs
    if (name.find("status_leds") != std::string::npos) {
        if (name.find("power") != std::string::npos)
            robot_->setPowerLED((get_command(name)>0));
        else if (name.find("dock") != std::string::npos)
            robot_->enableDockLED((get_command(name)>0));
        else if (name.find("debris") != std::string::npos)
            robot_->enableDebrisLED((get_command(name)>0));
        else if (name.find("check") != std::string::npos)
            robot_->enableCheckRobotLED((get_command(name)>0));
        else if (name.find("spot") != std::string::npos)
            robot_->enableSpotLED((get_command(name)>0));
    }
    
  }
  return hardware_interface::return_type::OK;
}

void hardware_interfaces::RoombaSystemHardware::get_bumper_readings()
{
    // wide range of front bumpers and their analog values
    // are available only on Create 2
    if (model_ == create::RobotModel::CREATE_2) {
        // boolean values for bumpers
        for (auto& [key, value] : light_bumpers_) {
             if (key.find("left") != std::string::npos)
                 value = robot_->isLightBumperLeft();
             else if (key.find("front_left") != std::string::npos)
                 value = robot_->isLightBumperFrontLeft();
             else if (key.find("center_left") != std::string::npos)
                 value = robot_->isLightBumperCenterLeft();
             else if (key.find("center_right") != std::string::npos)
                 value = robot_->isLightBumperCenterRight();
             else if (key.find("front_right") != std::string::npos)
                 value = robot_->isLightBumperFrontRight();
             else if (key.find("right") != std::string::npos)
                 value = robot_->isLightBumperRight();
        }
        
        // analog values for bumpers
        for (auto& [key, value] : light_signals_) {
             if (key.find("left") != std::string::npos)
                 value = robot_->getLightSignalLeft();
             else if (key.find("front_left") != std::string::npos)
                 value = robot_->getLightSignalFrontLeft();
             else if (key.find("center_left") != std::string::npos)
                 value = robot_->getLightSignalCenterLeft();
             else if (key.find("center_right") != std::string::npos)
                 value = robot_->getLightSignalCenterRight();
             else if (key.find("front_right") != std::string::npos)
                 value = robot_->getLightSignalFrontRight();
             else if (key.find("right") != std::string::npos)
                 value = robot_->getLightSignalRight();
        }
    }

    // static bumpers
    for (auto& [key, value] : static_bumpers_) {
         if (key.find("left") != std::string::npos)
             value = robot_->isLeftBumper();
         else if (key.find("right") != std::string::npos)
             value = robot_->isRightBumper();
    }
}

void hardware_interfaces::RoombaSystemHardware::get_battery_status()
{
    for (auto& [key, value] : battery_) {
         if (key.find("capacity") != std::string::npos)
             value = robot_->getBatteryCapacity();
         else if (key.find("charge") != std::string::npos)
             value = robot_->getBatteryCharge();
         else if (key.find("state") != std::string::npos)
             value = (battery_["charge"] / battery_["capacity"]) * 100.0;
         else if (key.find("voltage") != std::string::npos)
             value = robot_->getVoltage();
         else if (key.find("current") != std::string::npos)
             value = robot_->getCurrent();
         else if (key.find("temperature") != std::string::npos)
             value = robot_->getTemperature();
    }
}

void hardware_interfaces::RoombaSystemHardware::get_button_status()
{
    bool is_create2 = model_ == create::RobotModel::CREATE_2;
    for (auto& [key, value] : buttons_) {
         if (key.find("clean") != std::string::npos)
             value = robot_->isCleanButtonPressed();
         else if (key.find("clock") != std::string::npos)
             value = (is_create2) ? false : robot_->isClockButtonPressed();
         else if (key.find("schedule") != std::string::npos)
             value = (is_create2) ? false : robot_->isScheduleButtonPressed();
         else if (key.find("day") != std::string::npos)
             value = robot_->isDayButtonPressed();
         else if (key.find("hour") != std::string::npos)
             value = robot_->isHourButtonPressed();
         else if (key.find("minute") != std::string::npos)
             value = robot_->isMinButtonPressed();
         else if (key.find("dock") != std::string::npos)
             value = robot_->isDockButtonPressed();
         else if (key.find("spot") != std::string::npos)
             value = robot_->isSpotButtonPressed();
    }
}

void hardware_interfaces::RoombaSystemHardware::get_cliff_status()
{
    for (auto& [key, value] : cliff_) {
         if (key.find("front_left") != std::string::npos)
             value = robot_->isCliffFrontLeft();
         else if (key.find("left") != std::string::npos)
             value = robot_->isCliffLeft();
         else if (key.find("front_right") != std::string::npos)
             value = robot_->isCliffFrontRight();
         else if (key.find("right") != std::string::npos)
             value = robot_->isCliffRight();
    }
}

void hardware_interfaces::RoombaSystemHardware::get_wheeldrop_status()
{
    for (auto& [key, value] : wheeldrop_) {
         if (key.find("left") != std::string::npos)
             value = robot_->isLeftWheeldrop();
         else if (key.find("right") != std::string::npos)
             value = robot_->isRightWheeldrop();
    }
}

} // end of hardware_interfaces::RoombaSystemHardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hardware_interfaces::RoombaSystemHardware,
  hardware_interface::SystemInterface
)
