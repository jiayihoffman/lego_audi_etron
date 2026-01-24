// Copyright (c) 2026 Jiayi Hoffman. 
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
//
// The hardware components realize communication to physical hardware and represent its abstraction in the 
// ros2_control framework. The components have to be exported as plugins and the Resource Manager dynamically
// loads those plugins and manages their lifecycle.
//
// There are three basic types of components: Actuator, Sensor and System. 
// System: Complex (multi-DOF) robotic hardware like industrial robots.
// Actuator: Simple (1 DOF) robotic hardware like motors, valves, and similar. An actuator implementation is related to only one joint. 
// Sensor: Robotic hardware is used for sensing its environment. 
//
// The ros2_control framework provides a set of hardware interface types that can be used to implement a hardware component 
// for a specific robot or device: Joints, Sensors, GPIOs
// Joints: 
// <joint>-tag groups the interfaces associated with the joints of physical robots and actuators. 
// They have command and state interfaces to set the goal values for hardware and read its current state.
// The State interfaces of joints can be published as a ROS topic by means of the joint_state_broadcaster
// 
// <ros2_control name="RRBotSystemMutipleGPIOs" type="system">
//  <hardware>
//    <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
//    <param name="example_param_hw_start_duration_sec">2.0</param>
//    <param name="example_param_hw_stop_duration_sec">3.0</param>
//    <param name="example_param_hw_slowdown">2.0</param>
//  </hardware>
//  <joint name="joint1">
//    <command_interface name="position">
//      <param name="min">-1</param>
//      <param name="max">1</param>
//    </command_interface>
//    <state_interface name="position"/>
//  </joint>
//  <gpio name="flange_digital_IOs">
//    <command_interface name="digital_output1"/>
//    <state_interface name="digital_output1"/>    <!-- current state of the output -->
//    <command_interface name="digital_output2"/>
//    <state_interface name="digital_output2"/>
//    <state_interface name="digital_input1"/>
//    <state_interface name="digital_input2"/>
//  </gpio>
// </ros2_control>
// 
// References:
// https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html
// https://control.ros.org/humble/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

#include "audi_etron/carlikebot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "audi_etron/lego_motor_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace audi_etron
{

// initialize all member variables and process the parameters from the info argument
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.CarlikeBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a steering joint.", joint.name.c_str());

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
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
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
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Joint '%s' is a drive joint.", joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
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
          get_logger(), "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }


  // Motor scaling parameters (with defaults if not specified)
  if (info_.hardware_parameters.find("max_traction_power") != info_.hardware_parameters.end()) {
    max_traction_power_ = std::stod(info_.hardware_parameters["max_traction_power"]);
  } else {
    max_traction_power_ = 80.0;  // Default to 80% max power
  }

  if (info_.hardware_parameters.find("max_steering_power") != info_.hardware_parameters.end()) {
    max_steering_power_ = std::stod(info_.hardware_parameters["max_steering_power"]);
  } else {
    max_steering_power_ = 50.0;  // Default to 50% max power for steering
  }

  if (info_.hardware_parameters.find("max_traction_velocity") != info_.hardware_parameters.end()) {
    max_traction_velocity_ = std::stod(info_.hardware_parameters["max_traction_velocity"]);
  } else {
    max_traction_velocity_ = 25.0;  // Default max velocity (rad/s), corresponds to ~1.25 m/s with 0.05m wheel radius
  }

  if (info_.hardware_parameters.find("max_steering_position") != info_.hardware_parameters.end()) {
    max_steering_position_ = std::stod(info_.hardware_parameters["max_steering_position"]);
  } else {
    max_steering_position_ = 0.4;  // Default max steering position (rad), matches URDF joint limit
  }

  if (info_.hardware_parameters.find("steering_deadzone") != info_.hardware_parameters.end()) {
    steering_deadzone_ = std::stod(info_.hardware_parameters["steering_deadzone"]);
  } else {
    steering_deadzone_ = 0.05;  // Default deadzone (rad) - ~2.9 degrees, forces center when command is very small
  }

  if (info_.hardware_parameters.find("hub_name") != info_.hardware_parameters.end()) {
    hub_name_ = info_.hardware_parameters["hub_name"];
  } else {
    hub_name_ = "Technic";  // Default hub name pattern
  }

  // Initialize LEGO motor controller
  lego_motor_controller_ = std::make_unique<LegoMotorController>(get_logger());

  hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

  hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu state interfaces.", state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(get_logger(), "Exported state interface '%s'.", s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION,
        &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
        &joint.second.command.velocity));
    }
  }

  RCLCPP_INFO(get_logger(), "Exported %zu command interfaces.", command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      get_logger(), "Exported command interface '%s'.", command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

// Implement the on_activate method where hardware "power" is enabled.
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating hardware...");

  // Connect to LEGO Technic Hub
  RCLCPP_INFO(get_logger(), "Connecting to LEGO Technic Hub...");
  if (!lego_motor_controller_->connect(hub_name_)) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to LEGO Technic Hub");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first == "steering")
    {
      joint.second.command.position = 0.0;
    }
  }

  // Ensure all motors are stopped initially
  lego_motor_controller_->stop_all_motors();

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating hardware...");

  // Stop all motors before disconnecting
  if (lego_motor_controller_ && lego_motor_controller_->is_connected()) {
    RCLCPP_INFO(get_logger(), "Stopping all motors...");
    lego_motor_controller_->stop_all_motors();
    rclcpp::sleep_for(std::chrono::milliseconds(100));  // Give time for commands to be sent

    // Disconnect from hub
    RCLCPP_INFO(get_logger(), "Disconnecting from LEGO Technic Hub...");
    lego_motor_controller_->disconnect();
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Open-loop operation: Echo commands as states (no sensor feedback)
  
  // Steering position: echo command as state
  hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;

  // Traction velocity: echo command as state
  hw_interfaces_["traction"].state.velocity = hw_interfaces_["traction"].command.velocity;
  
  // Integrate velocity to estimate position for odometry (open-loop estimation)
  hw_interfaces_["traction"].state.position +=
    hw_interfaces_["traction"].state.velocity * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type audi_etron::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Check if motor controller is connected
  if (!lego_motor_controller_ || !lego_motor_controller_->is_connected()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "LEGO motor controller not connected");
    return hardware_interface::return_type::OK;
  }

  // Convert steering position command to motor power
  double steering_command = hw_interfaces_["steering"].command.position;
  
  // Apply deadzone: if command is within deadzone threshold, force to zero to actively center steering
  if (std::abs(steering_command) < steering_deadzone_) {
    steering_command = 0.0;
  }
  
  // Scale steering position (-max_steering_position to max_steering_position) to power (-max_steering_power to max_steering_power)
  double steering_power_raw = 0.0;
  if (max_steering_position_ > 0.0) {
    steering_power_raw = (steering_command / max_steering_position_) * max_steering_power_;
  }
  // Clamp to valid range
  steering_power_raw = std::max(-max_steering_power_, std::min(max_steering_power_, steering_power_raw));
  int8_t steering_power = static_cast<int8_t>(std::round(steering_power_raw));
  
  // Send steering command to PORT_D (negate to fix opposite direction)
  lego_motor_controller_->set_motor_power(LegoPort::PORT_D, -steering_power);

  // Convert traction velocity command to motor power
  double traction_velocity = hw_interfaces_["traction"].command.velocity;
  // Scale velocity (-max_traction_velocity to max_traction_velocity) to power (-max_traction_power to max_traction_power)
  double traction_power_raw = 0.0;
  if (max_traction_velocity_ > 0.0) {
    traction_power_raw = (traction_velocity / max_traction_velocity_) * max_traction_power_;
  }
  // Clamp to valid range
  traction_power_raw = std::max(-max_traction_power_, std::min(max_traction_power_, traction_power_raw));
  int8_t traction_power = static_cast<int8_t>(std::round(traction_power_raw));

  // Send traction commands to PORT_A and PORT_B (both wheels)
  lego_motor_controller_->set_motor_power(LegoPort::PORT_A, traction_power);
  lego_motor_controller_->set_motor_power(LegoPort::PORT_B, traction_power);


  return hardware_interface::return_type::OK;
}

}  // namespace audi_etron

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  audi_etron::CarlikeBotSystemHardware, hardware_interface::SystemInterface)
