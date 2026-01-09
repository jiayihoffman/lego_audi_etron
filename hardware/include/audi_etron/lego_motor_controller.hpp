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

#ifndef AUDI_ETRON__LEGO_MOTOR_CONTROLLER_HPP_
#define AUDI_ETRON__LEGO_MOTOR_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace audi_etron
{

// LEGO Technic Hub port definitions
enum class LegoPort : uint8_t
{
  PORT_A = 0x00,
  PORT_B = 0x01,
  PORT_D = 0x03
};

// Forward declaration for BLE implementation
class LegoMotorControllerImpl;

/**
 * @brief Controller for LEGO Technic Hub motors via Bluetooth Low Energy
 * 
 * This class provides an interface to control motors connected to a LEGO Technic Hub
 * using the LEGO Wireless Protocol over BLE. It supports up to 3 motors:
 * - Port A and B: Drive motors (typically used for traction)
 * - Port D: Steering motor
 */
class LegoMotorController
{
public:
  /**
   * @brief Construct a new Lego Motor Controller object
   * @param logger Logger for diagnostic messages
   */
  explicit LegoMotorController(rclcpp::Logger logger);

  /**
   * @brief Destroy the Lego Motor Controller object
   */
  ~LegoMotorController();

  /**
   * @brief Connect to the LEGO Technic Hub
   * @param hub_name Name pattern to search for (default: "Technic")
   * @return true if connection successful, false otherwise
   */
  bool connect(const std::string & hub_name = "Technic");

  /**
   * @brief Disconnect from the LEGO Technic Hub
   */
  void disconnect();

  /**
   * @brief Check if connected to the hub
   * @return true if connected, false otherwise
   */
  bool is_connected() const;

  /**
   * @brief Set motor power for a specific port
   * @param port Port identifier (PORT_A, PORT_B, or PORT_D)
   * @param power Power value from -100 to 100 (negative = reverse)
   * @return true if command sent successfully, false otherwise
   */
  bool set_motor_power(LegoPort port, int8_t power);

  /**
   * @brief Stop all motors
   * @return true if all stop commands sent successfully, false otherwise
   */
  bool stop_all_motors();

private:
  /**
   * @brief Create the start_power message for LEGO Wireless Protocol
   * @param port Port identifier
   * @param power Power value from -100 to 100
   * @return Message bytes as vector
   */
  std::vector<uint8_t> create_start_power_message(LegoPort port, int8_t power) const;

  rclcpp::Logger logger_;
  std::unique_ptr<LegoMotorControllerImpl> impl_;
};

}  // namespace audi_etron

#endif  // AUDI_ETRON__LEGO_MOTOR_CONTROLLER_HPP_

