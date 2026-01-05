// Copyright (c) 2026 Jiayi Hoffman. All rights reserved

#include "audi_etron/lego_motor_controller.hpp"

#include <algorithm>
#include <thread>
#include <chrono>

// SimpleBLE includes
#include <simpleble/SimpleBLE.h>

namespace audi_etron
{

// LEGO Wireless Protocol UUID
static const std::string UART_CHAR_UUID = "00001624-1212-efde-1623-785feabcd123";

// PIMPL implementation class
class LegoMotorControllerImpl
{
public:
  SimpleBLE::Adapter adapter_;
  SimpleBLE::Peripheral peripheral_;
  bool connected_{false};

  bool connect_to_hub(const std::string & hub_name, rclcpp::Logger logger)
  {
    try {
      // Get the first available adapter
      auto adapters = SimpleBLE::Adapter::get_adapters();
      if (adapters.empty()) {
        RCLCPP_ERROR(logger, "No Bluetooth adapter found");
        return false;
      }

      adapter_ = adapters[0];
      RCLCPP_INFO(logger, "Using adapter: %s", adapter_.identifier().c_str());

      // Enable the adapter
      if (!adapter_.bluetooth_enabled()) {
        RCLCPP_INFO(logger, "Enabling Bluetooth...");
        adapter_.bluetooth_enable();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      // Scan for devices
      RCLCPP_INFO(logger, "Scanning for LEGO Technic Hub (name pattern: %s)...", hub_name.c_str());
      adapter_.scan_for(3000);  // Scan for 3 seconds

      auto peripherals = adapter_.scan_get_results();
      RCLCPP_INFO(logger, "Found %zu devices", peripherals.size());

      // Find the Technic Hub
      SimpleBLE::Peripheral hub;
      bool found = false;
      for (auto & p : peripherals) {
        std::string name = p.identifier();
        RCLCPP_DEBUG(logger, "Found device: %s", name.c_str());
        if (name.find(hub_name) != std::string::npos) {
          hub = p;
          found = true;
          RCLCPP_INFO(logger, "Found Technic Hub: %s (Address: %s)", name.c_str(), p.address().c_str());
          break;
        }
      }

      if (!found) {
        RCLCPP_ERROR(logger, "LEGO Technic Hub not found");
        return false;
      }

      // Connect to the hub
      RCLCPP_INFO(logger, "Connecting to hub...");
      hub.connect();
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      if (!hub.is_connected()) {
        RCLCPP_ERROR(logger, "Failed to connect to hub");
        return false;
      }

      peripheral_ = hub;
      connected_ = true;
      RCLCPP_INFO(logger, "Successfully connected to LEGO Technic Hub!");
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Exception during connection: %s", e.what());
      connected_ = false;
      return false;
    }
  }

  void disconnect_from_hub(rclcpp::Logger logger)
  {
    try {
      if (connected_ && peripheral_.is_connected()) {
        RCLCPP_INFO(logger, "Disconnecting from hub...");
        peripheral_.disconnect();
        connected_ = false;
        RCLCPP_INFO(logger, "Disconnected from hub");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Exception during disconnect: %s", e.what());
      connected_ = false;
    }
  }

  bool write_characteristic(
    const std::vector<uint8_t> & data,
    rclcpp::Logger logger)
  {
    try {
      if (!connected_ || !peripheral_.is_connected()) {
        RCLCPP_WARN(logger, "Not connected to hub");
        return false;
      }

      auto services = peripheral_.services();
      for (const auto & service : services) {
        for (const auto & characteristic : service.characteristics()) {
          if (characteristic.uuid() == UART_CHAR_UUID) {
            peripheral_.write_request(service.uuid(), characteristic.uuid(), data);
            return true;
          }
        }
      }

      RCLCPP_ERROR(logger, "UART characteristic not found");
      return false;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Exception writing to hub: %s", e.what());
      return false;
    }
  }
};

LegoMotorController::LegoMotorController(rclcpp::Logger logger)
: logger_(logger), impl_(std::make_unique<LegoMotorControllerImpl>())
{
}

LegoMotorController::~LegoMotorController()
{
  disconnect();
}

bool LegoMotorController::connect(const std::string & hub_name)
{
  return impl_->connect_to_hub(hub_name, logger_);
}

void LegoMotorController::disconnect()
{
  impl_->disconnect_from_hub(logger_);
}

bool LegoMotorController::is_connected() const
{
  return impl_->connected_ && impl_->peripheral_.is_connected();
}

std::vector<uint8_t> LegoMotorController::create_start_power_message(
  LegoPort port, int8_t power) const
{
  // Clamp power to valid range
  int8_t clamped_power = std::max(static_cast<int8_t>(-100), std::min(static_cast<int8_t>(100), power));

  return std::vector<uint8_t>{
    0x07,  // message length
    0x00,  // hub id
    0x81,  // port output command
    static_cast<uint8_t>(port),  // port
    0x11,  // startup + completion
    0x01,  // START POWER
    static_cast<uint8_t>(clamped_power & 0xFF)  // power (signed byte)
  };
}

bool LegoMotorController::set_motor_power(LegoPort port, int8_t power)
{
  auto message = create_start_power_message(port, power);
  return impl_->write_characteristic(message, logger_);
}

bool LegoMotorController::stop_all_motors()
{
  bool success = true;
  success &= set_motor_power(LegoPort::PORT_A, 0);
  success &= set_motor_power(LegoPort::PORT_B, 0);
  success &= set_motor_power(LegoPort::PORT_D, 0);
  return success;
}

}  // namespace audi_etron

