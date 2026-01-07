# Copyright (c) 2026 Jiayi Hoffman. All rights reserved

"""ROS2 Control hardware interface for LEGO Technic Hub carlike robot."""

from typing import List
import logging

from hardware_interface import (
    CallbackReturn,
    HardwareInfo,
    return_type,
    SystemInterface,
    StateInterface,
    CommandInterface,
)
from hardware_interface.types import HW_IF_POSITION, HW_IF_VELOCITY
from rclpy.lifecycle import State

from .lego_motor_controller import LegoMotorController, PORT_A, PORT_B, PORT_D


class CarlikeBotSystemHardware(SystemInterface):
    """Hardware interface for LEGO Technic Hub carlike robot."""

    def __init__(self):
        """Initialize the hardware interface."""
        super().__init__()
        self.logger = None
        self.clock = None

        # Joint data structures
        self.hw_interfaces = {}

        # LEGO motor controller
        self.lego_motor_controller: LegoMotorController = None

        # Motor scaling parameters
        self.max_traction_power = 80.0
        self.max_steering_power = 50.0
        self.max_traction_velocity = 25.0
        self.max_steering_position = 0.4
        self.hub_name = "Technic"

    def on_init(self, info: HardwareInfo) -> CallbackReturn:
        """
        Initialize the hardware interface.

        Args:
            info: Hardware information from the configuration

        Returns:
            CallbackReturn.SUCCESS if initialization successful, ERROR otherwise
        """
        if super().on_init(info) != CallbackReturn.SUCCESS:
            return CallbackReturn.ERROR

        # Create logger
        self.logger = logging.getLogger(
            "controller_manager.resource_manager.hardware_component.system.CarlikeBot"
        )

        # Check if the number of joints is correct
        if len(info.joints) != 2:
            self.logger.error(
                f"CarlikeBotSystemHardware::on_init() - Failed to initialize, "
                f"because the number of joints {len(info.joints)} is not 2."
            )
            return CallbackReturn.ERROR

        # Validate joint interfaces
        for joint in info.joints:
            joint_is_steering = "front" in joint.name

            if joint_is_steering:
                self.logger.info(f"Joint '{joint.name}' is a steering joint.")

                if len(joint.command_interfaces) != 1:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {len(joint.command_interfaces)} "
                        f"command interfaces found. 1 expected."
                    )
                    return CallbackReturn.ERROR

                if joint.command_interfaces[0].name != HW_IF_POSITION:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {joint.command_interfaces[0].name} "
                        f"command interface. '{HW_IF_POSITION}' expected."
                    )
                    return CallbackReturn.ERROR

                if len(joint.state_interfaces) != 1:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {len(joint.state_interfaces)} "
                        f"state interface. 1 expected."
                    )
                    return CallbackReturn.ERROR

                if joint.state_interfaces[0].name != HW_IF_POSITION:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {joint.state_interfaces[0].name} "
                        f"state interface. '{HW_IF_POSITION}' expected."
                    )
                    return CallbackReturn.ERROR
            else:
                self.logger.info(f"Joint '{joint.name}' is a drive joint.")

                if len(joint.command_interfaces) != 1:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {len(joint.command_interfaces)} "
                        f"command interfaces found. 1 expected."
                    )
                    return CallbackReturn.ERROR

                if joint.command_interfaces[0].name != HW_IF_VELOCITY:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {joint.command_interfaces[0].name} "
                        f"command interface. '{HW_IF_VELOCITY}' expected."
                    )
                    return CallbackReturn.ERROR

                if len(joint.state_interfaces) != 2:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {len(joint.state_interfaces)} "
                        f"state interface. 2 expected."
                    )
                    return CallbackReturn.ERROR

                if joint.state_interfaces[0].name != HW_IF_VELOCITY:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {joint.state_interfaces[1].name} "
                        f"state interface. '{HW_IF_VELOCITY}' expected."
                    )
                    return CallbackReturn.ERROR

                if joint.state_interfaces[1].name != HW_IF_POSITION:
                    self.logger.fatal(
                        f"Joint '{joint.name}' has {joint.state_interfaces[1].name} "
                        f"state interface. '{HW_IF_POSITION}' expected."
                    )
                    return CallbackReturn.ERROR

        # Motor scaling parameters (with defaults if not specified)
        if "max_traction_power" in info.hardware_parameters:
            self.max_traction_power = float(info.hardware_parameters["max_traction_power"])
        else:
            self.max_traction_power = 80.0  # Default to 80% max power

        if "max_steering_power" in info.hardware_parameters:
            self.max_steering_power = float(info.hardware_parameters["max_steering_power"])
        else:
            self.max_steering_power = 50.0  # Default to 50% max power for steering

        if "max_traction_velocity" in info.hardware_parameters:
            self.max_traction_velocity = float(info.hardware_parameters["max_traction_velocity"])
        else:
            self.max_traction_velocity = 25.0  # Default max velocity (rad/s)

        if "max_steering_position" in info.hardware_parameters:
            self.max_steering_position = float(info.hardware_parameters["max_steering_position"])
        else:
            self.max_steering_position = 0.4  # Default max steering position (rad)

        if "hub_name" in info.hardware_parameters:
            self.hub_name = info.hardware_parameters["hub_name"]
        else:
            self.hub_name = "Technic"  # Default hub name pattern

        # Initialize LEGO motor controller
        self.lego_motor_controller = LegoMotorController(self.logger)

        # Initialize joint data structures
        self.hw_interfaces["steering"] = {
            "joint_name": "virtual_front_wheel_joint",
            "state": {"position": 0.0},
            "command": {"position": 0.0},
        }

        self.hw_interfaces["traction"] = {
            "joint_name": "virtual_rear_wheel_joint",
            "state": {"position": 0.0, "velocity": 0.0},
            "command": {"velocity": 0.0},
        }

        return CallbackReturn.SUCCESS

    def export_state_interfaces(self) -> List[StateInterface]:
        """
        Export state interfaces.

        Returns:
            List of state interfaces
        """
        state_interfaces = []

        for joint_key, joint_data in self.hw_interfaces.items():
            state_interfaces.append(
                StateInterface(
                    joint_data["joint_name"],
                    HW_IF_POSITION,
                    joint_data["state"]["position"],
                )
            )

            if joint_key == "traction":
                state_interfaces.append(
                    StateInterface(
                        joint_data["joint_name"],
                        HW_IF_VELOCITY,
                        joint_data["state"]["velocity"],
                    )
                )

        self.logger.info(f"Exported {len(state_interfaces)} state interfaces.")

        for s in state_interfaces:
            self.logger.info(f"Exported state interface '{s.get_name()}'.")

        return state_interfaces

    def export_command_interfaces(self) -> List[CommandInterface]:
        """
        Export command interfaces.

        Returns:
            List of command interfaces
        """
        command_interfaces = []

        for joint_key, joint_data in self.hw_interfaces.items():
            if joint_key == "steering":
                command_interfaces.append(
                    CommandInterface(
                        joint_data["joint_name"],
                        HW_IF_POSITION,
                        joint_data["command"]["position"],
                    )
                )
            elif joint_key == "traction":
                command_interfaces.append(
                    CommandInterface(
                        joint_data["joint_name"],
                        HW_IF_VELOCITY,
                        joint_data["command"]["velocity"],
                    )
                )

        self.logger.info(f"Exported {len(command_interfaces)} command interfaces.")

        for i, cmd in enumerate(command_interfaces):
            self.logger.info(f"Exported command interface '{cmd.get_name()}'.")

        return command_interfaces

    def on_activate(self, previous_state: State) -> CallbackReturn:
        """
        Activate the hardware interface.

        Args:
            previous_state: Previous lifecycle state

        Returns:
            CallbackReturn.SUCCESS if activation successful, ERROR otherwise
        """
        self.logger.info("Activating hardware...")

        # Connect to LEGO Technic Hub
        self.logger.info("Connecting to LEGO Technic Hub...")
        if not self.lego_motor_controller.connect(self.hub_name):
            self.logger.error("Failed to connect to LEGO Technic Hub")
            return CallbackReturn.ERROR

        # Initialize joint states
        for joint_key, joint_data in self.hw_interfaces.items():
            joint_data["state"]["position"] = 0.0

            if joint_key == "traction":
                joint_data["state"]["velocity"] = 0.0
                joint_data["command"]["velocity"] = 0.0
            elif joint_key == "steering":
                joint_data["command"]["position"] = 0.0

        # Ensure all motors are stopped initially
        self.lego_motor_controller.stop_all_motors()

        self.logger.info("Successfully activated!")

        return CallbackReturn.SUCCESS

    def on_deactivate(self, previous_state: State) -> CallbackReturn:
        """
        Deactivate the hardware interface.

        Args:
            previous_state: Previous lifecycle state

        Returns:
            CallbackReturn.SUCCESS if deactivation successful
        """
        self.logger.info("Deactivating hardware...")

        # Stop all motors before disconnecting
        if self.lego_motor_controller and self.lego_motor_controller.is_connected():
            self.logger.info("Stopping all motors...")
            self.lego_motor_controller.stop_all_motors()

            import time
            time.sleep(0.1)  # Give time for commands to be sent

            # Disconnect from hub
            self.logger.info("Disconnecting from LEGO Technic Hub...")
            self.lego_motor_controller.disconnect()

        self.logger.info("Successfully deactivated!")

        return CallbackReturn.SUCCESS

    def read(self, time, period) -> return_type:
        """
        Read hardware state (open-loop: echo commands as states).

        Args:
            time: Current time
            period: Time period since last read

        Returns:
            return_type.OK
        """
        # Open-loop operation: Echo commands as states (no sensor feedback)

        # Steering position: echo command as state
        self.hw_interfaces["steering"]["state"]["position"] = (
            self.hw_interfaces["steering"]["command"]["position"]
        )

        # Traction velocity: echo command as state
        self.hw_interfaces["traction"]["state"]["velocity"] = (
            self.hw_interfaces["traction"]["command"]["velocity"]
        )

        # Integrate velocity to estimate position for odometry (open-loop estimation)
        self.hw_interfaces["traction"]["state"]["position"] += (
            self.hw_interfaces["traction"]["state"]["velocity"] * period.nanoseconds / 1e9
        )

        return return_type.OK

    def write(self, time, period) -> return_type:
        """
        Write commands to hardware.

        Args:
            time: Current time
            period: Time period since last write

        Returns:
            return_type.OK
        """
        # Check if motor controller is connected
        if not self.lego_motor_controller or not self.lego_motor_controller.is_connected():
            self.logger.warning("LEGO motor controller not connected")
            return return_type.OK

        # Convert steering position command to motor power
        steering_command = self.hw_interfaces["steering"]["command"]["position"]
        # Scale steering position to power
        steering_power_raw = 0.0
        if self.max_steering_position > 0.0:
            steering_power_raw = (steering_command / self.max_steering_position) * self.max_steering_power
        # Clamp to valid range
        steering_power_raw = max(-self.max_steering_power, min(self.max_steering_power, steering_power_raw))
        steering_power = int(round(steering_power_raw))

        # Send steering command to PORT_D
        self.lego_motor_controller.set_motor_power(PORT_D, steering_power)

        # Convert traction velocity command to motor power
        traction_velocity = self.hw_interfaces["traction"]["command"]["velocity"]
        # Scale velocity to power
        traction_power_raw = 0.0
        if self.max_traction_velocity > 0.0:
            traction_power_raw = (traction_velocity / self.max_traction_velocity) * self.max_traction_power
        # Clamp to valid range
        traction_power_raw = max(-self.max_traction_power, min(self.max_traction_power, traction_power_raw))
        traction_power = int(round(traction_power_raw))

        # Send traction commands to PORT_A and PORT_B (both wheels)
        self.lego_motor_controller.set_motor_power(PORT_A, traction_power)
        self.lego_motor_controller.set_motor_power(PORT_B, traction_power)

        return return_type.OK

