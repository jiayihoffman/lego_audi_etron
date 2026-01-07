# Copyright (c) 2026 Jiayi Hoffman. All rights reserved

"""LEGO Technic Hub motor controller using Bluetooth Low Energy."""

import asyncio
from typing import Optional
import logging

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    raise ImportError(
        "bleak is required for LEGO motor control. Install with: pip install bleak"
    )

# LEGO Wireless Protocol UUID
UART_CHAR_UUID = "00001624-1212-efde-1623-785feabcd123"

# Port definitions
PORT_A = 0x00
PORT_B = 0x01
PORT_D = 0x03


class LegoMotorController:
    """Controller for LEGO Technic Hub motors via Bluetooth Low Energy."""

    def __init__(self, logger: Optional[logging.Logger] = None):
        """
        Initialize the LEGO motor controller.

        Args:
            logger: Optional logger instance. If None, creates a default logger.
        """
        self.logger = logger or logging.getLogger(__name__)
        self.client: Optional[BleakClient] = None
        self.connected = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread = None

    def _start_event_loop(self):
        """Start an event loop in a separate thread for async operations."""
        if self._loop is None or self._loop.is_closed():
            self._loop = asyncio.new_event_loop()
            self._loop_thread = None  # We'll run in the current thread's event loop

    def _create_start_power_message(self, port: int, power: int) -> bytes:
        """
        Create the start_power message for LEGO Wireless Protocol.

        Args:
            port: Port identifier (PORT_A, PORT_B, or PORT_D)
            power: Power value from -100 to 100 (negative = reverse)

        Returns:
            Message bytes
        """
        # Clamp power to valid range
        clamped_power = max(-100, min(100, power))

        return bytes([
            0x07,  # message length
            0x00,  # hub id
            0x81,  # port output command
            port,  # port
            0x11,  # startup + completion
            0x01,  # START POWER
            clamped_power & 0xFF  # power (signed byte)
        ])

    async def _connect_async(self, hub_name: str) -> bool:
        """
        Connect to the LEGO Technic Hub asynchronously.

        Args:
            hub_name: Name pattern to search for (default: "Technic")

        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.logger.info(f"Scanning for LEGO Technic Hub (name pattern: {hub_name})...")
            devices = await BleakScanner.discover(timeout=3.0)

            # Find the Technic Hub
            hub = None
            for device in devices:
                if device.name and hub_name in device.name:
                    hub = device
                    self.logger.info(
                        f"Found Technic Hub: {device.name} (Address: {device.address})"
                    )
                    break

            if not hub:
                self.logger.error("LEGO Technic Hub not found")
                return False

            # Connect to the hub
            self.logger.info("Connecting to hub...")
            self.client = BleakClient(hub)
            await self.client.connect()

            if not self.client.is_connected:
                self.logger.error("Failed to connect to hub")
                return False

            self.connected = True
            self.logger.info("Successfully connected to LEGO Technic Hub!")
            return True

        except Exception as e:
            self.logger.error(f"Exception during connection: {e}")
            self.connected = False
            return False

    def connect(self, hub_name: str = "Technic") -> bool:
        """
        Connect to the LEGO Technic Hub (synchronous wrapper).

        Args:
            hub_name: Name pattern to search for (default: "Technic")

        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Try to get existing event loop, or create new one
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

            return loop.run_until_complete(self._connect_async(hub_name))
        except Exception as e:
            self.logger.error(f"Exception in connect: {e}")
            return False

    async def _disconnect_async(self):
        """Disconnect from the LEGO Technic Hub asynchronously."""
        try:
            if self.connected and self.client and self.client.is_connected:
                self.logger.info("Disconnecting from hub...")
                await self.client.disconnect()
                self.connected = False
                self.logger.info("Disconnected from hub")
        except Exception as e:
            self.logger.error(f"Exception during disconnect: {e}")
            self.connected = False

    def disconnect(self):
        """Disconnect from the LEGO Technic Hub (synchronous wrapper)."""
        try:
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

            loop.run_until_complete(self._disconnect_async())
        except Exception as e:
            self.logger.error(f"Exception in disconnect: {e}")
            self.connected = False

    def is_connected(self) -> bool:
        """
        Check if connected to the hub.

        Returns:
            True if connected, False otherwise
        """
        if self.client:
            try:
                return self.connected and self.client.is_connected
            except Exception:
                return False
        return False

    async def _set_motor_power_async(self, port: int, power: int) -> bool:
        """
        Set motor power for a specific port asynchronously.

        Args:
            port: Port identifier (PORT_A, PORT_B, or PORT_D)
            power: Power value from -100 to 100 (negative = reverse)

        Returns:
            True if command sent successfully, False otherwise
        """
        try:
            if not self.connected or not self.client or not self.client.is_connected:
                self.logger.warning("Not connected to hub")
                return False

            message = self._create_start_power_message(port, power)
            await self.client.write_gatt_char(UART_CHAR_UUID, message, response=False)
            return True

        except Exception as e:
            self.logger.error(f"Exception writing to hub: {e}")
            return False

    def set_motor_power(self, port: int, power: int) -> bool:
        """
        Set motor power for a specific port (synchronous wrapper).

        Args:
            port: Port identifier (PORT_A, PORT_B, or PORT_D)
            power: Power value from -100 to 100 (negative = reverse)

        Returns:
            True if command sent successfully, False otherwise
        """
        try:
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)

            return loop.run_until_complete(self._set_motor_power_async(port, power))
        except Exception as e:
            self.logger.error(f"Exception in set_motor_power: {e}")
            return False

    def stop_all_motors(self) -> bool:
        """
        Stop all motors.

        Returns:
            True if all stop commands sent successfully, False otherwise
        """
        success = True
        success &= self.set_motor_power(PORT_A, 0)
        success &= self.set_motor_power(PORT_B, 0)
        success &= self.set_motor_power(PORT_D, 0)
        return success

