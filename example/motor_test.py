"""
This script controls the three motors in LEGO Technic 42160 using Bluetooth,
	1.	Scans for the Technic Hub
	2.	Connects
	3.	Starts all three motors
	4.	Turns steering
	5.	Stops motors

# bleak (cross-platform BLE library)
pip install bleak
"""

import asyncio
from bleak import BleakClient, BleakScanner

UART_CHAR_UUID = "00001624-1212-efde-1623-785feabcd123"

# Ports
PORT_A = 0x00
PORT_B = 0x01
PORT_D = 0x03

def start_power(port, power):
    """
    power: -100 .. 100
    """
    return bytes([
        0x07,       # message length
        0x00,       # hub id
        0x81,       # port output command
        port,
        0x11,       # startup + completion
        0x01,       # START POWER
        power & 0xFF
    ])

async def main():
    print("Scanning for Technic Hub...")
    devices = await BleakScanner.discover()

    hub = next((d for d in devices if d.name and "Technic" in d.name), None)
    if not hub:
        print("Hub not found")
        return

    async with BleakClient(hub) as client:
        print("Connected!")

        # Drive forward
        print("Driving motors A + B")
        await client.write_gatt_char(UART_CHAR_UUID, start_power(PORT_A, 80), response=False)
        await client.write_gatt_char(UART_CHAR_UUID, start_power(PORT_B, 80), response=False)

        await asyncio.sleep(1.5)

        print("Driving motors A + B Backwards")
        await client.write_gatt_char(UART_CHAR_UUID, start_power(PORT_A, -80), response=False)
        await client.write_gatt_char(UART_CHAR_UUID, start_power(PORT_B, -80), response=False)

        await asyncio.sleep(1.5)

        print("Steer left")
        await client.write_gatt_char(UART_CHAR_UUID, start_power(PORT_D, 25), response=False)
        await asyncio.sleep(0.35)   # tweak this value

        print("Return to center")
        await client.write_gatt_char(UART_CHAR_UUID, start_power(PORT_D, -25), response=False)
        await asyncio.sleep(0.35)   # must match the above

        # Stop everything
        print("Stopping motors")
        for port in (PORT_A, PORT_B, PORT_D):
            await client.write_gatt_char(
                UART_CHAR_UUID,
                start_power(port, 0),
                response=False
            )

        print("Done")

asyncio.run(main())

