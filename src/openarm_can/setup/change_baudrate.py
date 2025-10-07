#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
DM Motor ID and Baudrate Writer script for writing baudrate and ID parameters to DM motors
Work with python-can library
Original author: @necobit (Co-Authored-By: Claude)
"""

import argparse
import sys
import can
import time
from typing import Optional


class DMMotorWriter:
    """DM Motor parameter writer"""

    # Supported baudrates and their codes
    BAUDRATE_MAP = {
        125000: 0,   # 125K
        200000: 1,   # 200K
        250000: 2,   # 250K
        500000: 3,   # 500K
        1000000: 4,  # 1M
        2000000: 5,  # 2M
        2500000: 6,  # 2.5M
        3200000: 7,  # 3.2M
        4000000: 8,  # 4M
        5000000: 9   # 5M
    }

    def __init__(self, socketcan_port: str = "can0"):
        self.socketcan_port = socketcan_port
        self.can_bus: Optional[can.BusABC] = None

    def connect(self) -> bool:
        """Connect to CAN bus"""
        try:
            print(f"Connecting to CAN interface: {self.socketcan_port}")
            self.can_bus = can.interface.Bus(
                channel=self.socketcan_port,
                interface='socketcan'
            )
            print(f"✓ Connected to {self.socketcan_port}")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False

    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.can_bus:
            self.can_bus.shutdown()
            self.can_bus = None
            print("Disconnected from CAN bus")

    def validate_baudrate(self, baudrate: int) -> bool:
        """Validate if baudrate is supported"""
        return baudrate in self.BAUDRATE_MAP

    def write_baudrate(self, motor_id: int, baudrate: int) -> bool:
        """Write baudrate to motor"""
        if not self.can_bus:
            print("✗ Not connected to CAN bus")
            return False

        if not self.validate_baudrate(baudrate):
            print(f"✗ Unsupported baudrate: {baudrate}")
            print(f"Supported baudrates: {list(self.BAUDRATE_MAP.keys())}")
            return False

        try:
            baudrate_code = self.BAUDRATE_MAP[baudrate]

            print(
                f"Writing baudrate {baudrate} (code: {baudrate_code}) to motor ID {motor_id}")

            # Set Baudrate (can_br) - RID 35 (0x23)
            baudrate_data = [
                motor_id & 0xFF,
                (motor_id >> 8) & 0xFF,
                0x55,
                0x23,
                baudrate_code,
                0x00,
                0x00,
                0x00
            ]

            baudrate_msg = can.Message(
                arbitration_id=0x7FF,
                data=baudrate_data,
                is_extended_id=False
            )

            self.can_bus.send(baudrate_msg)
            time.sleep(0.1)  # Wait for processing

            print(f"✓ Baudrate {baudrate} written to motor ID {motor_id}")
            return True

        except Exception as e:
            print(f"✗ Failed to write baudrate: {e}")
            return False

    def save_to_flash(self, motor_id: int) -> bool:
        """Save parameters to flash memory"""
        if not self.can_bus:
            print("✗ Not connected to CAN bus")
            return False

        try:
            print(f"Saving parameters to flash for motor ID {motor_id}")
            print("⚠️  Motor has a hard limit of writing to flash of 10000 times")
            print("⚠️  Motor will be disabled during save operation")

            # Step 1: Disable motor first (required for save operation)
            disable_data = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
            disable_msg = can.Message(
                arbitration_id=motor_id,
                data=disable_data,
                is_extended_id=False
            )

            self.can_bus.send(disable_msg)
            time.sleep(0.5)  # Wait for disable to take effect

            # Step 2: Send save command
            save_data = [
                motor_id & 0xFF,
                (motor_id >> 8) & 0xFF,
                0xAA,
                0x00,
                0x00,
                0x00,
                0x00,
                0x00
            ]

            save_msg = can.Message(
                arbitration_id=0x7FF,
                data=save_data,
                is_extended_id=False
            )

            self.can_bus.send(save_msg)
            time.sleep(0.5)  # Wait for save operation (up to 30ms)

            print(f"✓ Parameters saved to flash for motor ID {motor_id}")
            print("⚠️  Motor is now in DISABLE mode")
            return True

        except Exception as e:
            print(f"✗ Failed to save to flash: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(
        description="DM Motor ID and Baudrate Writer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Write baudrate 1000000 to motor ID 2 on can0
  python change_baudrate.py --baudrate 1000000 --canid 2 --socketcan can0

  # Write baudrate and save to flash
  python change_baudrate.py --baudrate 500000 --canid 1 --socketcan can0 --flash

  # Write to motor on different CAN port
  python change_baudrate.py --baudrate 2000000 --canid 3 --socketcan can1

Supported baudrates:
  125000, 200000, 250000, 500000, 1000000, 2000000, 2500000, 3200000, 4000000, 5000000
        """
    )

    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        required=True,
        help='Baudrate to write (125000, 200000, 250000, 500000, 1000000, 2000000, 2500000, 3200000, 4000000, 5000000)'
    )

    parser.add_argument(
        '-c', '--canid',
        type=int,
        required=True,
        help='Motor CAN ID (slave ID) to write to (0-255)'
    )

    parser.add_argument(
        '-s', '--socketcan',
        type=str,
        default='can0',
        help='SocketCAN port (default: can0)'
    )

    parser.add_argument(
        '-f', '--flash',
        action='store_true',
        help='Save parameters to flash memory after writing'
    )

    args = parser.parse_args()

    # Validate inputs
    if args.canid < 0 or args.canid > 255:
        print("✗ Error: CAN ID must be between 0 and 255")
        sys.exit(1)

    # Create writer instance
    writer = DMMotorWriter(args.socketcan)

    try:
        # Connect to CAN bus
        if not writer.connect():
            print("✗ Failed to connect to CAN bus")
            sys.exit(1)

        # Write baudrate
        if not writer.write_baudrate(args.canid, args.baudrate):
            print("✗ Failed to write baudrate")
            sys.exit(1)

        # Save to flash if requested
        if args.flash:
            if not writer.save_to_flash(args.canid):
                print("✗ Failed to save to flash")
                sys.exit(1)

        print("✓ Operation completed successfully")

    except KeyboardInterrupt:
        print("\n⚠️  Operation cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"✗ Unexpected error: {e}")
        sys.exit(1)
    finally:
        writer.disconnect()


if __name__ == "__main__":
    main()
