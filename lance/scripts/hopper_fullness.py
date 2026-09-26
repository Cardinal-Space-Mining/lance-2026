#!/usr/bin/env python3

import glob
import time
from serial import Serial

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32


"""
Hopper Fullness Sensor Driver Node.

This script interfaces with an external microcontroller (such as an Arduino or
CH340/CH341 USB-to-serial adapter, denoted by '1a86' vendor ID) connected to
hopper fullness sensors (e.g., optical, ultrasonic, or weight sensors).
It continuously polls the serial stream, parses the fullness percentage/level,
and publishes the reading as a sensor_msgs/msg/Float32 on "lance/hopper_fullness".
"""

def serial_ports() -> list[str]:
    """
    Search for connected serial devices matching the CH34x USB-to-UART vendor ID (1a86).
    Returns a list of device paths under /dev/serial/by-id/.
    """
    try:
        return glob.glob("/dev/serial/by-id/*1a86*")
    except:
        return []

def make_serial(port: str) -> Serial | None:
    """
    Attempt to instantiate and open a serial connection on the given port at 9600 baud.
    Returns the Serial object on success, or None on failure.
    """
    try:
        return Serial(port, 9600)
    except:
        return None

def wait_for_serial_port(attempts=1, sleep_time=1) -> Serial | None:
    """
    Poll for an available serial port matching the CH34x adapter, retrying
    up to `attempts` times with `sleep_time` seconds delay between retries.
    """
    for i in range(attempts):
        try:
            ports = glob.glob("/dev/serial/by-id/*1a86*")
            for j in range(len(ports)):
                serial = make_serial(ports[j])
                if serial is not None:
                    return serial
        except:
            pass

        if i + 1 < attempts:
            time.sleep(sleep_time)
    return None


class HopperFullnessNode(Node):
    """
    ROS 2 node that reads hopper fullness measurements over serial and
    publishes them as sensor data.
    """
    publisher_: Publisher
    timer: Timer
    serial: Serial

    __slots__ = ("timer", "publisher_", "serial")

    def __get_data(self) -> str:
        """Read a single line from the serial connection and decode as UTF-8 string."""
        return self.serial.readline().decode().strip()

    def __init__(self, serial_port: str | None = None):
        super().__init__("hopper_fullness_driver")
        # Publisher for the hopper fullness reading (topic: lance/hopper_fullness)
        self.publisher_ = self.create_publisher(Float32,
                                                "lance/hopper_fullness",
                                                qos_profile_sensor_data)
        # Periodically poll serial every 1 second
        self.timer = self.create_timer(1, self.timer_callback)
        self.serial = None

        if isinstance(serial_port, str):
            self.serial = make_serial(serial_port)

        # If no explicit port given or failed to open, search dynamically
        if self.serial is None:
            self.serial = wait_for_serial_port(1)

        if self.serial is None:
            print('Failed to initialize valid serial port!')
        else:
            # Clear any partial line in buffer before normal reading starts
            self.serial.read_until()

    def timer_callback(self):
        """
        Timer callback executing once per second:
        1. Reconnects to serial if connection was lost or not yet established.
        2. Reads a line from the sensor.
        3. Parses format "Fs: <value>" (Fullness sensor reading).
        4. Publishes float data onto "lance/hopper_fullness".
        """
        if self.serial is None:
            self.serial = wait_for_serial_port(1)

        if self.serial is not None:
            try:
                line = self.__get_data()
                # Expected format is "Fs: <float_value>"
                if "Fs: " in line:
                    data = float(line.replace("Fs: ", ""))
                    print(f'{line} --> {data}')

                    msg = Float32()
                    msg.data = data
                    self.publisher_.publish(msg)
            except:
                print("Failed to read line from serial!")
                return
        else:
            print("Serial is invalid!")


def main(args=None):
    """ROS 2 node lifecycle entry point."""
    rclpy.init(args=args)
    minimal_publisher = HopperFullnessNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
