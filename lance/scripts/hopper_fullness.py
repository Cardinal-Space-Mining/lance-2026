#!/usr/bin/env python3

import glob, time
from serial import Serial

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32


def serial_ports() -> list[str]:
    try: 
        return glob.glob("/dev/serial/by-id/*1a86*")
    except:
        return []

def make_serial(port : str) -> Serial | None:
    try:
        return Serial(port, 9600)
    except:
        return None

def wait_for_serial_port(attempts = 1, sleep_time = 1) -> Serial | None:
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
    publisher_: Publisher
    timer: Timer
    serial: Serial

    __slots__ = ("timer", "publisher_", "serial")

    def __get_data(self) -> str:
        return self.serial.readline().decode().strip()

    def __init__(self, serial_port: str | None = None):
        super().__init__("hopper_fullness_driver")
        self.publisher_ = self.create_publisher(Float32, 
                                                "lance/hopper_fullness",
                                                qos_profile_sensor_data)
        self.timer = self.create_timer(1, self.timer_callback)
        self.serial = None

        if isinstance(serial_port, str):
            self.serial = make_serial(serial_port)

        if self.serial is None:
            self.serial = wait_for_serial_port(1)

        if self.serial is None:
            print('Failed to initialize valid serial port!')
        else:
            self.serial.read_until()

    def timer_callback(self):
        if self.serial is None:
            self.serial = wait_for_serial_port(1)

        if self.serial is not None:
            try:
                line = self.__get_data()
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
    rclpy.init(args=args)
    minimal_publisher = HopperFullnessNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
