#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Timer

from std_msgs.msg import Float32
import serial, sys, glob


# this is just code I took that find what port the arduino is connected
def serial_ports() -> list[str]:
    try: 
        ports = glob.glob("/dev/serial/by-id/*1a86*")
    except:
        ports = []
        raise EnvironmentError("Unsupported platform")

    return ports

# this is just a publish I stole the code from most of it I left the same but I added code to timer_callback
class MinimalPublisher(Node):
    publisher_: Publisher
    timer: Timer
    i: int
    serial_port: serial.Serial

    __slots__ = ("i", "timer", "publisher_", "serial_port")

    def __get_data(self) -> str:
        return self.serial_port.readline().decode().strip()

    def __init__(self, serial_port: str | None = None):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(Float32, "lance/hopper_fullness", 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        if serial_port is None:
            while True:
                try:
                    self.serial_port = serial.Serial(serial_ports()[0], 9600)
                    break
                except (serial.SerialException, IndexError):
                    continue
        elif isinstance(serial_port, str):
            self.serial_port = serial.Serial(serial_port, 9600)
        else:
            raise ValueError(f"parameter serial_port: {serial_port} is an invalid type")
        self.serial_port.read_until()

    def timer_callback(self):
        msg = Float32()

        while True:
            try:
                line = self.__get_data()
                break
            except serial.serialutil.SerialException:
                serPorts = serial_ports()
                if len(serPorts) > 0:
                    self.serial_port = serial.Serial(serPorts[0], 9600)
                    continue
                else:
                    return

        if not "Fs: " in line:
            return

        Dout = float(line.replace("Fs: ", ""))

        print(Dout, line)

        msg.data = Dout
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    print(serial_ports())
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
