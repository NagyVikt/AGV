#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from odrive_can.msg import ControlMessage  # Import the custom ControlMessage

import serial
import time

speed = 2

class SerialReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader_node')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout_duration', 0.5)
        self.declare_parameter('alpha', 0.2)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout_duration = self.get_parameter('timeout_duration').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value

        # Initialize serial connection
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'Opened serial port: {self.port} at {self.baudrate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            raise

        # Initialize variables for left and right wheels
        self.last_data_time = time.time()
        self.ema_left = 0.0
        self.ema_right = 0.0
        self.initialized = False

        # Publishers for ControlMessage
        self.control_publisher_axis0 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.control_publisher_axis1 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)

        # Timer for reading serial data
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def map_pwm_to_value(self, width):
        if width < 1100:
            width = 1100
        elif width > 1900:
            width = 1900

        if 1450 <= width <= 1550:
            return 0.0
        elif width < 1450:
            mapped_value = (1450 - width) / 350 * speed
            return round(max(-speed, min(speed, mapped_value)), 2)
        else:
            mapped_value = (width - 1550) / 350 * -speed
            return round(max(-speed, min(speed, mapped_value)), 2)

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').rstrip()
        except UnicodeDecodeError:
            self.get_logger().warning('Received undecodable bytes from serial.')
            return

        current_time = time.time()

        if line:
            self.last_data_time = current_time
            try:
                width1_str, width2_str = line.split(',')
                width1 = int(width1_str)
                width2 = int(width2_str)

                mapped_value1 = self.map_pwm_to_value(width1)
                mapped_value2 = self.map_pwm_to_value(width2)

                if not self.initialized:
                    self.ema_left = mapped_value1
                    self.ema_right = mapped_value2
                    self.initialized = True
                else:
                    self.ema_left = self.alpha * mapped_value1 + (1 - self.alpha) * self.ema_left
                    self.ema_right = self.alpha * mapped_value2 + (1 - self.alpha) * self.ema_right

            except ValueError:
                self.get_logger().warning(f'Invalid data format: "{line}"')
        else:
            if current_time - self.last_data_time > self.timeout_duration:
                self.ema_left = 0.0
                self.ema_right = 0.0
                self.initialized = False

        # Prepare and publish ControlMessage for axis0 (left wheel)
        control_msg_axis0 = ControlMessage()
        control_msg_axis0.control_mode = 2
        control_msg_axis0.input_mode = 1
        control_msg_axis0.input_pos = 0.0
        control_msg_axis0.input_vel = round(self.ema_left, 2)
        control_msg_axis0.input_torque = 0.0

        self.control_publisher_axis0.publish(control_msg_axis0)

        # Prepare and publish ControlMessage for axis1 (right wheel)
        control_msg_axis1 = ControlMessage()
        control_msg_axis1.control_mode = 2
        control_msg_axis1.input_mode = 1
        control_msg_axis1.input_pos = 0.0
        control_msg_axis1.input_vel = round(self.ema_right, 2)
        control_msg_axis1.input_vel *= -1
        control_msg_axis1.input_torque = 0.0

        self.control_publisher_axis1.publish(control_msg_axis1)

        self.get_logger().debug(f"Published ControlMessage Axis0: {control_msg_axis0}")
        self.get_logger().debug(f"Published ControlMessage Axis1: {control_msg_axis1}")

    def destroy_node(self):
        self.ser.close()
        self.get_logger().info('Closed serial port.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialReaderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
