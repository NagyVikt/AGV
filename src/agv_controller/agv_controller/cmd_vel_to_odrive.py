#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from std_msgs.msg import String
import time

class CmdVelToOdriveResetNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_odrive_reset')

        # Declare (and retrieve) parameters.
        self.declare_parameter('wheel_separation', 0.5)
        self.wheel_separation = self.get_parameter('wheel_separation').value

        # Declare a parameter for ramp rate (units per second).
        # Adjust ramp_rate so that you control how fast the commands change.
        self.declare_parameter('ramp_rate', 2.8)  # e.g., 2.8 units per second
        self.ramp_rate = self.get_parameter('ramp_rate').value

        # Publishers for sending control messages to each ODrive axis.
        self.pub_axis0 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.pub_axis1 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)

        # Service clients to change the axis state.
        self.axis0_client = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.axis1_client = self.create_client(AxisState, '/odrive_axis1/request_axis_state')

        # Subscribe to the cmd_vel topic.
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Subscribe to the error topic (published by your C++ CAN interface node).
        self.create_subscription(String, '/odrive_can/errors', self.error_callback, 10)

        # For ramping commands, initialize target and current commands.
        self.target_left_cmd = 0.0
        self.target_right_cmd = 0.0
        self.current_left_cmd = 0.0
        self.current_right_cmd = 0.0

        # Timer for ramp updates (e.g., 50 Hz update rate).
        self.ramp_timer_period = 0.02  # seconds
        self.ramp_timer = self.create_timer(self.ramp_timer_period, self.ramp_callback)

        # Flag to prevent overlapping resets.
        self.reset_in_progress = False

        self.get_logger().info("CmdVelToOdriveReset node has been started with ramping enabled.")

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert incoming Twist messages to left/right velocity commands.
        Instead of publishing directly, we update the target commands.
        """
        linear = msg.linear.x
        angular = msg.angular.z
        half_sep = self.wheel_separation / 2.0

        # Calculate wheel commands based on turning direction.
        if angular >= 0:
            # For a left turn: reduce the left wheel command.
            left_cmd = -linear + (angular * half_sep)
            right_cmd = linear + (angular * half_sep)
        else:
            # For a right turn: further reduce (more negative) the left wheel command.
            left_cmd = -linear - (abs(angular) * half_sep)
            right_cmd = linear - (abs(angular) * half_sep)

        # Update target commands; the ramp_timer will gradually change the current commands.
        self.target_left_cmd = left_cmd
        self.target_right_cmd = right_cmd

        self.get_logger().debug(f"Updated target commands: left={left_cmd}, right={right_cmd}")

    def ramp_callback(self):
        """
        Gradually adjust current wheel commands towards the target commands
        using the specified ramp rate.
        """
        dt = self.ramp_timer_period
        max_step = self.ramp_rate * dt

        # Ramp the left command.
        delta_left = self.target_left_cmd - self.current_left_cmd
        if abs(delta_left) > max_step:
            self.current_left_cmd += max_step if delta_left > 0 else -max_step
        else:
            self.current_left_cmd = self.target_left_cmd

        # Ramp the right command.
        delta_right = self.target_right_cmd - self.current_right_cmd
        if abs(delta_right) > max_step:
            self.current_right_cmd += max_step if delta_right > 0 else -max_step
        else:
            self.current_right_cmd = self.target_right_cmd

        # Create and fill the ControlMessage for the left wheel (axis0).
        ctrl_msg_axis0 = ControlMessage()
        ctrl_msg_axis0.control_mode = 2  # e.g., velocity control mode
        ctrl_msg_axis0.input_mode = 1    # input from velocity command
        ctrl_msg_axis0.input_pos = 0.0     # not used in velocity control
        ctrl_msg_axis0.input_vel = self.current_left_cmd
        ctrl_msg_axis0.input_torque = 0.0

        # Create and fill the ControlMessage for the right wheel (axis1).
        ctrl_msg_axis1 = ControlMessage()
        ctrl_msg_axis1.control_mode = 2  # e.g., velocity control mode
        ctrl_msg_axis1.input_mode = 1    # input from velocity command
        ctrl_msg_axis1.input_pos = 0.0     # not used in velocity control
        ctrl_msg_axis1.input_vel = self.current_right_cmd
        ctrl_msg_axis1.input_torque = 0.0

        # Publish the ramped control messages.
        self.pub_axis0.publish(ctrl_msg_axis0)
        self.pub_axis1.publish(ctrl_msg_axis1)

        self.get_logger().debug(f"Ramped commands published: axis0={self.current_left_cmd}, axis1={self.current_right_cmd}")

    def error_callback(self, msg: String):
        """
        Monitor error messages published by the CAN interface. If the error indicates
        a failure to send a CAN frame, trigger a wheel reset.
        """
        if "Failed to send CAN frame" in msg.data:
            self.get_logger().warn("Error received: " + msg.data)
            self.reset_wheels()

    def reset_wheels(self):
        """
        Reset the ODrive wheels by setting the axes first to idle (state 1) and then
        back to closed-loop control (state 8). Adjust the state values as needed.
        """
        if self.reset_in_progress:
            self.get_logger().info("Wheel reset already in progress. Ignoring duplicate request.")
            return

        self.reset_in_progress = True
        self.get_logger().info("Initiating wheel reset due to CAN frame send failure...")

        # --- Step 1: Set both axes to idle.
        request_idle = AxisState.Request()
        request_idle.axis_requested_state = 1  # Idle state (example value)

        future0 = self.axis0_client.call_async(request_idle)
        future1 = self.axis1_client.call_async(request_idle)
        rclpy.spin_until_future_complete(self, future0)
        rclpy.spin_until_future_complete(self, future1)

        if future0.result() is not None:
            self.get_logger().info("Axis0 set to IDLE.")
        else:
            self.get_logger().error("Failed to set Axis0 to IDLE.")
        if future1.result() is not None:
            self.get_logger().info("Axis1 set to IDLE.")
        else:
            self.get_logger().error("Failed to set Axis1 to IDLE.")

        # Wait briefly for the hardware to process the state change.
        time.sleep(1.0)

        # --- Step 2: Restore both axes to closed-loop control.
        request_closed = AxisState.Request()
        request_closed.axis_requested_state = 8  # Closed-loop control state (example value)

        future0 = self.axis0_client.call_async(request_closed)
        future1 = self.axis1_client.call_async(request_closed)
        rclpy.spin_until_future_complete(self, future0)
        rclpy.spin_until_future_complete(self, future1)

        if future0.result() is not None:
            self.get_logger().info("Axis0 set to CLOSED-LOOP CONTROL.")
        else:
            self.get_logger().error("Failed to set Axis0 to CLOSED-LOOP CONTROL.")
        if future1.result() is not None:
            self.get_logger().info("Axis1 set to CLOSED-LOOP CONTROL.")
        else:
            self.get_logger().error("Failed to set Axis1 to CLOSED-LOOP CONTROL.")

        self.get_logger().info("Wheel reset complete. Resuming normal operation.")
        self.reset_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToOdriveResetNode()

    # --- Wait for the AxisState services to become available.
    node.get_logger().info("Waiting for /odrive_axis0/request_axis_state service...")
    node.axis0_client.wait_for_service()
    node.get_logger().info("Axis0 service available.")

    node.get_logger().info("Waiting for /odrive_axis1/request_axis_state service...")
    node.axis1_client.wait_for_service()
    node.get_logger().info("Axis1 service available.")

    # --- Set the initial operating state to CLOSED-LOOP CONTROL (state 8 in this example).
    request_closed = AxisState.Request()
    request_closed.axis_requested_state = 8  # Adjust if your system uses a different value.
    future0 = node.axis0_client.call_async(request_closed)
    future1 = node.axis1_client.call_async(request_closed)
    rclpy.spin_until_future_complete(node, future0)
    rclpy.spin_until_future_complete(node, future1)
    if future0.result() is not None:
        node.get_logger().info("Axis0 is in CLOSED-LOOP CONTROL.")
    else:
        node.get_logger().error("Failed to set Axis0 to CLOSED-LOOP CONTROL.")
    if future1.result() is not None:
        node.get_logger().info("Axis1 is in CLOSED-LOOP CONTROL.")
    else:
        node.get_logger().error("Failed to set Axis1 to CLOSED-LOOP CONTROL.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
