#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
A ROS2 node to convert VelocityReport to OxTS GAD packets and send via UDP.

This node subscribes to autoware_vehicle_msgs/msg/VelocityReport, constructs
Generic Aiding Data (GAD) packets using the oxts_sdk, and sends them to an
OxTS GNSS/INS unit (e.g., AV200) via UDP for velocity aiding.

NOTE: This node requires the following Python package:
- oxts-sdk-py (pip install oxts-sdk-py)
"""

import rclpy
from rclpy.node import Node
import ipaddress
import threading

import oxts_sdk

from autoware_vehicle_msgs.msg import VelocityReport


class OxTSGadSenderNode(Node):
    """
    Subscribes to VelocityReport and forwards speed as GAD packets to OxTS device.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, and GAD handler."""
        super().__init__('oxts_gad_sender_node')

        # --- Member variables ---
        self.latest_speed_ms: float | None = None
        self.lock = threading.Lock()

        # --- Declare and get parameters ---
        self.declare_parameter('velocity_report_topic', '/vehicle/velocity_report')
        self.declare_parameter('ins_ip', '192.168.4.250')
        self.declare_parameter('stream_id', 157)
        self.declare_parameter('gad_speed_std_dev', 0.1)
        self.declare_parameter('gad_latency', 0.0)
        self.declare_parameter('aiding_lever_arm', [-0.6895, 0.0, -1.9705])
        self.declare_parameter('publish_rate_hz', 10.0)

        self.velocity_report_topic: str = self.get_parameter(
            'velocity_report_topic'
        ).get_parameter_value().string_value
        self.ins_ip: str = self.get_parameter('ins_ip').get_parameter_value().string_value
        self.stream_id: int = self.get_parameter('stream_id').get_parameter_value().integer_value
        self.gad_speed_std_dev: float = self.get_parameter(
            'gad_speed_std_dev'
        ).get_parameter_value().double_value
        self.gad_latency: float = self.get_parameter('gad_latency').get_parameter_value().double_value
        self.aiding_lever_arm: list[float] = self.get_parameter(
            'aiding_lever_arm'
        ).get_parameter_value().double_array_value
        self.publish_rate_hz: float = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value

        # --- Initialize OxTS GAD Handler ---
        self.gad_handler = oxts_sdk.GadHandler()

        if self.ins_ip.lower() == 'csv':
            self.get_logger().info(
                "Output mode set to CSV. GAD packets will be saved to 'gad_info.csv'."
            )
            self.gad_handler.set_encoder_to_csv()
            self.gad_handler.set_output_mode_to_file("gad_info.csv")
        else:
            try:
                ipaddress.ip_address(self.ins_ip)
                self.get_logger().info(
                    f"Output mode set to UDP. Sending GAD packets to: {self.ins_ip}"
                )
                self.gad_handler.set_encoder_to_bin()
                self.gad_handler.set_output_mode_to_udp(self.ins_ip)
            except ValueError:
                self.get_logger().fatal(
                    f"Invalid IP address provided for 'ins_ip': '{self.ins_ip}'. "
                    "Please provide a valid IP or 'csv'."
                )
                raise ValueError(f"Invalid IP address: {self.ins_ip}")

        self.get_logger().info("--- OxTS GAD Sender Configuration ---")
        self.get_logger().info(f"Subscribing to: '{self.velocity_report_topic}'")
        if self.ins_ip.lower() == 'csv':
            self.get_logger().info("Output mode: CSV file (gad_info.csv)")
        else:
            self.get_logger().info(f"Sending GAD UDP packets to: {self.ins_ip}")
        self.get_logger().info(f"GAD Stream ID: {self.stream_id}")
        self.get_logger().info(f"Aiding Lever Arm: {self.aiding_lever_arm}")
        self.get_logger().info("------------------------------------")

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            VelocityReport,
            self.velocity_report_topic,
            self.velocity_report_callback,
            10,
        )

        # --- Timer for sending GAD packets ---
        timer_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0 else 0.1
        self.gad_timer = self.create_timer(timer_period, self.send_gad_packet_callback)

    def velocity_report_callback(self, msg: VelocityReport) -> None:
        """
        Store the latest longitudinal velocity from VelocityReport.
        """
        with self.lock:
            self.latest_speed_ms = float(msg.longitudinal_velocity)

    def send_gad_packet_callback(self) -> None:
        """
        Called by timer to send the latest received speed as a GAD packet.
        """
        with self.lock:
            if self.latest_speed_ms is None:
                self.get_logger().debug("No velocity data received yet, skipping GAD packet.")
                return
            speed_to_send = self.latest_speed_ms

        try:
            gad_speed = oxts_sdk.GadSpeed(self.stream_id)
            gad_speed.speed_fw_ms = speed_to_send
            gad_speed.speed_ms_var = self.gad_speed_std_dev ** 2
            gad_speed.set_time_void()
            gad_speed.aiding_lever_arm_fixed = self.aiding_lever_arm
            gad_speed.aiding_lever_arm_var = [0.01, 0.01, 0.01]

            self.gad_handler.send_packet(gad_speed)
            self.get_logger().debug(
                f"Sent GAD speed packet with speed {speed_to_send:.2f} m/s."
            )

        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during GAD sending: {e}")


def main(args=None):
    """The main entry point for the node."""
    rclpy.init(args=args)
    node = None
    try:
        node = OxTSGadSenderNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as e:
        if isinstance(e, Exception) and node:
            node.get_logger().fatal(f"Unhandled exception in spin: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
