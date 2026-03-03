#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
A ROS2 node to convert CAN messages to Autoware vehicle messages.

This node subscribes to can_msgs.msg.Frame, decodes vehicle information using a DBC file
identified by vehicle type, and publishes vehicle messages (e.g., VelocityReport).
"""

import rclpy
from rclpy.node import Node
from pathlib import Path

import cantools

from can_msgs.msg import Frame
from autoware_vehicle_msgs.msg import VelocityReport


class VehicleInfoConverterNode(Node):
    """
    Converts CAN frame messages to Autoware vehicle messages using DBC-based decoding.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, and CAN database."""
        super().__init__('vehicle_info_converter_node')

        # --- Declare and get parameters ---
        self.declare_parameter('can_topic', '/vehicle/from_can_bus')
        self.declare_parameter('velocity_report_topic', '/vehicle/velocity_report')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('param_root_dir', '/opt/drs/config')
        self.declare_parameter('can_message_name', 'VEHICLE_SPEED_RPT')
        self.declare_parameter('can_signal_name', 'VEHICLE_SPEED')
        self.declare_parameter('vehicle_id', 'default')
        self.declare_parameter('speed_unit_conversion_factor', 1.0)
        # Debug: fixed speed override in m/s (negative value disables override)
        self.declare_parameter('debug_fixed_speed', -1.0)

        self.can_topic: str = self.get_parameter('can_topic').get_parameter_value().string_value
        self.velocity_report_topic: str = self.get_parameter(
            'velocity_report_topic'
        ).get_parameter_value().string_value
        self.frame_id: str = self.get_parameter('frame_id').get_parameter_value().string_value
        self.param_root_dir: str = self.get_parameter(
            'param_root_dir'
        ).get_parameter_value().string_value
        self.can_message_name: str = self.get_parameter(
            'can_message_name'
        ).get_parameter_value().string_value
        self.can_signal_name: str = self.get_parameter(
            'can_signal_name'
        ).get_parameter_value().string_value
        self.vehicle_id: str = self.get_parameter('vehicle_id').get_parameter_value().string_value
        self.speed_unit_conversion_factor: float = self.get_parameter(
            'speed_unit_conversion_factor'
        ).get_parameter_value().double_value

        # Resolve DBC path: $(var param_root_dir)/vehicle.dbc
        self.dbc_file_path = str(Path(self.param_root_dir) / 'vehicle.dbc')


        dbc_path = Path(self.dbc_file_path)
        if not dbc_path.is_file():
            self.get_logger().fatal(f"DBC file not found at: '{self.dbc_file_path}'")
            raise FileNotFoundError(f"DBC file not found at: '{self.dbc_file_path}'")

        try:
            self.can_db = cantools.database.load_file(dbc_path)
            self.can_message = self.can_db.get_message_by_name(self.can_message_name)
            self.target_can_id = self.can_message.frame_id
            self.get_logger().info(f"Successfully loaded DBC file: '{self.dbc_file_path}'")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load or parse DBC file: {e}")
            raise

        self.get_logger().info("--- Vehicle Info Converter Configuration ---")
        self.get_logger().info(f"Vehicle ID: {self.vehicle_id}")
        self.get_logger().info(f"Subscribing to CAN topic: '{self.can_topic}'")
        self.get_logger().info(f"Publishing to: '{self.velocity_report_topic}'")
        self.get_logger().info(f"DBC Message: '{self.can_message_name}' (ID: {self.target_can_id})")
        self.get_logger().info(f"DBC Signal: '{self.can_signal_name}'")
        self.get_logger().info(f"Speed unit conversion factor: {self.speed_unit_conversion_factor}")
        self.get_logger().info("---------------------------------------------")

        # --- Subscriber and Publisher ---
        self.subscription = self.create_subscription(
            Frame,
            self.can_topic,
            self.can_message_callback,
            10,
        )
        self.publisher = self.create_publisher(
            VelocityReport,
            self.velocity_report_topic,
            10,
        )

    def can_message_callback(self, msg: Frame) -> None:
        """
        Process incoming CAN messages, decode speed, and publish VelocityReport.
        """
        if msg.id != self.target_can_id:
            return

        try:
            # Check debug fixed speed override (read dynamically for ros2 param set)
            debug_fixed_speed: float = self.get_parameter(
                'debug_fixed_speed'
            ).get_parameter_value().double_value

            if debug_fixed_speed >= 0.0:
                longitudinal_velocity = debug_fixed_speed
            else:
                decoded_data = self.can_db.decode_message(msg.id, bytes(msg.data))
                raw_speed = float(decoded_data[self.can_signal_name])
                longitudinal_velocity = raw_speed * self.speed_unit_conversion_factor

            velocity_report = VelocityReport()
            velocity_report.header.stamp = self.get_clock().now().to_msg()
            velocity_report.header.frame_id = self.frame_id
            velocity_report.longitudinal_velocity = longitudinal_velocity
            velocity_report.lateral_velocity = 0.0
            velocity_report.heading_rate = 0.0

            self.publisher.publish(velocity_report)
            self.get_logger().debug(
                f"Published VelocityReport: longitudinal_velocity={longitudinal_velocity:.2f} m/s"
                + (" [FIXED]" if debug_fixed_speed >= 0.0 else "")
            )

        except KeyError:
            self.get_logger().warn(
                f"Signal '{self.can_signal_name}' not found in message "
                f"'{self.can_message_name}' (ID: {msg.id}). Check your DBC file."
            )
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during CAN decoding: {e}")


def main(args=None):
    """The main entry point for the node."""
    rclpy.init(args=args)
    node = None
    try:
        node = VehicleInfoConverterNode()
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
