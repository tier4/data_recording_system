#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
A ROS2 node to parse vehicle speed from CAN messages and send it to an OxTS device.

This node uses the 'cantools' library to decode CAN messages based on a DBC file,
extracts a specified signal, and then uses the 'oxts_sdk' to construct and
send a Generic Aiding Data (GAD) packet to a configured OxTS GNSS/INS unit.

NOTE: This node requires the following Python packages:
- oxts-sdk-py (pip install oxts-sdk-py)
- cantools (pip install cantools)
"""

import rclpy
from rclpy.node import Node
from pathlib import Path
import ipaddress
import threading

# The oxts_sdk handles GAD packet creation and transmission.
import oxts_sdk
# The cantools library handles DBC-based CAN message decoding.
import cantools

from can_msgs.msg import Frame
from sensor_msgs.msg import NavSatFix
from rclpy.parameter import Parameter

class GADConverterNode(Node):
    """
    Parses CAN signals using a DBC file and forwards them as GAD packets.
    """

    def __init__(self) -> None:
        """Initialize the node, parameters, CAN database, and GAD handler."""
        super().__init__('gad_converter_node')

        # --- Member variables ---
        self.latest_speed_ms: float | None = None
        self.lock = threading.Lock()
        
        # --- Declare and get parameters ---
        self.declare_parameter('can_topic', '/can/rx')
        self.declare_parameter('ins_ip', '192.168.1.10') # Changed from oxts_ip
        self.declare_parameter('stream_id', 157)
        self.declare_parameter('gad_speed_std_dev', 0.1)
        self.declare_parameter('gad_latency', 0.0)
        self.declare_parameter('dbc_file_path', '')
        self.declare_parameter('can_message_name', 'CAN_MSG_VEHICLE_SPEED')
        self.declare_parameter('can_signal_name', 'VEHICLE_SPEED')
        self.declare_parameter('aiding_lever_arm', [-0.6895, 0.0, -1.9705])

        self.can_topic: str = self.get_parameter('can_topic').get_parameter_value().string_value
        self.ins_ip: str = self.get_parameter('ins_ip').get_parameter_value().string_value # Changed from oxts_ip
        self.stream_id: int = self.get_parameter('stream_id').get_parameter_value().integer_value
        self.gad_speed_std_dev: float = self.get_parameter('gad_speed_std_dev').get_parameter_value().double_value
        self.gad_latency: float = self.get_parameter('gad_latency').get_parameter_value().double_value
        self.dbc_file_path: str = self.get_parameter('dbc_file_path').get_parameter_value().string_value
        self.can_message_name: str = self.get_parameter('can_message_name').get_parameter_value().string_value
        self.can_signal_name: str = self.get_parameter('can_signal_name').get_parameter_value().string_value
        self.aiding_lever_arm: list[float] = self.get_parameter('aiding_lever_arm').get_parameter_value().double_array_value

        # --- Load CAN Database (.dbc file) ---
        if not self.dbc_file_path:
            self.get_logger().fatal("Parameter 'dbc_file_path' is not set. Please provide a path to your .dbc file.")
            raise ValueError("DBC file path is required.")
        
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

        # --- Initialize OxTS GAD Handler based on ins_ip ---
        self.gad_handler = oxts_sdk.GadHandler()
        
        if self.ins_ip.lower() == 'csv':
            # CSV mode
            self.get_logger().info("Output mode set to CSV. GAD packets will be saved to 'gad_info.csv'.")
            self.gad_handler.set_encoder_to_csv()
            self.gad_handler.set_output_mode_to_file("gad_info.csv")
        else:
            try:
                # Check IP address
                ipaddress.ip_address(self.ins_ip)
                self.get_logger().info(f"Output mode set to UDP. Sending GAD packets to: {self.ins_ip}")
                self.gad_handler.set_encoder_to_bin()
                self.gad_handler.set_output_mode_to_udp(self.ins_ip)
            except ValueError:
                # Invlid ip address
                self.get_logger().fatal(f"Invalid IP address provided for 'ins_ip': '{self.ins_ip}'. Please provide a valid IP or 'csv'.")
                raise ValueError(f"Invalid IP address: {self.ins_ip}")
        
        self.get_logger().info("OxTS GAD Handler initialized successfully.")

        self.get_logger().info("--- GAD Converter Node Configuration ---")
        self.get_logger().info(f"Subscribing to CAN topic: '{self.can_topic}'")
        self.get_logger().info(f"DBC Message: '{self.can_message_name}' (ID: {self.target_can_id})")
        self.get_logger().info(f"DBC Signal: '{self.can_signal_name}'")
        if self.ins_ip.lower() == 'csv':
            self.get_logger().info("Output mode: CSV file (gad_info.csv)")
        else:
            self.get_logger().info(f"Sending GAD UDP packets to: {self.ins_ip}")
        self.get_logger().info(f"Using GAD Stream ID: {self.stream_id}")
        self.get_logger().info(f"Aiding Lever Arm: {self.aiding_lever_arm}")
        self.get_logger().info("------------------------------------")
        
        # --- Initialize subscriber ---
        self.subscription = self.create_subscription(
            Frame,
            self.can_topic,
            self.can_message_callback,
            10)

        # --- Initialize 10Hz timer for sending GAD packets ---
        self.gad_timer = self.create_timer(0.1, self.send_gad_packet_callback) # 10Hz timer

    def can_message_callback(self, msg: Frame) -> None:
        """
        Process incoming CAN messages, decode speed, and store it.
        """
        if msg.id != self.target_can_id:
            return

        try:
            # --- Decode CAN message using cantools ---
            decoded_data = self.can_db.decode_message(msg.id, bytes(msg.data))
            speed_ms = decoded_data[self.can_signal_name]

            self.get_logger().info(f"Decoded speed: {speed_ms:.2f} m/s from signal '{self.can_signal_name}'")

            # --- Store the latest speed for the timer to send ---
            with self.lock:
                self.latest_speed_ms = speed_ms

        except KeyError:
            self.get_logger().warn(
                f"Signal '{self.can_signal_name}' not found in message '{self.can_message_name}' (ID: {msg.id}). Check your DBC file.")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during CAN decoding: {e}")

    def send_gad_packet_callback(self) -> None:
        """
        Called by a 10Hz timer to send the latest received speed data.
        """
        with self.lock:
            if self.latest_speed_ms is None:
                self.get_logger().debug("No speed data received yet, skipping GAD packet.")
                return
            # Create a local copy to use outside the lock
            speed_to_send = self.latest_speed_ms
        
        try:
            # --- Generate and Send GAD Packet using oxts_sdk ---
            gad_speed = oxts_sdk.GadSpeed(self.stream_id)
            # Set a forwards-positive speed measurement in m/s
            gad_speed.speed_fw_ms = speed_to_send
            # The SDK expects variance, which is the square of the standard deviation.
            gad_speed.speed_ms_var = self.gad_speed_std_dev ** 2
            # Timestamp setting, timestamp set by INS with latency
            gad_speed.set_time_void()
            # gad_speed.time_latency = self.gad_latency
            # Set the aiding lever arm from parameters
            gad_speed.aiding_lever_arm_fixed = self.aiding_lever_arm
            gad_speed.aiding_lever_arm_var = [0.01, 0.01, 0.01]

            self.gad_handler.send_packet(gad_speed)
            self.get_logger().debug(f"Sent GAD speed packet with speed {speed_to_send:.2f} m/s.")

        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during GAD sending: {e}")

def main(args=None):
    """The main entry point for the node."""
    rclpy.init(args=args)
    node = None
    try:
        node = GADConverterNode()
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
