#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from tf2_msgs.msg import TFMessage


class BagTFStaticPublisher(Node):
    def __init__(self, bag_path):
        super().__init__("bag_tf_static_publisher")

        tf_static_data = self.read_tf_static_from_bag(bag_path)

        if tf_static_data is None:
            self.get_logger().error(f"No /tf_static found in bag file: {bag_path}")
            sys.exit(1)

        self.tf_static_data = tf_static_data

        static_broadcaster_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.tf_static_publisher = self.create_publisher(
            TFMessage, "/tf_static", static_broadcaster_qos
        )

        self.get_logger().info(
            f"Publishing {len(self.tf_static_data.transforms)} static transforms from bag"
        )
        for transform in self.tf_static_data.transforms:
            self.get_logger().info(
                f"  {transform.header.frame_id} -> {transform.child_frame_id}"
            )

        self.tf_static_publisher.publish(self.tf_static_data)
        self.get_logger().info("Published /tf_static with TRANSIENT_LOCAL QoS")

    def read_tf_static_from_bag(self, bag_path):
        try:
            storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
            converter_options = ConverterOptions("", "")
            reader = SequentialReader()

            reader.open(storage_options, converter_options)

            topic_types = reader.get_all_topics_and_types()
            type_map = {
                topic_types[i].name: topic_types[i].type
                for i in range(len(topic_types))
            }

            if "/tf_static" not in type_map:
                self.get_logger().error("/tf_static topic not found in bag")
                return None

            storage_filter = StorageFilter(topics=["/tf_static"])
            reader.set_filter(storage_filter)

            while reader.has_next():
                (topic, data, _) = reader.read_next()
                if topic == "/tf_static":
                    msg_type = TFMessage
                    tf_static_data = deserialize_message(data, msg_type)
                    self.get_logger().info(
                        f"Found /tf_static with {len(tf_static_data.transforms)} transforms"
                    )
                    return tf_static_data

        except Exception as e:
            self.get_logger().error(f"Error reading bag file: {e}")
            return None

        return None


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Publish /tf_static from a ROS 2 bag file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  %(prog)s sample.mcap
  %(prog)s /path/to/your/bag.mcap
        """,
    )

    parser.add_argument("bag_path", help="Path to the ROS 2 bag file (MCAP format)")

    parsed_args = parser.parse_args()

    bag_path = parsed_args.bag_path

    print(f"Starting bag_tf_static_publisher with bag file: {bag_path}")

    rclpy.init(args=args)

    try:
        node = BagTFStaticPublisher(bag_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
