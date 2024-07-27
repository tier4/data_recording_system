"""
Set C2 readout delay after confirming camera stream & trigger signal emission
This node assumes that only 1 C2 camera is connected to 1 deserializer
"""
import sys
import subprocess
import re
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from ament_index_python.packages import get_package_share_directory

import sensor_msgs.msg

class MovingAverageStatistics:
    # ref: https://github.com/ros-tooling/libstatistics_collector/blob/rolling/src/libstatistics_collector/moving_average_statistics/moving_average.cpp
    def __init__(self):
        self.__lock = threading.Lock()
        self.__average = 0
        self.__count = 0

    def reset(self):
        with self.__lock:
            self.__average = 0
            self.__count = 0

    def add_measurement(self, item: float):
        with self.__lock:
            self.__count += 1
            self.__average = self.__average + (item - self.__average) / self.__count

    @property
    def average(self):
        with self.__lock:
            return self.__average

    @property
    def count(self):
        with self.__lock:
            return self.__count

class ReadoutDelaySetter(Node):
    def __init__(self):
        super().__init__('c2_readout_delay_setter' )

        self.declare_parameter('delay_ms', rclpy.Parameter.Type.INTEGER)
        self.delay_ms = self.get_parameter('delay_ms').value

        self.declare_parameter('eval_topic_num', 10)
        self.eval_topic_num = self.get_parameter('eval_topic_num').value

        self.declare_parameter('expected_hz', 20)
        expected_hz = self.get_parameter('expected_hz').value
        # Set lower/upper bound of topic hz as +- 20%
        self.topic_rate_lower = expected_hz * 0.80
        self.topic_rate_upper = expected_hz * 1.20

        # Get target camera's i2c bus ID from node name
        self.declare_parameter('target_v4l2_node', rclpy.Parameter.Type.STRING)
        target_node_name = self.get_parameter('target_v4l2_node').value
        video_device = self.__get_video_device_from_node_name(target_node_name)
        self.i2c_bus_id = self.__get_i2c_bus_from_video_device(video_device)

        self.topic_statistics = MovingAverageStatistics()

        # Query QoS using timer so that constructor does not get stuck by the query
        self._timer = self.create_timer(0.1, self.__qos_query_callback)

    def __qos_query_callback(self):
        # Start subscriptions using proper QoS
        camera_info_topic = self.resolve_topic_name('camera_info')
        camera_info_qos = self.__get_qos_by_name(camera_info_topic)
        if camera_info_qos is None:
            return
        self.prev_now = self.get_clock().now().nanoseconds
        self.create_subscription(sensor_msgs.msg.CameraInfo,
                                 camera_info_topic, self.__callback, camera_info_qos)

        # Once proper Qos is acquired, stop the timer
        self._timer.cancel()

    def __get_video_device_from_node_name(self, target_node_name):
        # Ask the node which video device was specified to use
        param_client = self.create_client(GetParameters, target_node_name + '/get_parameters')
        request = GetParameters.Request()
        request.names = ['video_device']
        future = param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        video_device = future.result().values[0].string_value
        self.get_logger().info(f'request got response: video_device={video_device}')
        return video_device

    def __get_i2c_bus_from_video_device(self, video_device):
        # Get bus related information exploiting `v4l2-ctl`
        cmd = f'v4l2-ctl -d {video_device} --info | grep -i "Card type"'
        cmd_return = subprocess.run(cmd, shell=True, capture_output=True, text=True).stdout.strip()

        # Parse command result to get i2c bus ID
        i2c_bus_str = re.match('.*(\d\d)-(\w+)', cmd_return).group(1)
        self.get_logger().info(f'i2c bus determined: {i2c_bus_str}')
        return i2c_bus_str

    def __get_qos_by_name(self, topic_name):
        qos_list = self.get_publishers_info_by_topic(topic_name)
        if len(qos_list) < 1:
            self.get_logger().info('Waiting for ' + topic_name,
                                   throttle_duration_sec = 2.0)
            return None
        elif len(qos_list) > 1:
            self.get_logger().error('Multiple publisher for ' + topic_name + ' are detected. '
                                    'Cannot determine proper QoS')
            sys.exit(1)     # exit with error status
        else:
            self.get_logger().info('QoS for ' + topic_name + ' is acquired.')
            return qos_list[0].qos_profile

    def __callback(self, msg: sensor_msgs.msg.CameraInfo):
        now = self.get_clock().now().nanoseconds
        interval_sec = (now - self.prev_now) / 10**9
        self.prev_now = now

        self.topic_statistics.add_measurement(interval_sec)
        current_average = 1. / self.topic_statistics.average  # Hz
        if (self.topic_rate_lower < current_average
            and current_average < self.topic_rate_upper
            and self.topic_statistics.count == self.eval_topic_num):
            # Pass the evaluation = streaming was started. Set the readout delay and terminate the node
            script_path = Path(get_package_share_directory('c2_readout_delay_setter')) \
                / 'scripts' / 'set_readout_delay.sh'
            script_path = script_path.resolve().absolute()
            proc = subprocess.run(f'{script_path} {self.delay_ms} {self.i2c_bus_id}',
                                  shell=True, capture_output=False)

            # Once set the read out delay, terminate the node
            raise SystemExit
        else:
            if self.topic_statistics.count > self.eval_topic_num:
                # reset the statistics to see the latest topic rate
                self.topic_statistics.reset()

def main(args=None):
    rclpy.init(args=args)
    node = ReadoutDelaySetter()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
