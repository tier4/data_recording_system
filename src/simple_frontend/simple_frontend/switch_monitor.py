"""
Entry point of the ROS node (switch monitor)
NOTE: This code assumes python3-gpiod==1.5.4
"""
import rclpy
from rclpy.node import Node
import std_msgs.msg

from dataclasses import dataclass
import gpiod
import threading
from datetime import timedelta

from .switch_monitor_base import SwitchMonitorBase


@dataclass
class GpioSpecifier:
    device_name: str
    line: int

    def __init__(self, chip_number : int, line: int ):
        self.device_name = f'gpiochip{chip_number}'
        self.line = line

@dataclass(frozen=True)
class Gpio:
    # Anvil GP_IN_1, PA.02 (chip 0, line 2)
    signal_in: GpioSpecifier = GpioSpecifier(chip_number=0,
                                             line=2)
    # Anvil GP_OUT_1, PAC.05 (chip 0, line 143)
    power_supply: GpioSpecifier =  GpioSpecifier(chip_number=0,
                                                 line=143)

class SwitchMonitor(Node, SwitchMonitorBase):
    def __init__(self):
        Node.__init__(self, 'switch_monitor')

        self.long_push_sec_threshold = self.declare_parameter(name='long_push_sec_threshold',
                                                              value=5).value

        self.rise_edge_pub = self.create_publisher(std_msgs.msg.Bool, 'switch_push_detected', 10)
        self.fall_edge_pub = self.create_publisher(std_msgs.msg.Bool, 'switch_release_detected', 10)

        # Start power supply from GPIO_OUT_1 to the switch
        self.power_supply = gpiod.chip(Gpio.power_supply.device_name).get_line(Gpio.power_supply.line)
        config = gpiod.line_request()
        config.consumer = 'power_supply'
        config.request_type = gpiod.line_request.DIRECTION_OUTPUT
        self.power_supply.request(config)
        self.power_supply.set_value(1)

        # Exit thread when the main thread is terminated
        self.watch_thread = threading.Thread(target=self.monitor_switch, daemon=True)
        self.watch_thread.start()

    def __del__(self):
        # Stop power supply from GPIO_OUT_1
        self.power_supply.set_value(0)

    def monitor_switch(self):
        signal_in = gpiod.chip(Gpio.signal_in.device_name).get_line(Gpio.signal_in.line)
        config = gpiod.line_request()
        config.consumer = 'signal_in'
        config.request_type = gpiod.line_request.EVENT_BOTH_EDGES
        signal_in.request(config)

        previous_event = None
        event_time = self.get_clock().now()

        while True:
            if signal_in.event_wait(timedelta(seconds=1)):
                event = signal_in.event_read()
                if event.event_type == gpiod.line_event.RISING_EDGE:
                    print('rising sun')
                    if previous_event != gpiod.line_event.RISING_EDGE:
                        event_time = self.get_clock().now()
                        previous_event = gpiod.line_event.RISING_EDGE
                        msg = std_msgs.msg.Bool()
                        msg.data = True
                        self.rise_edge_pub.publish(msg)

                elif event.event_type == gpiod.line_event.FALLING_EDGE:
                    print('fallen')
                    if previous_event != gpiod.line_event.RISING_EDGE:
                        continue

                    now = self.get_clock().now()
                    elapsed_in_sec = (now.nanoseconds - event_time.nanoseconds) / (10**9)

                    previous_event = gpiod.line_event.FALLING_EDGE

                    if elapsed_in_sec < self.long_push_sec_threshold:
                        # Short push: restart service
                        print(f'short press: {elapsed_in_sec}')
                        self.restart_drs_launch_services()
                        self.restart_drs_rosbag_record_services()
                    else:
                        # Long push: system shutdown
                        print(f'long press: {elapsed_in_sec}')
                        self.shutdown_drs_components()
                    msg = std_msgs.msg.Bool()
                    msg.data = True
                    self.fall_edge_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwitchMonitor()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import pdb, traceback, sys
    try:
        main()
    except:
        extype, value, tb = sys.exc_info()
        traceback.print_exc()
        pdb.post_mortem(tb)


