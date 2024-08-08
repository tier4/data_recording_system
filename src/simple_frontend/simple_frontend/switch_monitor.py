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
from os.path import expanduser
from pathlib import Path
import subprocess

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

@dataclass
class RemoteHost:
    ip: str
    user: str

class SwitchMonitor(Node):
    def __init__(self):
        super().__init__('switch_monitor')

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
                        self.__restart_drs_launch_services()
                        self.__restart_drs_rosbag_record_services()
                    else:
                        # Long push: system shutdown
                        print(f'long press: {elapsed_in_sec}')
                        self.__shutdown_drs_components()
                    msg = std_msgs.msg.Bool()
                    msg.data = True
                    self.fall_edge_pub.publish(msg)

    def __generate_ssh_command(self, ip: str, user: str, cmd: str):
        # suppress prompt to trust hosts for the first connection
        ssh_option = 'StrictHostKeyChecking=no'

        # The key should be registered during setup
        ssh_key = Path(expanduser('~'))/'.ssh'/'drs_rsa'

        return f'ssh -o {ssh_option} -i {ssh_key} {user}@{ip} {cmd}'

    def __restart_drs_launch_services(self):
        """
        Very rough implementation to restart drs_launch.service on each ECU
        """
        # The command should be registerd in /etc/sudoers.d/ so that the user can execute it
        # without password
        restart_cmd = 'sudo systemctl restart drs_launch.service'

        targets = [
            RemoteHost(ip='192.168.20.2', user='nvidia'),  # ECU#1
            RemoteHost(ip='192.168.20.1', user='nvidia'),  # ECU#0
        ]

        for t in targets:
            cmd = self.__generate_ssh_command(t.ip, t.user, restart_cmd)
            subprocess.Popen(cmd, shell=True)  # execute in background

    def __restart_drs_rosbag_record_services(self):
        """
        Very rough implementation to restart drs_rosbag_record.service on each ECU
        """
        # The command should be registerd in /etc/sudoers.d/ so that the user can execute it
        # without password
        restart_cmd = 'sudo systemctl restart drs_rosbag_record.service'

        targets = [
            RemoteHost(ip='192.168.20.2', user='nvidia'),  # ECU#1
            RemoteHost(ip='192.168.20.1', user='nvidia'),  # ECU#0
        ]

        for t in targets:
            cmd = self.__generate_ssh_command(t.ip, t.user, restart_cmd)
            subprocess.Popen(cmd, shell=True)  # execute in background

    def __shutdown_drs_components(self):
        """
        Very rough implementation to shutdown each ECU components gently
        """
        # The command should be registerd in /etc/sudoers.d/ so that the user can execute it
        # without password
        poweroff_cmd = 'sudo poweroff'

        targets = [
            RemoteHost(ip='192.168.20.2', user='nvidia'),  # ECU#1
            RemoteHost(ip='192.168.10.100', user='comlops'),  # NAS
            RemoteHost(ip='192.168.20.1', user='nvidia'),  # ECU#0, this entry have to come at the very last!
        ]

        for t in targets:
            cmd = self.__generate_ssh_command(t.ip, t.user, poweroff_cmd)
            subprocess.run(cmd, shell=True, capture_output=False)

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


