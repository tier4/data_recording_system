import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory

from PySide2.QtWidgets import (QPushButton, QApplication, QGridLayout, QWidget)
from PySide2.QtGui import QIcon, QPixmap, QPainter
from PySide2.QtSvg import QSvgRenderer
from PySide2.QtCore import QSize

from .switch_monitor_base import SwitchMonitorBase


class SoftwareSwitchMonitor(Node, SwitchMonitorBase):
    def __init__(self):
        Node.__init__(self, 'switch_monitor')

        self.long_push_sec_threshold = self.declare_parameter(name='long_push_sec_threshold',
                                                              value=5).value
        self.rise_edge_pub = self.create_publisher(std_msgs.msg.Bool, 'switch_push_detected', 10)
        self.fall_edge_pub = self.create_publisher(std_msgs.msg.Bool, 'switch_release_detected', 10)

        # ui setup
        self.app = QApplication(sys.argv)

        self.top_widget = QWidget()
        layout = QGridLayout()

        self.software_switch = SoftwareSwitch(
            svg_path=str(Path(get_package_share_directory('simple_frontend'))/'resource'/'power_icon.svg')
        )
        layout.addWidget(self.software_switch, 0, 0)

        self.top_widget.setLayout(layout)
        layout.activate()
        self.top_widget.show()

        # Start monitoring operations to the button
        self.event_time = self.get_clock().now()
        self.software_switch.pressed.connect(self.detect_button_pressed)
        self.software_switch.released.connect(self.detect_button_released)

        # update GUI every 0.3 sec
        self.frontend_update_timer = self.create_timer(0.3, lambda: self.app.processEvents())


    def detect_button_pressed(self):
        self.event_time = self.get_clock().now()
        msg = std_msgs.msg.Bool()
        msg.data = True
        self.rise_edge_pub.publish(msg)

    def detect_button_released(self):
        now = self.get_clock().now()
        elapsed_in_sec = (now.nanoseconds - self.event_time.nanoseconds) / (10**9)
        if elapsed_in_sec < self.long_push_sec_threshold:
            # Short push: restart services
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


class SoftwareSwitch(QPushButton):
    def __init__ (self, text='', parent=None, svg_path=''):
        QPushButton.__init__(self, text, parent)

        self.svg_renderer = QSvgRenderer(svg_path)
        self.update_icon()

    def update_icon(self):
        size = QSize(128, 128)
        pixmap = QPixmap(size)
        pixmap.fill(0)
        painter = QPainter(pixmap)
        self.svg_renderer.render(painter)
        painter.end()
        self.setIcon(pixmap)
        self.setIconSize(size)


def main(args=None):
    rclpy.init(args=args)
    node = SoftwareSwitchMonitor()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
