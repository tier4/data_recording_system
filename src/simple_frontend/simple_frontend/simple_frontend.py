"""
Entry point of the ROS node (simple_frontend)
"""
import sys
import threading
import shutil
from pathlib import Path
from time import sleep

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
import std_msgs.msg
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from PySide2.QtWidgets import (QApplication, QWidget, QLabel, QLayout, QGridLayout,  QSizePolicy,
                               QProgressBar, QStyleOptionProgressBar)
from PySide2 import QtCore
from PySide2.QtGui import QPalette, QColor


from .led_indicator import QLed, LedState

class UiEventFilter(QtCore.QObject):
    def __init__(self, close_fn):
        super().__init__()
        self.close_fn = close_fn

    def eventFilter(self, widget, event):
        if event.type() == QtCore.QEvent.Close:
            # Catch close event
            # https://stackoverflow.com/questions/72133476/pyqt5-how-to-reimplement-close-event-in-event-filter
            self.close_fn()
            return True
        else:
            return False        # make it unhandled (= pass through)

class LabelAndLed:
    def __init__(self, parent=None, title=None):
        self.layout = QGridLayout()

        self.title = QLabel(parent=parent)
        self.title.setText(title)
        self.layout.addWidget(self.title, 0, 0)

        self.state_label = QLabel(parent=parent)
        self.state_label.setSizePolicy(QSizePolicy.Maximum, QSizePolicy.Maximum)
        self.layout.addWidget(self.state_label, 1, 0)

        self.led = QLed(parent=parent)
        self.led.setState(LedState.RED)
        self.layout.addWidget(self.led, 2, 0)

    def set_state_text(self, text):
        self.state_label.setText(f'status: {text}')
        self.state_label.adjustSize()

    def set_status(self, is_ok: bool):
        if is_ok:
            self.set_state_text('OK')
            self.led.setState(LedState.GREEN)
        else:
            self.set_state_text('Failed')
            self.led.setState(LedState.RED)

class LabelAndBar:
    def __init__(self, parent=None, title=None, orientation=QtCore.Qt.Vertical):
        self.layout = QGridLayout()

        self.title = QLabel(parent=parent)
        self.title.setText(title)
        self.layout.addWidget(self.title, 0, 0)

        self.bar = QProgressBar(parent=parent)
        self.bar.setOrientation(orientation)
        self.bar.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)
        self.bar.setTextVisible(True)
        palette = self.bar.palette()
        palette.setColor(QPalette.Text, QtCore.Qt.black)
        palette.setColor(QPalette.HighlightedText, QtCore.Qt.black)
        self.bar.setPalette(palette)
        self.layout.addWidget(self.bar, 1, 0)


class SimpleFrontend(Node):
    def __init__(self):
        super().__init__('simple_frontend')

        # Get switch_monitor information
        parameter_client = self.create_client(GetParameters, '/switch_monitor/get_parameters')
        request = GetParameters.Request()
        request.names = ['long_push_sec_threshold']
        future = parameter_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.long_push_millisec_threshold = future.result().values[0].integer_value * 10**3

        # ui setup
        self.app = QApplication(sys.argv)

        self.top_widget = QWidget()
        self.close_filter = UiEventFilter(close_fn=self.terminate)
        # When windows is closed, ROS process will also be shutdown
        self.top_widget.installEventFilter(self.close_filter)

        layout = QGridLayout()
        layout.setSizeConstraint(QLayout.SetMinimumSize)

        self.ecu1_indicator = LabelAndLed(parent=self.top_widget, title='ECU#1')
        layout.addLayout(self.ecu1_indicator.layout, 0, 0)

        self.ecu2_indicator = LabelAndLed(parent=self.top_widget, title='ECU#2')
        layout.addLayout(self.ecu2_indicator.layout, 0, 1)

        self.__set_init_state()

        self.disk_usage_indicator = LabelAndBar(parent=self.top_widget, title='Disk Usage:')
        layout.addLayout(self.disk_usage_indicator.layout, 0, 2)
        self.disk_usage_monitor_path_str = self.declare_parameter('usage_monitored_disk',
                                                                  rclpy.Parameter.Type.STRING).value
        self.disk_usage_check_interval_sec = self.declare_parameter(name='disk_check_inteval_sec',
                                                                    value=60).value
        # Because usage check query may take a while, which affects to the UI response,
        # execute usage monitoring in another thread
        self.disk_usage_check_thread = threading.Thread(target=self.check_disk_usage, daemon=True)
        self.disk_usage_check_thread.start()

        self.button_hold_indicator = QProgressBar(parent=self.top_widget)
        self.button_hold_indicator.setMaximum(self.long_push_millisec_threshold)
        self.button_hold_indicator.setTextVisible(False)
        self.__set_ui_color(self.button_hold_indicator, QColor(QtCore.Qt.red))
        layout.addWidget(self.button_hold_indicator, 1, 0, 1, 3)  # row 1, colums 0 to 3

        self.top_widget.setLayout(layout)
        layout.activate()
        self.top_widget.show()


        # Create subscribers to monitor each ECU states
        self.create_subscription(std_msgs.msg.Bool, '/ecu1/statistics',
                                 lambda msg: self.state_callback(msg, self.ecu1_indicator), 10)

        self.create_subscription(std_msgs.msg.Bool, '/ecu2/statistics',
                                 lambda msg: self.state_callback(msg, self.ecu2_indicator), 10)

        # Subscribe switch information from monitor
        self.push_detect_time = None
        self.create_subscription(std_msgs.msg.Bool, 'switch_push_detected',
                                 self.switch_push_callback, 10)
        self.create_subscription(std_msgs.msg.Bool, 'switch_release_detected',
                                 self.switch_release_callback, 10)

        # update GUI every 0.1 sec
        self.frontend_update_timer = self.create_timer(0.1, self.process_event_callback)

    def __set_init_state(self):
        self.ecu1_indicator.set_state_text("Initializing...")
        self.ecu1_indicator.led.setState(LedState.OFF)

        self.ecu2_indicator.set_state_text("Initializing...")
        self.ecu2_indicator.led.setState(LedState.OFF)

    def __set_ui_color(self, ui: QWidget, color: QColor):
        palette = QPalette(ui.palette())
        palette.setColor(QPalette.Highlight, color)
        ui.setPalette(palette)

    def process_event_callback(self):
        if self.push_detect_time:
            # If switch push is detected, update the indicator
            press_length_millisec =(self.get_clock().now().nanoseconds
                                    - self.push_detect_time.nanoseconds) / 10**6
            self.button_hold_indicator.setValue(press_length_millisec)
            if press_length_millisec >= self.long_push_millisec_threshold:
                # If switch is hold over the threshold, change indicator color
                self.button_hold_indicator.setValue(self.long_push_millisec_threshold)
                self.__set_ui_color(self.button_hold_indicator, QColor(QtCore.Qt.green))
        else:
            # If switch release is detected, reset the indicator
            self.button_hold_indicator.reset()
            self.__set_ui_color(self.button_hold_indicator, QColor(QtCore.Qt.red))

        # re-draw UI
        self.app.processEvents()

    def terminate(self):
        # Terminate ROS process from inside node
        # https://answers.ros.org/question/406469/ros-2-how-to-quit-a-node-from-within-a-callback/
        raise SystemExit

    def state_callback(self, msg: std_msgs.msg.Bool, indicator: LabelAndLed):
        print(f'{indicator.title.text()}: {msg.data}')
        indicator.set_status(msg.data)

    def switch_push_callback(self, _: std_msgs.msg.Bool):
        self.push_detect_time = self.get_clock().now()

    def switch_release_callback(self, _: std_msgs.msg.Bool):
        self.push_detect_time = None

    def check_disk_usage(self):
        while True:
            total, used, free = shutil.disk_usage(Path(self.disk_usage_monitor_path_str)
                                                  .resolve().absolute())
            total_mb = total / 1024**2
            used_mb = used / 1024**2
            self.disk_usage_indicator.bar.setMaximum(total_mb)
            self.disk_usage_indicator.bar.setValue(used_mb)
            sleep(self.disk_usage_check_interval_sec)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFrontend()

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
