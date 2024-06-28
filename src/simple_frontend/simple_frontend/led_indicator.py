"""
virtual LED indicator
https://forum.qt.io/topic/101648/how-to-create-simply-virtual-led-indicator
"""
from PySide2.QtWidgets import QWidget, QLabel, QSizePolicy
from enum import IntEnum, auto

class LedState(IntEnum):
    GREEN = auto()
    RED = auto()
    OFF = auto()


class QLed(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setState(LedState.GREEN)

        # Make this widget always square
        policy = QSizePolicy()
        policy.setHeightForWidth(True)
        policy.setWidthForHeight(True)
        policy.setHorizontalPolicy(QSizePolicy.MinimumExpanding)
        policy.setVerticalPolicy(QSizePolicy.MinimumExpanding)
        self.setSizePolicy(policy)

    def setState(self, state: LedState):
        if state == LedState.GREEN:
            self.setStyleSheet(self.__generate_green_style())
        elif state == LedState.RED:
            self.setStyleSheet(self.__generate_red_style())
        elif state == LedState.OFF:
            self.setStyleSheet(self.__generate_off_style())

    def __generate_green_style(self):
        return self.__generate_style_str((20, 252, 7, 255), (25, 134, 5, 255))

    def __generate_red_style(self):
        return self.__generate_style_str((255, 12, 12, 255), (103, 0, 0, 255))

    def __generate_off_style(self):
        return self.__generate_style_str((0, 0, 0, 255), (0, 0, 0, 255))

    def __generate_style_str(self, rgba_1, rgba_2):
        return (f'color: white;border-radius: {int(self.size().width()/2)}px;'
                f'background-color: qlineargradient(spread:pad, x1:0.145, y1:0.16, x2:1, y2:1, '
                f'stop:0 rgba({rgba_1[0]}, {rgba_1[1]}, {rgba_1[2]}, {rgba_1[3]}), '
                f'stop:1 rgba({rgba_2[0]}, {rgba_2[1]}, {rgba_2[2]}, {rgba_2[3]}));')
        # return (f'border: 0.5px solid red; border-radius: 50%; background-clip: padding;')

    def resizeEvent(self, evt):
        width = self.size().width()
        height = self.size().height()
        if width < height:
            width = height
        else:
            height = width
        self.setFixedSize(width, height)


def debug():
    import sys
    from PySide2.QtWidgets import QApplication
    app = QApplication(sys.argv)
    led = QLed()
    led.show()

    app.exec_()


if __name__ == '__main__':
    debug()
