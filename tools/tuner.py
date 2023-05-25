import sys
from PySide6 import QtCore, QtWidgets
from serial import Serial
from typing import Callable

class TuningKnob(QtWidgets.QWidget):
    def __init__(self, parameterLabel: str, valueChangedCb: Callable):
        super().__init__()

        self.text = QtWidgets.QLabel(parameterLabel, alignment=QtCore.Qt.AlignCenter)
        self.indicator = QtWidgets.QLabel("", alignment=QtCore.Qt.AlignCenter)
        
        self.knob = QtWidgets.QDial()
        self.knob.setMaximum(1000)
        self.knob.setMinimum(0)

        self.knob.valueChanged.connect(valueChangedCb)
        self.knob.valueChanged.connect(self.updateIndicator)

        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.indicator)
        self.layout.addWidget(self.knob)

    def getValue(self) -> float:
        return self.knob.value() / 1000
    
    @QtCore.Slot(int)
    def updateIndicator(self, val: int):
        self.indicator.setText(f"{val/1000}")


class MainWindow(QtWidgets.QWidget):
    def __init__(self, serial: Serial):
        super().__init__()

        self.kp_knob = TuningKnob("kp", self.paramChanged)
        self.ki_knob = TuningKnob("ki", self.paramChanged)
        self.kd_knob = TuningKnob("kd", self.paramChanged)

        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.addWidget(self.kp_knob)
        self.layout.addWidget(self.ki_knob)
        self.layout.addWidget(self.kd_knob)

        self.serial = serial

    @QtCore.Slot()
    def paramChanged(self):
        """
        A parameter changed. Send all.
        """
        kp = self.kp_knob.getValue()
        ki = self.ki_knob.getValue()
        kd = self.kd_knob.getValue()

        print(f"kp: {kp}, ki: {ki}, kd: {kd}")
        self.serial.write(f"{kp} {ki} {kd}".encode())


if __name__ == "__main__":
    app = QtWidgets.QApplication([])

    with Serial("/dev/cu.usbserial-0001", 115200) as s:
        window = MainWindow(s)

        window.resize(800, 600)
        window.show()

        sys.exit(app.exec())