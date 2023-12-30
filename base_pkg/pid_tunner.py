import sys
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton, QTextEdit, \
    QHBoxLayout, QScrollArea, QLineEdit, QFileDialog
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

KP_RANGE = 10
KI_RANGE = 100
KD_RANGE = 1
KP_PRECISION = 1000
KI_PRECISION = 1000
KD_PRECISION = 100000

class PidNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        self.pid_publisher = self.create_publisher(Float32MultiArray, '/pid_config', 10)
        self.get_logger().info('Pid node is running...')

class MotorSliderWidget(QWidget):
    def __init__(self, motor_index, kp_range, ki_range, kd_range, kp_precision, ki_precision, kd_precision, parent=None):
        super().__init__(parent)
        self.motor_index = motor_index
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.kp_range = kp_range
        self.ki_range = ki_range
        self.kd_range = kd_range
        self.kp_precision = kp_precision
        self.ki_precision = ki_precision
        self.kd_precision = kd_precision
        self.init_ui()
        self.pid_node = PidNode()

    def init_ui(self):
        layout = QHBoxLayout()

        motor_label = QLabel(f"Motor {self.motor_index}")
        layout.addWidget(motor_label)

        kp_layout = self.create_slider_layout('kp')
        layout.addLayout(kp_layout)

        ki_layout = self.create_slider_layout('ki')
        layout.addLayout(ki_layout)

        kd_layout = self.create_slider_layout('kd')
        layout.addLayout(kd_layout)

        self.setLayout(layout)

    def create_slider_layout(self, slider_type):
        slider_layout = QVBoxLayout()

        slider_label = QLabel(f"{slider_type.upper()}")
        slider_layout.addWidget(slider_label)

        slider = QSlider()
        slider.setObjectName(f"{slider_type}_slider")
        slider.setOrientation(1)
        slider.setMinimum(0)
        slider.setMaximum(self.kp_range * self.kp_precision if slider_type == 'kp' else
                           self.ki_range * self.ki_precision if slider_type == 'ki' else
                           self.kd_range * self.kd_precision)
        slider.setValue(0)
        slider.setTickInterval(1)
        slider.valueChanged.connect(lambda value: self.set_slider_value(slider_type, value))
        slider_layout.addWidget(slider)

        input_label = QLabel(f"{slider_type.upper()} Input:")
        slider_layout.addWidget(input_label)

        input_field = QLineEdit()
        input_field.setObjectName(f"{slider_type}_input")
        input_field.setReadOnly(True)
        self.enable_editing(input_field, slider_type)
        slider_layout.addWidget(input_field)

        return slider_layout

    def enable_editing(self, input_field, slider_type):
        input_field.setReadOnly(False)
        input_field.editingFinished.connect(lambda: self.update_slider_value(input_field, slider_type))

    def update_slider_value(self, input_field, slider_type):
        value = float(input_field.text())
        setattr(self, slider_type, value)
        for widget in self.children():
            if isinstance(widget, QSlider) and widget.objectName() == f"{slider_type}_slider":
                widget.setValue(int(value * self.kp_precision if slider_type == 'kp' else
                                    value * self.ki_precision if slider_type == 'ki' else
                                    value * self.kd_precision))

    def set_slider_value(self, slider_type, value):
        setattr(self, slider_type, value / self.kp_precision if slider_type == 'kp' else
                value / self.ki_precision if slider_type == 'ki' else
                value / self.kd_precision)
        for widget in self.children():
            if isinstance(widget, QLineEdit) and widget.objectName() == f"{slider_type}_input":
                widget.setText(str(getattr(self, slider_type)))


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.motor_sliders = []
        self.kp_range = KP_RANGE
        self.ki_range = KI_RANGE
        self.kd_range = KD_RANGE
        self.kp_precision = KP_PRECISION
        self.ki_precision = KI_PRECISION
        self.kd_precision = KD_PRECISION
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget(scroll_area)
        scroll_layout = QVBoxLayout(scroll_content)

        for motor_index in range(4):
            motor_widget = MotorSliderWidget(motor_index, self.kp_range, self.ki_range, self.kd_range,
                                             self.kp_precision, self.ki_precision, self.kd_precision)
            scroll_layout.addWidget(motor_widget)
            self.motor_sliders.append(motor_widget)

        scroll_content.setLayout(scroll_layout)
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)

        self.text_area = QTextEdit()
        self.text_area.setMaximumHeight(100)
        layout.addWidget(self.text_area)

        generate_button = QPushButton('Generate Array')
        generate_button.clicked.connect(self.generate_array)
        layout.addWidget(generate_button)

        save_button = QPushButton('Save to JSON')
        save_button.clicked.connect(self.save_to_json)
        layout.addWidget(save_button)

        load_button = QPushButton('Load from JSON')
        load_button.clicked.connect(self.load_from_json)
        layout.addWidget(load_button)

        send_button = QPushButton('Send to ROS')
        send_button.clicked.connect(self.send_to_ros)
        layout.addWidget(send_button)

        self.setLayout(layout)
        self.setWindowTitle('Motor Control GUI')

    def generate_array(self):
        kp_values = []
        ki_values = []
        kd_values = []
        for motor in self.motor_sliders:
            kp_values.append(motor.kp)
            ki_values.append(motor.ki)
            kd_values.append(motor.kd)

        np_array = np.array([kp_values, ki_values, kd_values])
        self.text_area.clear()
        self.text_area.insertPlainText(f"Numpy Array:\n{np_array}\n\n")

    def save_to_json(self):
        motor_values = []
        for motor in self.motor_sliders:
            motor_values.append({
                'kp': motor.kp,
                'ki': motor.ki,
                'kd': motor.kd
            })

        file_name, _ = QFileDialog.getSaveFileName(self, "Save Array", "", "JSON Files (*.json)")
        if file_name:
            with open(file_name, 'w') as file:
                json.dump(motor_values, file)
            self.text_area.insertPlainText(f"Array saved to {file_name}\n")

    def load_from_json(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Load Array", "", "JSON Files (*.json)")
        if file_name:
            with open(file_name, 'r') as file:
                loaded_data = json.load(file)

            for motor, values in zip(self.motor_sliders, loaded_data):
                motor.kp = values['kp']
                motor.ki = values['ki']
                motor.kd = values['kd']
                motor.set_slider_value('kp', values['kp'])
                motor.set_slider_value('ki', values['ki'])
                motor.set_slider_value('kd', values['kd'])
            self.text_area.insertPlainText(f"Array loaded from {file_name}\n")

    def send_to_ros(self):
        kp_values = []
        ki_values = []
        kd_values = []
        for motor in self.motor_sliders:
            kp_values.append(motor.kp)
            ki_values.append(motor.ki)
            kd_values.append(motor.kd)

        np_array = np.array([kp_values, ki_values, kd_values])
        msg = Float32MultiArray()
        msg.data = np_array
        self.pid_node.publish(msg)
        self.text_area.insertPlainText(f"Sent array to ROS:\n{np_array}\n")


def main():
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
