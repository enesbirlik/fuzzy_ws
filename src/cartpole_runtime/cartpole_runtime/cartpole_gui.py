import sys
import time
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

class CartpoleGUI(Node):
    def __init__(self, controller_type='position'):
        super().__init__('cartpole_gui')
        self.controller_type = controller_type

        # Controller tipine göre uygun konu ve ölçek faktörünü belirleyelim
        if self.controller_type == 'position':
            self.cmd_topic = '/cart_position_controller/commands'
            self.scale = 0.235
        elif self.controller_type == 'velocity':
            self.cmd_topic = '/cart_velocity_controller/commands'
            self.scale = 1.0
        elif self.controller_type == 'effort':
            self.cmd_topic = '/cart_effort_controller/commands'
            self.scale = 1.0   # Önceki 0.1 yerine 0.2 yapıldı; slider -100..100 => -20..20
        else:
            self.cmd_topic = '/cart_position_controller/commands'
            self.scale = 0.235

        self.cmd_pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Joint state değerleri
        self.cart_pos = 0.0
        self.cart_vel = 0.0
        self.cart_effort = 0.0

        self.pole_pos = 0.0
        self.pole_vel = 0.0
        self.pole_effort = 0.0

        # Grafik verileri
        self.time_data = []
        self.cart_pos_data = []
        self.cart_vel_data = []
        self.cart_effort_data = []

        self.pole_pos_data = []
        self.pole_vel_data = []
        self.pole_effort_data = []

        self.start_time = time.time()

    def joint_state_callback(self, msg: JointState):
        # "slider_to_cart" joint bilgileri
        if "slider_to_cart" in msg.name:
            idx_cart = msg.name.index("slider_to_cart")
            self.cart_pos = msg.position[idx_cart]
            self.cart_vel = msg.velocity[idx_cart]
            if len(msg.effort) > idx_cart:
                self.cart_effort = msg.effort[idx_cart]
        # "cart_to_pole" joint bilgileri
        if "cart_to_pole" in msg.name:
            idx_pole = msg.name.index("cart_to_pole")
            self.pole_pos = msg.position[idx_pole]
            self.pole_vel = msg.velocity[idx_pole]
            if len(msg.effort) > idx_pole:
                self.pole_effort = msg.effort[idx_pole]

        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.cart_pos_data.append(self.cart_pos)
        self.cart_vel_data.append(self.cart_vel)
        self.cart_effort_data.append(self.cart_effort)
        self.pole_pos_data.append(self.pole_pos)
        self.pole_vel_data.append(self.pole_vel)
        self.pole_effort_data.append(self.pole_effort)

    def send_command(self, value: float):
        msg = Float64MultiArray()
        msg.data = [value]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"[{self.controller_type.upper()} CMD] -> {self.cmd_topic}: {value:.3f}")

class MainWindow(QtWidgets.QWidget):
    def __init__(self, ros_node: CartpoleGUI):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_displays)
        self.timer.start(100)

    def init_ui(self):
        if self.ros_node.controller_type == 'position':
            self.setWindowTitle('Cartpole Position Controller Interface')
        elif self.ros_node.controller_type == 'velocity':
            self.setWindowTitle('Cartpole Velocity Controller Interface')
        elif self.ros_node.controller_type == 'effort':
            self.setWindowTitle('Cartpole Effort Controller Interface')

        main_layout = QtWidgets.QVBoxLayout()

        # Komut gönderme alanı
        cmd_layout = QtWidgets.QHBoxLayout()
        if self.ros_node.controller_type == 'position':
            self.command_label = QtWidgets.QLabel('Position Command:')
        elif self.ros_node.controller_type == 'velocity':
            self.command_label = QtWidgets.QLabel('Velocity Command:')
        elif self.ros_node.controller_type == 'effort':
            self.command_label = QtWidgets.QLabel('Effort Command:')

        self.command_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.command_slider.setMinimum(-100)
        self.command_slider.setMaximum(100)
        self.command_slider.setValue(0)
        self.command_value_label = QtWidgets.QLabel('0.00')
        self.command_slider.valueChanged.connect(self.slider_changed)

        send_button = QtWidgets.QPushButton('Send Command')
        send_button.clicked.connect(self.send_command)

        cmd_layout.addWidget(self.command_label)
        cmd_layout.addWidget(self.command_slider)
        cmd_layout.addWidget(self.command_value_label)
        cmd_layout.addWidget(send_button)
        main_layout.addLayout(cmd_layout)

        # Joint state metin kutusu (üst kısım)
        main_layout.addWidget(QtWidgets.QLabel("Joint States (position, velocity, effort):"))
        self.joint_state_display = QtWidgets.QTextEdit()
        self.joint_state_display.setReadOnly(True)
        self.joint_state_display.setMaximumHeight(60)
        main_layout.addWidget(self.joint_state_display)

        # Grafik alanı: 6 grafik
        self.graph_widget = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self.graph_widget, 1)

        self.plot_cart_pos = self.graph_widget.addPlot(row=0, col=0, title="Cart Position")
        self.plot_cart_pos_data = self.plot_cart_pos.plot(pen='b')

        self.plot_cart_vel = self.graph_widget.addPlot(row=0, col=1, title="Cart Velocity")
        self.plot_cart_vel_data = self.plot_cart_vel.plot(pen='r')

        self.plot_pole_pos = self.graph_widget.addPlot(row=1, col=0, title="Pole Position")
        self.plot_pole_pos_data = self.plot_pole_pos.plot(pen='g')

        self.plot_pole_vel = self.graph_widget.addPlot(row=1, col=1, title="Pole Velocity")
        self.plot_pole_vel_data = self.plot_pole_vel.plot(pen='m')

        self.plot_cart_effort = self.graph_widget.addPlot(row=2, col=0, title="Cart Effort")
        self.plot_cart_effort_data = self.plot_cart_effort.plot(pen='c')

        self.plot_pole_effort = self.graph_widget.addPlot(row=2, col=1, title="Pole Effort")
        self.plot_pole_effort_data = self.plot_pole_effort.plot(pen='y')

        self.setLayout(main_layout)

    def slider_changed(self, value):
        command_val = (value / 100.0) * self.ros_node.scale
        self.command_value_label.setText(f"{command_val:.3f}")

    def send_command(self):
        value = self.command_slider.value()
        command_val = (value / 100.0) * self.ros_node.scale
        self.ros_node.send_command(command_val)

    def update_displays(self):
        text = (
            f"Cart Position: {self.ros_node.cart_pos:.3f}, "
            f"Cart Velocity: {self.ros_node.cart_vel:.3f}, "
            f"Cart Effort: {self.ros_node.cart_effort:.3f}\n"
            f"Pole Position: {self.ros_node.pole_pos:.3f}, "
            f"Pole Velocity: {self.ros_node.pole_vel:.3f}, "
            f"Pole Effort: {self.ros_node.pole_effort:.3f}\n"
        )
        self.joint_state_display.setPlainText(text)

        t = self.ros_node.time_data
        self.plot_cart_pos_data.setData(t, self.ros_node.cart_pos_data)
        self.plot_cart_vel_data.setData(t, self.ros_node.cart_vel_data)
        self.plot_pole_pos_data.setData(t, self.ros_node.pole_pos_data)
        self.plot_pole_vel_data.setData(t, self.ros_node.pole_vel_data)
        self.plot_cart_effort_data.setData(t, self.ros_node.cart_effort_data)
        self.plot_pole_effort_data.setData(t, self.ros_node.pole_effort_data)

def main(args=None):
    parser = argparse.ArgumentParser(description='Cartpole GUI with position, velocity or effort mode')
    parser.add_argument(
        'controller_type',
        nargs='?',
        default='position',
        choices=['position', 'velocity', 'effort'],
        help='Which controller type to use (position, velocity, or effort)'
    )
    parsed_args, unknown = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=args)
    node = CartpoleGUI(controller_type=parsed_args.controller_type)

    app = QtWidgets.QApplication(sys.argv)
    main_window = MainWindow(node)
    main_window.show()

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
