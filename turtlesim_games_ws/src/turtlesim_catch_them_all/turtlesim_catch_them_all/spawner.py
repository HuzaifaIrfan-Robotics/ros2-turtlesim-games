import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtCore import QTimer
from turtlesim.srv import Spawn

import random

from functools import partial
from PyQt6.QtWidgets import QSlider, QVBoxLayout
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QSpinBox

class TurtlesimCatchThemAllSpawner(Node):
    def __init__(self):
        super().__init__("spawner")

        self.spawn_turtle_client_ = self.create_client(Spawn, "spawn")
        self.spawn_turtle_timer_ = self.create_timer(5.0, self.call_spawn_turtle)
        self.spawn_turtle_timer_.timer_period_ns = int(5.0 * 1e9)  # Default 5 seconds

        self.get_logger().info("Turtlesim Catch Them All Spawner is active")

    def call_spawn_turtle(self):
        while not self.spawn_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn server...")

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)

        future = self.spawn_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_turtle, request=request))

    def callback_call_spawn_turtle(self, future, request):
        response = future.result()
        self.get_logger().info("Spawned turtle at (" +
                               str(request.x) + ", " + str(request.y) + ") with name '" +
                               str(response.name) + "'")


class MainWindow(QMainWindow):
    def __init__(self, node: TurtlesimCatchThemAllSpawner):
        super().__init__()
        self.setWindowTitle("Spawner Turtlesim")
        
        self.node = node

        layout = QVBoxLayout()
        # Integer box for direct input
        self.spin_box = QSpinBox()
        self.spin_box.setMinimum(1)
        self.spin_box.setMaximum(10)
        self.spin_box.setValue(5)
        self.spin_box.valueChanged.connect(self.update_timer_interval)
        layout.addWidget(QLabel("Set Interval (seconds):"))
        layout.addWidget(self.spin_box)
        layout.addWidget(QLabel("Spawner is running..."))

        widget = QLabel()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def update_timer_interval(self, value):
        self.node.spawn_turtle_timer_.timer_period_ns = int(value * 1e9)




def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = TurtlesimCatchThemAllSpawner()
    window = MainWindow(node)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(50)  # Spin ROS every 50 ms

    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()