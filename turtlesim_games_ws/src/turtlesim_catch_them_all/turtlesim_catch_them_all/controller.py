

import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtCore import QTimer

class TurtlesimCatchThemAllController(Node):
    def __init__(self):
        super().__init__("turtlesim_catch_them_all_controller")
        self.get_logger().info("Turtlesim Catch Them All Controller is active")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Turtlesim Catch Them All Controller")
        self.setCentralWidget(QLabel("Controller is running..."))

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = TurtlesimCatchThemAllController()
    window = MainWindow()
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(50)  # Spin ROS every 50 ms

    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()