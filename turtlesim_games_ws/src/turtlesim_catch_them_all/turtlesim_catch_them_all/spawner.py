import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtCore import QTimer

class TurtlesimCatchThemAllSpawner(Node):
    def __init__(self):
        super().__init__("turtlesim_catch_them_all_spawner")
        self.get_logger().info("Turtlesim Catch Them All Spawner is active")

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Turtlesim Catch Them All Spawner")
        self.setCentralWidget(QLabel("Spawner is running..."))

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = TurtlesimCatchThemAllSpawner()
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