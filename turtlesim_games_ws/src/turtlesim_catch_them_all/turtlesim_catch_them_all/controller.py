

import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtCore import QTimer

class TurtlesimCatchThemAllController(Node):
    def __init__(self):
        super().__init__("turtlesim_catch_them_all_controller")
        self.get_logger().info("Turtlesim Catch Them All Controller is active")

        def log_pose_topics():
            topics = self.get_topic_names_and_types()
            pose_topics = [name for name, types in topics if name.endswith('/pose')]
            self.get_logger().info(f"Pose topics: {pose_topics}")

        self.timer = self.create_timer(1.0, log_pose_topics)

class MainWindow(QMainWindow):
    def __init__(self, node: TurtlesimCatchThemAllController):
        super().__init__()
        self.setWindowTitle("Turtlesim Catch Them All Controller")
        self.setCentralWidget(QLabel("Controller is running..."))
        self.node = node

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = TurtlesimCatchThemAllController()
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