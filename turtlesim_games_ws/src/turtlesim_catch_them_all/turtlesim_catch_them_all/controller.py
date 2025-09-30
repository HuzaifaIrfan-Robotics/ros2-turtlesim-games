

import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtCore import QTimer

import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

import random

from functools import partial

class TurtlesimCatchThemAllController(Node):
    def __init__(self):
        super().__init__("turtlesim_catch_them_all_controller")
        self.get_logger().info("Turtlesim Catch Them All Controller is active")

        # def log_pose_topics():
        #     topics = self.get_topic_names_and_types()
        #     pose_topics = [name for name, types in topics if name.endswith('/pose')]
        #     self.get_logger().info(f"Pose topics: {pose_topics}")

        # self.timer = self.create_timer(1.0, log_pose_topics)

        self.spawn_turtle_client_ = self.create_client(Spawn, "spawn")
        self.call_spawn_turtle()

        self.target_x = 2.0
        self.target_y = 8.0
        self.pose_: Pose = None
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/master/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/master/pose", self.callback_pose, 10)
        self.control_loop_timer_ = self.create_timer(
            0.1, self.control_loop)

    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def control_loop(self):
        if self.pose_ == None:
            return

        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        cmd = Twist()

        if distance > 1:
            # position
            cmd.linear.x = 2*distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            cmd.angular.z = 6*diff
        else:
            # target reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_publisher_.publish(cmd)



    def call_spawn_turtle(self):
        while not self.spawn_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn server...")

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.name = "master"

        future = self.spawn_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_turtle, request=request))

    def callback_call_spawn_turtle(self, future, request):
        response = future.result()
        self.get_logger().info("Spawned turtle at (" +
                               str(request.x) + ", " + str(request.y) + ") with name '" +
                               str(response.name) + "'")



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