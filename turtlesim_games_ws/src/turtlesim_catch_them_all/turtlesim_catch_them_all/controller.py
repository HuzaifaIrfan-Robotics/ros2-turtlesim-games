

import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtCore import QTimer

import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn , Kill

import random

from functools import partial

class TurtlesimCatchThemAllController(Node):
    def __init__(self):
        super().__init__("controller")
        self.namespace_ = self.get_namespace()

        if self.namespace_ == "/":
            self.master_turtle_name = "player"
        else:
            self.master_turtle_name = f"{self.namespace_}".replace("/", "_").strip("_")

        self.spawn_turtle_client_ = self.create_client(Spawn, "/spawn")
        self.call_spawn_master_turtle()


        self.pose_: Pose = None
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, f"/{self.master_turtle_name}/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, f"/{self.master_turtle_name}/pose", self.callback_pose, 10)
        self.control_loop_timer_ = self.create_timer(
            0.1, self.control_loop)
        
        self.kill_turtle_client_ = self.create_client(Kill, "/kill")
        self.target_turtle_name = ""
        self.target_x = 2.0
        self.target_y = 8.0

        self.get_logger().info(f"{self.master_turtle_name} Turtlesim Catch Them All Controller is active")



    def callback_pose(self, pose: Pose):
        self.pose_ = pose

    def control_loop(self):
        if self.pose_ == None:
            return

        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x * dist_x + dist_y * dist_y)

        cmd = Twist()

        if distance > 0.5:
            # position
            cmd.linear.x = 1.5*distance

            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            cmd.angular.z = 2*diff
        else:
            # target reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.kill_target_turtle()



        self.cmd_vel_publisher_.publish(cmd)


    def kill_target_turtle(self):

        if self.target_turtle_name:
            self.get_logger().info(f"Killing turtle: {self.target_turtle_name}")
            self.call_kill_target_turtle()

        self.get_next_target_turtle()





    def get_next_target_turtle(self):

        topics = self.get_topic_names_and_types()
        pose_topics = [name for name, types in topics if name.endswith('/pose') and name.startswith('/turtle')]
        self.get_logger().info(f"Pose topics: {pose_topics}")

        if len(pose_topics) > 0:
            target_turtle_pose_topic = random.choice(pose_topics)
            target_turtle_name = target_turtle_pose_topic.split('/')[1]


            self.target_pose_subscription_ = self.create_subscription(
                Pose,
                target_turtle_pose_topic,
                partial(self.callback_target_pose, target_turtle_name=target_turtle_name),
                10
            )




    def callback_target_pose(self, pose: Pose, target_turtle_name=None):
        self.target_x = pose.x
        self.target_y = pose.y
        if target_turtle_name:
            self.target_turtle_name = target_turtle_name
        self.get_logger().info(f"New target turtle: {self.target_turtle_name} {self.target_x} {self.target_y}")
        if hasattr(self, 'target_pose_subscription_'):
            self.destroy_subscription(self.target_pose_subscription_)



    def call_kill_target_turtle(self):
        while not self.kill_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Kill server...")

        request = Kill.Request()
        request.name = self.target_turtle_name
        self.target_turtle_name = ""


        future = self.kill_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_target_turtle, request=request))

    def callback_call_kill_target_turtle(self, future, request):
        response = future.result()
        self.get_logger().info("Killed turtle at (" +
                               str(self.target_x) + ", " + str(self.target_y) + ") with name '" +
                               str(request.name) + "'")



    def call_spawn_master_turtle(self):
        while not self.spawn_turtle_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Spawn server...")

        request = Spawn.Request()
        request.x = random.uniform(1.0, 10.0)
        request.y = random.uniform(1.0, 10.0)
        request.name = self.master_turtle_name

        future = self.spawn_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn_master_turtle, request=request))

    def callback_call_spawn_master_turtle(self, future, request):
        response = future.result()
        self.get_logger().info("Spawned turtle at (" +
                               str(request.x) + ", " + str(request.y) + ") with name '" +
                               str(response.name) + "'")




def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimCatchThemAllController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()