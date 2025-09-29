import rclpy
from rclpy.node import Node

class TurtlesimCatchThemAllSpawner(Node):
    def __init__(self):
        super().__init__("turtlesim_catch_them_all_spawner")
        self.get_logger().info("Turtlesim Catch Them All Spawner is active")

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimCatchThemAllSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()