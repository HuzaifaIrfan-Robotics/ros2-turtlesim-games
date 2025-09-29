import rclpy
from rclpy.node import Node

class TurtlesimCatchThemAllController(Node):
    def __init__(self):
        super().__init__("turtlesim_catch_them_all_controller")
        self.get_logger().info("Turtlesim Catch Them All Controller is active")

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimCatchThemAllController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()