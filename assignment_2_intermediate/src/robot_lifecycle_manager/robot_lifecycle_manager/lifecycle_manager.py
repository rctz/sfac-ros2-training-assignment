import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn

class SystemManager(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_manager')
        self.declare_parameter('processing_rate', 5.0)
        self.get_logger().info("Lifecycle manager initialized")

    def on_configure(self, state):
        rate = self.get_parameter('processing_rate').get_parameter_value().double_value
        self.get_logger().info(f"Configured with rate={rate}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("Deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("Cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info("Shutdown")
        return TransitionCallbackReturn.SUCCESS

def main():
    rclpy.init()
    node = SystemManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
