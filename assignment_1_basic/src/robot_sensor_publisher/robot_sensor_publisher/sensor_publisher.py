import rclpy
from rclpy.node import Node
import random
from robot_interfaces.msg import SensorData 

class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_publisher')
        self.publisher_ = self.create_publisher(SensorData, 'sensor_data', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = SensorData()
        msg.temperature = random.uniform(20.0, 30.0)
        msg.humidity = random.uniform(40.0, 80.0)
        msg.timestamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: Temperature={msg.temperature:.2f}, Humidity={msg.humidity:.2f}, Timestamp={msg.timestamp}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    robot_publisher = RobotPublisher()
    rclpy.spin(robot_publisher)
    robot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
