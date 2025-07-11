import rclpy
from rclpy.node import Node

from robot_interfaces.msg import SensorData 


class RobotSubscriber(Node):

    def __init__(self):

        super().__init__('robot_subscriber')

        self.subscription = self.create_subscription(
            SensorData,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  

        self.publisher_ = self.create_publisher(SensorData, 'processed_data', 10)

        self.total_temperature = 0.0
        self.total_humidity = 0.0
        self.total_count = 0

    def listener_callback(self, msg):

        self.total_temperature += msg.temperature
        self.total_humidity += msg.humidity
        self.total_count += 1

        avg_temperature = self.total_temperature / self.total_count
        avg_humidity = self.total_humidity / self.total_count

        self.get_logger().info(f"Received data: Temperature: {msg.temperature} °C, "
                               f"Humidity: {msg.humidity} %, "
                               f"Timestamp: {msg.timestamp.sec} sec, {msg.timestamp.nanosec} ns")
        self.get_logger().info(f"Average Temperature: {avg_temperature} °C, "
                               f"Average Humidity: {avg_humidity} %")

        processed_data = SensorData()
        processed_data.temperature = avg_temperature
        processed_data.humidity = avg_humidity
        processed_data.timestamp = self.get_clock().now().to_msg()

        self.publisher_.publish(processed_data)

def main(args=None):
    rclpy.init(args=args)
    robot_subscriber = RobotSubscriber()
    rclpy.spin(robot_subscriber)
    robot_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()