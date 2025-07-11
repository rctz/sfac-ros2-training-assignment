import rclpy
from rclpy.node import Node
from robot_interfaces.srv import RobotControl

class RobotControlService(Node):
    def __init__(self):
        super().__init__('robot_control_service')
        self.srv = self.create_service(RobotControl, 'robot_control', self.handle_move)

    def handle_move(self, req, res):
        distance = req.linear_velocity * req.duration
        res.success = True
        res.distance_traveled = distance
        res.message = f"Moved {distance:.2f} m with angular velocity {req.angular_velocity}"
        self.get_logger().info(res.message)
        return res

def main():
    rclpy.init()
    node = RobotControlService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
