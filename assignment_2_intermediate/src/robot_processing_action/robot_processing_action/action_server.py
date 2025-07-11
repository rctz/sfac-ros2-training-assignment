import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from action_msgs.msg import GoalStatus
from robot_interfaces.action import ProcessData

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        self._current_goal_handle = None
        self._should_stop = False
        grp = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            ProcessData,
            'process_data',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=grp
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received new goal: data_size = {goal_request.data_size}")

        if self._current_goal_handle and self._current_goal_handle.is_active:
            self.get_logger().warn("Canceling current goal for new one")
            self._should_stop = True
            self._current_goal_handle.abort()  

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested by client")
        if self._current_goal_handle == goal_handle:
            self._should_stop = True
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._current_goal_handle = goal_handle
        self._should_stop = False
        count = 0
        start = time.time()
        data_size = goal_handle.request.data_size
        feedback = ProcessData.Feedback()

        for _ in range(data_size):
            if self._should_stop:
                self.get_logger().warn("Canceling goal")
                if goal_handle.status == GoalStatus.STATUS_EXECUTING:
                    goal_handle.abort()
                return ProcessData.Result(
                    success=False,
                    processed_count=count,
                    processing_time=time.time() - start,
                    result_message="Preempted or canceled"
                )

            time.sleep(0.01)
            count += 1

            if count % (data_size // 10 + 1) == 0:
                feedback.progress_percent = int(count / data_size * 100)
                feedback.current_status = f"{count}/{data_size}"
                goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        self._current_goal_handle = None
        self.get_logger().info("Goal completed successfully")
        return ProcessData.Result(
            success=True,
            processed_count=count,
            processing_time=time.time() - start,
            result_message="Done"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
