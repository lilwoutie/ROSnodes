import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
import threading

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._current_goal_handle = None
        self._odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_position = None

    def send_goal(self, x, y, theta):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        goal_msg.pose.pose.orientation = q

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def send_goal(self, x_increment, y_increment, theta_increment):
        if self.current_position is None:
            self.get_logger().info('Current position not available yet.')
            return

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.current_position.position.x + x_increment
        goal_msg.pose.pose.position.y = self.current_position.position.y + y_increment
        goal_msg.pose.pose.position.z = 0.0

        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.z = math.sin((self.get_yaw_from_quaternion(self.current_position.orientation) + theta_increment) / 2.0)
        q.w = math.cos((self.get_yaw_from_quaternion(self.current_position.orientation) + theta_increment) / 2.0)
        goal_msg.pose.pose.orientation = q

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_yaw_from_quaternion(self, quaternion):
        import tf_transformations
        euler = tf_transformations.euler_from_quaternion([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w
        ])
        return euler[2]

    def goal_response_callback(self, future):
        self._current_goal_handle = future.result()
        if not self._current_goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = self._current_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def cancel_goal(self):
        if self._current_goal_handle:
            self.get_logger().info('Canceling current goal...')
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
    
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose

    def get_current_location(self):
        if self.current_position:
            self.get_logger().info(f'Current location: x={self.current_position.position.x}, y={self.current_position.position.y}')
        else:
            self.get_logger().info('Current location not available yet.')

def input_goal(action_client):
    while True:
        key = input("Press 'g' to set a new goal, 'l' to get location, or 'q' to quit: ")
        if key == 'g':
            try:
                x = float(input("Enter the x-coordinate of the goal: "))
                y = float(input("Enter the y-coordinate of the goal: "))
                theta = float(input("Enter the orientation (theta in radians) of the goal: "))
                action_client.send_goal(x, y, theta)
            except ValueError:
                print("Invalid input! Please enter numeric values.")
        elif key == 'l':
            action_client.get_current_location()
        elif key == 'q':
            action_client.cancel_goal()
            rclpy.shutdown()
            break

def main(args=None):
    rclpy.init(args=args)
    action_client = Nav2ActionClient()
    input_thread = threading.Thread(target=input_goal, args=(action_client,))
    input_thread.start()
    rclpy.spin(action_client)
    input_thread.join()

if __name__ == '__main__':
    main()
