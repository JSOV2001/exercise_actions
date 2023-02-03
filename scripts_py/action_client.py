#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from exercise_navigation.action import Navigation2D

class NavigationActionClient(Node):
    def __init__(self):
        super().__init__("navigation_action_client")
        self.action_client = ActionClient(self, Navigation2D, "/navigation")
    
    def send_goal(self, x, y):
        goal_msg = Navigation2D.Goal()
        goal_msg.goal.x = float(x)
        goal_msg.goal.y = float(y)
        goal_msg.goal.z = float(0)

        self.action_client.wait_for_server()
        self.goal_future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)
        self.goal_future.add_done_callback(self.respond_goal)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_to_point = feedback.distance_to_point
        print(f"Feedback: {distance_to_point} meters")

    def respond_goal(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            print("Goal Rejected!")
            rclpy.spin_once(self, timeout_sec= 5)
        print("Goal Accepted!")

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        elapsed_time = result.elapsed_time
        print(f"Result: {elapsed_time} seconds")

def main():
    rclpy.init()
    my_node = NavigationActionClient()
    try:
        x = input("Enter x coordinate: ")
        y = input("Enter y coordinate: ")
        my_node.send_goal(x, y)
        rclpy.spin(my_node)
    except KeyboardInterrupt:
        my_node.action_client.destroy()
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
