#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from exercise_navigation.action import Navigation2D
from geometry_msgs.msg import Point
import math

distance_threshold = 0.125

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__("navigation_action_server")
        self.action_server = ActionServer(self, Navigation2D, "/navigation", self.navigation_callback)
        self.current_position = None
        self.position_subscriber = self.create_subscription(Point, "/position", self.position_callback, 10)
    
    def position_callback(self, point_msg):
        self.current_position = [point_msg.x, 
                                point_msg.y,
                                point_msg.z]
    
    def navigation_callback(self, goal_handle):
        start_time = self.get_clock().now().to_msg().sec
        
        #Recibir Goal
        robot_goal_point = [goal_handle.request.goal.x, 
                            goal_handle.request.goal.y,
                            goal_handle.request.goal.z]
        print("Goal Received")

        while self.current_position == None:
            print("Robot Point Not Detected")
            rclpy.spin_once(self, timeout_sec= 5)
        
        #Enviar Feedback
        initial_position = self.current_position
        distance_to_point = math.dist(initial_position, robot_goal_point)
        print("Robot Point Detected")
        feedback_msg = Navigation2D.Feedback()
        while distance_to_point > distance_threshold:
            current_distance = math.dist(self.current_position, robot_goal_point)
            feedback_msg.distance_to_point = current_distance
            goal_handle.publish_feedback(feedback_msg)
            rclpy.spin_once(self, timeout_sec=5)

        #Enviar Result
        final_time = self.get_clock().now().to_msg().sec
        total_time = float(final_time - start_time)
        goal_handle.succeed()
        print("Goal Succeeded!")
        result_msg = Navigation2D.Result()
        result_msg.elapsed_time = total_time
        print(f"Result: {round(result_msg.elapsed_time, 3)} seconds")
        return result_msg
    
def main():
    rclpy.init() #Iniciamos el DDS
    my_node = NavigationActionServer()
    try:
        while rclpy.ok:
            rclpy.spin_once(my_node, timeout_sec= 5)
    except KeyboardInterrupt:
        my_node.action_server.destroy()
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()