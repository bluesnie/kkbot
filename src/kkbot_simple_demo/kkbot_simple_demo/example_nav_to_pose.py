#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    rclpy.init()
    
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 3.14
    initial_pose.pose.position.y = 2.15
    initial_pose.pose.orientation.z = 1.0
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)


    navigator.waitUntilNav2Active()  # autostart == True
    # navigator.lifecycleStartup()  # autostart == False

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp  = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i% 5 == 0:
            navigator.get_logger().info(f"Est time of arrival: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} seconds")
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600):
                navigator.cancelTask()
            
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18):
                goal_pose.pose.position.x = 1.5
                navigator.goToPose(goal_pose)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info("Goal successed!")
    elif result == TaskResult.CANCELED:
        navigator.get_logger().info("Goal was canceled!")
    elif result == TaskResult.FAILED:
        navigator.get_logger().info("Goal failed")
    else:
        navigator.get_logger().info("Goal has an invalid return status!")
    navigator.lifecycleShutdown()
    exit(0)

if __name__ == "__main__":
    main()

