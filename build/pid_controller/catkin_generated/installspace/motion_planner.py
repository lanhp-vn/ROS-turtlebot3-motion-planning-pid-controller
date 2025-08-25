#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

def goal_reached_callback(msg):
    """Callback to update global goal_reached flag."""
    global goal_reached
    goal_reached = msg.data

if __name__ == "__main__":
    rospy.init_node("motion_planner", anonymous=True)
    ref_pub = rospy.Publisher("/reference_pose", Float64MultiArray, queue_size=10)
    goal_sub = rospy.Subscriber("/goal_reached", Bool, goal_reached_callback)

    # Global flag to track goal status
    goal_reached = False
    
    # Main loop: get user input for goals repeatedly
    while not rospy.is_shutdown():
        # 1. Get target pose and mode from user input
        try:
            x_r = float(input("Enter target X: "))
            y_r = float(input("Enter target Y: "))
            theta_r = float(input("Enter target orientation theta (rad): "))
            mode = int(input("Enter mode (0 for sequential, 1 for simultaneous): "))
        except ValueError:
            print("Invalid input")
            continue

        # 2. Publish the target as a Float64MultiArray on /reference_pose
        target_msg = Float64MultiArray()
        target_msg.data = [x_r, y_r, theta_r, float(mode)]
        ref_pub.publish(target_msg)
        rospy.loginfo(f"Published target pose ({x_r:.2f}, {y_r:.2f}, {theta_r:.2f}) in mode {mode}.")

        # 3. Wait until the robot reaches the goal (notified by pid_controller)
        rospy.loginfo("Waiting for the robot to reach the goal...")
        goal_reached = False  # Reset flag for new goal
        while not rospy.is_shutdown() and not goal_reached:
            rospy.sleep(0.1)  # Small sleep to prevent busy looping

        if goal_reached:
            rospy.loginfo("Goal reached")

        # Loop back to prompt for another goal (unless ROS is shutting down)