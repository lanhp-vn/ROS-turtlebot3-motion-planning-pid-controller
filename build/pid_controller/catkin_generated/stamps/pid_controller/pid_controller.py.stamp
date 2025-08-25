#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# --- PID Controller Parameters and State ---
# PID gains for linear (distance) control and angular (heading) control
Kp_lin, Ki_lin, Kd_lin = 0.4, 0.0001, 0.0001
Kp_ang, Ki_ang, Kd_ang = 1, 0.0001, 0.0001

# State variables for PID calculations
linear_error_sum = 0.0      # Integrative term (linear)
angular_error_sum = 0.0     # Integrative term (angular)
prev_linear_error = 0.0     # For derivative term (linear)
prev_angular_error = 0.0    # For derivative term (angular)

# Target (reference) pose and mode (received from /reference_pose topic)
target_x = 0.0
target_y = 0.0
target_theta = 0.0
mode = 0
target_received = False     # Flag to indicate a new target has been received

# Current robot pose (from odometry)
current_x = 0.0
current_y = 0.0
current_yaw = 0.0

# Tolerances for considering the goal reached
pos_tolerance = 0.1    # 10 cm
ang_tolerance = 0.1    # 0.1 rad ≈ 5.7°

# Flag to indicate if the goal has been reached
goal_reached_flag = False

# --- Callback Functions ---

def reference_pose_callback(msg):
    """Callback for /reference_pose topic. Receives target [x, y, theta, mode]."""
    global target_x, target_y, target_theta, mode, target_received, goal_reached_flag
    global linear_error_sum, angular_error_sum, prev_linear_error, prev_angular_error
    if len(msg.data) >= 4:
        target_x = msg.data[0]
        target_y = msg.data[1]
        target_theta = msg.data[2]
        mode = int(msg.data[3])
        target_received = True
        goal_reached_flag = False  # Reset flag for a new target
        # Reset PID accumulators for a fresh goal
        linear_error_sum = 0.0
        angular_error_sum = 0.0
        prev_linear_error = 0.0
        prev_angular_error = 0.0
        rospy.loginfo(f"Received new target: ({target_x:.2f}, {target_y:.2f}, {target_theta:.2f}) mode={mode}")
    else:
        rospy.logwarn("reference_pose message did not contain 4 elements")

def odom_callback(msg):
    """Callback for /odom topic. Updates current robot pose (x, y, yaw)."""
    global current_x, current_y, current_yaw
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    current_yaw = math.atan2(siny_cosp, cosy_cosp)
    rospy.loginfo(f"CURRENT: ({current_x:.2f}, {current_y:.2f}, {current_yaw:.2f})")

# --- Main PID Control Loop ---
if __name__ == "__main__":
    rospy.init_node("pid_controller", anonymous=True)
    rospy.Subscriber("/reference_pose", Float64MultiArray, reference_pose_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    goal_pub = rospy.Publisher("/goal_reached", Bool, queue_size=10)

    rate = rospy.Rate(10)
    prev_time = rospy.Time.now()

    while not rospy.is_shutdown():
        if not target_received or goal_reached_flag:
            # No target or goal reached: publish zero velocities
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            cmd_pub.publish(stop_msg)
            rate.sleep()
            continue

        current_time = rospy.Time.now()
        dt = (current_time - prev_time).to_sec()
        prev_time = current_time
        if dt == 0:
            rate.sleep()
            continue

        # Calculate errors
        dx = target_x - current_x
        dy = target_y - current_y
        distance_error = math.sqrt(dx*dx + dy*dy)
        rospy.loginfo(f"Distance error: {distance_error:.2f}")
        desired_angle_to_target = math.atan2(dy, dx)
        angle_to_target_error = desired_angle_to_target - current_yaw
        angle_to_target_error = math.atan2(math.sin(angle_to_target_error), math.cos(angle_to_target_error))
        rospy.loginfo(f"Angle to target error: {angle_to_target_error:.2f}")
        final_orientation_error = target_theta - current_yaw
        final_orientation_error = math.atan2(math.sin(final_orientation_error), math.cos(final_orientation_error))
        rospy.loginfo(f"final orientation error: {final_orientation_error:.2f}")

        # Check if goal is reached
        if distance_error <= pos_tolerance and abs(final_orientation_error) <= ang_tolerance:
            goal_msg = Bool()
            goal_msg.data = True
            goal_pub.publish(goal_msg)
            goal_reached_flag = True
            linear_error_sum = 0.0  # Reset integral terms
            angular_error_sum = 0.0
            linear_error = 0.0
            angular_error = 0.0
        else:
            # Mode-specific control logic
            if mode == 0:
                if distance_error > 0.1:
                    if abs(angle_to_target_error) > 0.1:
                        linear_error = 0.0
                        angular_error = angle_to_target_error
                    else:
                        linear_error = distance_error
                        angular_error = 0.0
                else:
                    linear_error = 0.0
                    angular_error = final_orientation_error
            else:
                if distance_error > 0.1:
                    linear_error = distance_error
                    angular_error = angle_to_target_error
                else:
                    linear_error = 0.0
                    angular_error = final_orientation_error

        # PID calculations
        lin_P = Kp_lin * linear_error
        linear_error_sum += linear_error * dt
        lin_I = Ki_lin * linear_error_sum
        lin_D = Kd_lin * ((linear_error - prev_linear_error) / dt)
        linear_cmd = lin_P + lin_I + lin_D
        prev_linear_error = linear_error

        ang_P = Kp_ang * angular_error
        angular_error_sum += angular_error * dt
        ang_I = Ki_ang * angular_error_sum
        ang_D = Kd_ang * ((angular_error - prev_angular_error) / dt)
        angular_cmd = ang_P + ang_I + ang_D
        prev_angular_error = angular_error

        # Clamp velocities
        max_lin = 0.3
        max_ang = 1.5
        linear_cmd = max(min(linear_cmd, max_lin), -max_lin)
        angular_cmd = max(min(angular_cmd, max_ang), -max_ang)

        # Ensure zero velocity when goal is reached
        if goal_reached_flag:
            linear_cmd = 0.0
            angular_cmd = 0.0

        # Publish velocity command
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_cmd
        cmd_msg.angular.z = angular_cmd
        cmd_pub.publish(cmd_msg)

        rate.sleep()