#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pid_controller.msg import ReferencePose

class PIDControllerNode:
    def __init__(self):
        # --- PID gains (tune these!) ---
        # Linear controller gains
        self.Kp_lin = rospy.get_param('~Kp_lin', 0.5)
        self.Ki_lin = rospy.get_param('~Ki_lin', 0.0)
        self.Kd_lin = rospy.get_param('~Kd_lin', 0.1)
        # Angular controller gains
        self.Kp_ang = rospy.get_param('~Kp_ang', 1.0)
        self.Ki_ang = rospy.get_param('~Ki_ang', 0.0)
        self.Kd_ang = rospy.get_param('~Kd_ang', 0.2)

        # --- State variables for PID ---
        self.lin_int  = 0.0    # integral term
        self.lin_prev = 0.0    # previous error
        self.ang_int  = 0.0
        self.ang_prev = 0.0

        # Thresholds
        self.dist_thresh = 0.01   # meters
        self.angle_thresh = 0.01  # radians

        # Reference and current pose
        self.xr = self.yr = self.theta_r = 0.0
        self.mode = 0
        self.x = self.y = self.theta = 0.0

        # State for sequential mode
        self.seq_step = 0

        # ROS interfaces
        rospy.init_node('pid_controller', anonymous=False)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/reference_pose', ReferencePose, self.ref_callback)
        rospy.Subscriber('/odom',           Odometry,       self.odom_callback)

        # Control loop at 10 Hz
        self.rate = rospy.Rate(10)
        rospy.loginfo("PID Controller node initialized.")

    def ref_callback(self, msg):
        """Handle new goal: reset integrals and state machine."""
        self.xr, self.yr = msg.xr, msg.yr
        self.theta_r   = msg.theta_r
        self.mode      = msg.mode
        # Reset PID internals
        self.lin_int = self.ang_int = 0.0
        self.lin_prev = self.ang_prev = 0.0
        # Reset sequential step
        self.seq_step = 0
        rospy.loginfo(f"New goal: x={self.xr:.2f}, y={self.yr:.2f}, θ={self.theta_r:.2f}, mode={self.mode}")

    def odom_callback(self, msg):
        """Update current pose from Odometry."""
        pose = msg.pose.pose
        self.x = pose.position.x
        self.y = pose.position.y
        # Convert quaternion to yaw
        q = pose.orientation
        siny_cosp = 2.0 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_to_pi(angle):
        """Normalize angle to [-π, π]."""
        return (angle + math.pi) % (2*math.pi) - math.pi

    def compute_pid(self, error, prev_error, integral, Kp, Ki, Kd, dt):
        """
        Generic PID compute:
          P = Kp * error
          I = Ki * (integral + error*dt)
          D = Kd * (error - prev_error)/dt
        Returns (control, new_integral, new_prev_error)
        """
        integral_new = integral + error * dt
        derivative   = (error - prev_error) / dt if dt > 0 else 0.0
        output = Kp*error + Ki*integral_new + Kd*derivative
        return output, integral_new, error

    def run(self):
        """Main control loop."""
        dt = 0.1  # seconds (1/rate)
        while not rospy.is_shutdown():
            # Compute errors
            dx = self.xr - self.x
            dy = self.yr - self.y
            dist_error = math.hypot(dx, dy)
            # Desired heading to goal
            heading = math.atan2(dy, dx)
            ang_error  = self.wrap_to_pi(heading - self.theta)
            final_error = self.wrap_to_pi(self.theta_r - self.theta)

            u_lin = u_ang = 0.0

            if self.mode == 0:
                # --- Sequential mode ---
                if self.seq_step == 0:
                    # Rotate to face goal
                    if abs(ang_error) > self.angle_thresh:
                        u_ang, self.ang_int, self.ang_prev = self.compute_pid(
                            ang_error, self.ang_prev, self.ang_int,
                            self.Kp_ang, self.Ki_ang, self.Kd_ang, dt
                        )
                    else:
                        self.seq_step = 1  # move to next phase
                        rospy.loginfo("Phase 1 complete: facing goal.")
                elif self.seq_step == 1:
                    # Drive straight to goal
                    if dist_error > self.dist_thresh:
                        u_lin, self.lin_int, self.lin_prev = self.compute_pid(
                            dist_error, self.lin_prev, self.lin_int,
                            self.Kp_lin, self.Ki_lin, self.Kd_lin, dt
                        )
                    else:
                        self.seq_step = 2
                        rospy.loginfo("Phase 2 complete: at goal location.")
                else:
                    # Rotate to final orientation
                    if abs(final_error) > self.angle_thresh:
                        u_ang, self.ang_int, self.ang_prev = self.compute_pid(
                            final_error, self.ang_prev, self.ang_int,
                            self.Kp_ang, self.Ki_ang, self.Kd_ang, dt
                        )
                    else:
                        # All done; zero velocities
                        u_lin = u_ang = 0.0

            else:
                # --- Simultaneous mode ---
                # Linear PID on distance
                u_lin, self.lin_int, self.lin_prev = self.compute_pid(
                    dist_error, self.lin_prev, self.lin_int,
                    self.Kp_lin, self.Ki_lin, self.Kd_lin, dt
                )
                # Angular PID on heading error
                u_ang, self.ang_int, self.ang_prev = self.compute_pid(
                    ang_error, self.ang_prev, self.ang_int,
                    self.Kp_ang, self.Ki_ang, self.Kd_ang, dt
                )

            # Publish Twist
            cmd = Twist()
            cmd.linear.x  = max(min(u_lin, 1.0), -1.0)    # clamp to [-1, 1] m/s
            cmd.angular.z = max(min(u_ang, 2.0), -2.0)    # clamp to [-2, 2] rad/s
            self.cmd_pub.publish(cmd)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = PIDControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
