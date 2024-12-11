#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion
import math
from sensor_msgs.msg import LaserScan
import numpy as np


class PathFollower:
    def __init__(self):
        rospy.init_node("path_follower", anonymous=False)

        # Subscribe to the planned path
        self.path_sub = rospy.Subscriber("/global_path", Path, self.path_callback)
        
        # Subscribe to the odometry data
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.trajectory = rospy.Subscriber('/trajectory_planner', Twist, self.traj_callback)

        # Publisher to send velocity commands
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscriber for the lidar messages
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.current_path = None  # Stores the received path
        self.current_goal_index = 0  # Index of the current target in the path
        self.current_pose = None  # Stores the robot's current pose

        # Parameters for control
        self.linear_gain = 1.2  # Gain for linear velocity
        self.angular_gain = 2.0  # Gain for angular velocity
        self.max_linear_velocity = 1.0  # Maximum linear velocity (m/s)
        self.max_angular_velocity = 0.75  # Maximum angular velocity (rad/s)
        self.distance_tolerance = 0.2  # Distance tolerance to consider a waypoint reached
        self.angle_tolerance = 0.2  # Angle tolerance to align to a waypoint
        self.look_ahead_distance = 0.5  # Distance to look ahead on the path for smoother motion
        self.obstacle_detected = False

        self.trajectory_cmd = None

        # Control loop
        self.rate = rospy.Rate(20)  
        self.control_loop()

    def traj_callback(self, msg):
        self.trajectory_cmd = msg

    def scan_callback(self, msg):
        scan_ranges = np.array(msg.ranges)
        scan_ranges = np.nan_to_num(scan_ranges, nan=0.0, posinf=msg.range_max, neginf=0.0)
        for i in scan_ranges:
            if i < 1.0:
                self.obstacle_detected = True
                return
        # self.obstacle_detected = False

    def path_callback(self, msg):
        """Callback to store the received path."""
        self.current_path = msg.poses
        self.current_goal_index = 0
        self.obstacle_detected = False
        rospy.loginfo("Received a new path with %d waypoints", len(self.current_path))

    def odom_callback(self, msg):
        """Callback to store the robot's current pose."""
        self.current_pose = msg.pose.pose

    def is_near_goal(self):
        """Check if the robot is within 0.5 meters from the goal."""
        if self.current_path is None or self.current_pose is None:
            return False

        # Get the final goal position
        goal_pose = self.current_path[-1].pose
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y

        # Get the robot's current position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Calculate the distance to the goal
        distance_to_goal = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

        # Return True if within 0.5 meters
        return distance_to_goal <= 0.5

    def control_loop(self):
        """Main loop to control the robot to follow the path."""
        while not rospy.is_shutdown():
            if self.obstacle_detected and self.trajectory_cmd:
                self.cmd_vel_pub.publish(self.trajectory_cmd)
                # rospy.loginfo("Obstacle detected. Switching to trajectory planner.")
            elif self.current_path is not None and self.current_pose is not None:
                if self.current_goal_index < len(self.current_path):
                    # Get the current waypoint or look-ahead point
                    current_goal = self.get_look_ahead_point()

                    # Control the robot to move toward the goal
                    self.move_to_goal(current_goal)
                if self.is_near_goal():
                    rospy.loginfo("Reached the final goal.")
                    self.stop_robot()
                    self.current_path = None  # Clear the path to wait for the next plan
            else:
                self.stop_robot()

            self.rate.sleep()

    def get_look_ahead_point(self):
        """Find the look-ahead point along the path for smoother motion."""
        while self.current_goal_index < len(self.current_path):
            current_goal = self.current_path[self.current_goal_index]
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            goal_x = current_goal.pose.position.x
            goal_y = current_goal.pose.position.y

            distance = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)
            if distance > self.look_ahead_distance:
                return current_goal
            self.current_goal_index += 1
        return self.current_path[-1]  # Default to the last point

    def move_to_goal(self, goal_pose):
        """
        Move the robot toward the given goal pose.
        """
        if self.current_pose is None:
            return

        # Extract goal position
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        # Extract robot position
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        # Calculate distance and angle to goal
        distance = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)

        # Calculate robot orientation (yaw)
        orientation = self.current_pose.orientation
        _, _, robot_yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        # Calculate desired yaw
        angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
        angle_diff = self.normalize_angle(angle_to_goal - robot_yaw)

        # Calculate curvature (turning radius inverse)
        if abs(angle_diff) > self.angle_tolerance:
            curvature = 2 * math.sin(angle_diff) / distance
        else:
            curvature = 0

        # Adjust velocities for smooth arc motion
        cmd = Twist()
        cmd.linear.x = max(0.0, min(self.linear_gain * (1 - abs(curvature)), self.max_linear_velocity))
        cmd.angular.z = max(-self.max_angular_velocity, min(self.angular_gain * angle_diff, self.max_angular_velocity))

        # Publish the velocity command
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def normalize_angle(self, angle):
        """Normalize an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


if __name__ == "__main__":
    try:
        follower = PathFollower()
    except rospy.ROSInterruptException:
        pass
