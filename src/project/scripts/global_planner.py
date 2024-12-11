#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
import numpy as np
from scipy.ndimage import grey_dilation


class GlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner', anonymous=False)
        rospy.loginfo('Running global planner...')

        # Publisher for the global path
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=10)

        # Subscriber to the goal position
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # Get Static map service
        rospy.wait_for_service('static_map')
        self.get_map = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = self.get_map().map

        # Obstacle padding radius (in meters)
        self.padding_radius = 0.25

        self.start = None
        self.goal = None

    def goal_callback(self, msg):
        """Callback to handle new goal."""
        self.goal = msg.pose
        rospy.loginfo(f'New goal received: {self.goal}')

        if self.start:
            self.plan_path()

    def get_start_position(self):
        """Get the robot's current position using AMCL or a similar localization method."""
        try:
            odom_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
            self.start = odom_pose.pose.pose
        except rospy.ROSException:
            rospy.logwarn('Failed to get start position')

    def plan_path(self):
        """Compute a path from start to goal using continuous-space A*."""
        if not self.start or not self.goal:
            rospy.logwarn("Start or goal is not set. Cannot plan path.")
            return

        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        # Convert start and goal positions to continuous coordinates
        start_pos = (self.start.position.x, self.start.position.y)
        goal_pos = (self.goal.position.x, self.goal.position.y)

        # Create padded map
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )
        padded_map = self.add_padding(map_array, self.padding_radius, resolution)

        # Run A* algorithm to generate the path
        path = self.a_star(padded_map, start_pos, goal_pos, resolution, origin)

        # Smooth the path
        if path:
            smoothed_path = self.smooth_path(path, weight_data=0.5, weight_smooth=0.3, tolerance=0.0001)
            self.publish_path(smoothed_path, resolution)
        else:
            rospy.logwarn("No valid path found!")

    def add_padding(self, map_array, padding_radius, resolution):
        """Add padding around obstacles in the map."""
        padding_cells = int(padding_radius / resolution)
        padded_map = grey_dilation(map_array, size=(2 * padding_cells + 1, 2 * padding_cells + 1))
        padded_map[map_array == -1] = -1  # Preserve unknown cells as -1
        return padded_map

    def is_free_space(self, position, map_array, resolution, origin):
        """Check if a position in continuous space is free."""
        x, y = position
        grid_x = int((x - origin.x) / resolution)
        grid_y = int((y - origin.y) / resolution)

        if 0 <= grid_x < map_array.shape[1] and 0 <= grid_y < map_array.shape[0]:
            return map_array[grid_y, grid_x] == 0
        return False

    def a_star(self, map_array, start, goal, resolution, origin):
        """Continuous-space A* algorithm for path planning."""
        from heapq import heappop, heappush

        if not self.is_free_space(start, map_array, resolution, origin) or not self.is_free_space(goal, map_array, resolution, origin):
            rospy.logwarn("Start or goal is in an obstacle!")
            return None

        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heappop(open_set)

            if self.distance(current, goal) < resolution:  # Goal is reached
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current, resolution):
                if not self.is_free_space(neighbor, map_array, resolution, origin):
                    continue

                tentative_g_score = g_score[current] + self.distance(current, neighbor)
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        rospy.logwarn("Failed to find a path!")
        return None

    def get_neighbors(self, current, resolution):
        """Generate neighbors in continuous space."""
        neighbors = []
        x, y = current
        offsets = [
            (-resolution, 0), (resolution, 0),
            (0, -resolution), (0, resolution),
            (-resolution, -resolution), (-resolution, resolution),
            (resolution, -resolution), (resolution, resolution)
        ]

        for dx, dy in offsets:
            neighbors.append((x + dx, y + dy))
        return neighbors

    def heuristic(self, node, goal):
        """Heuristic function for A*."""
        return self.distance(node, goal)

    def distance(self, point1, point2):
        """Calculate Euclidean distance between two points."""
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def smooth_path(self, path, weight_data=0.5, weight_smooth=0.3, tolerance=0.0001):
        """Smooth the path using an iterative smoothing algorithm."""
        new_path = [list(p) for p in path]
        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):  # Skip the first and last points
                for j in range(2):  # x and y
                    old_value = new_path[i][j]
                    new_path[i][j] += weight_data * (path[i][j] - new_path[i][j])
                    new_path[i][j] += weight_smooth * (new_path[i - 1][j] + new_path[i + 1][j] - 2 * new_path[i][j])
                    change += abs(old_value - new_path[i][j])
        return [(p[0], p[1]) for p in new_path]

    def publish_path(self, path, resolution):
        """Publish the global path as a ROS Path message."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published global path to /global_path.")

    def run(self):
        """Run the global planner node."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.get_start_position()
            rate.sleep()


if __name__ == '__main__':
    try:
        planner = GlobalPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
