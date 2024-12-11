#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
import numpy as np

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
        """Compute a path from start to goal using A*."""
        if not self.start or not self.goal:
            rospy.logwarn("Start or goal is not set. Cannot plan path.")
            return

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position

        # Convert start and goal positions to grid coordinates
        start_grid = self.world_to_grid(self.start.position, origin, resolution)
        goal_grid = self.world_to_grid(self.goal.position, origin, resolution)

        if not (0 <= start_grid[0] < map_array.shape[1] and 0 <= start_grid[1] < map_array.shape[0]):
            rospy.logwarn("Start position is out of bounds!")
            return
        if not (0 <= goal_grid[0] < map_array.shape[1] and 0 <= goal_grid[1] < map_array.shape[0]):
            rospy.logwarn("Goal position is out of bounds!")
            return

        # Run A* algorithm to generate the path
        path = self.a_star(map_array, start_grid, goal_grid)

        # Publish the path
        if path:
            self.publish_path(path, origin, resolution)
        else:
            rospy.logwarn("No valid path found!")

    def world_to_grid(self, position, origin, resolution):
        """Convert world coordinates to grid coordinates."""
        grid_x = int((position.x - origin.x) / resolution)
        grid_y = int((position.y - origin.y) / resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid, origin, resolution):
        """Convert grid coordinates to world coordinates."""
        world_x = grid[0] * resolution + origin.x
        world_y = grid[1] * resolution + origin.y
        return world_x, world_y

    def a_star(self, map_array, start, goal):
        """A* algorithm for path planning."""
        from heapq import heappop, heappush

        if map_array[start[1], start[0]] > 0 or map_array[goal[1], goal[0]] > 0:
            rospy.logwarn("Start or goal is in an obstacle!")
            return None

        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current, map_array):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

        rospy.logwarn("Failed to find a path!")
        return None

    def heuristic(self, node, goal):
        """Heuristic function for A* (Euclidean distance)."""
        return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

    def get_neighbors(self, node, map_array):
        """Get valid neighbors for a node."""
        neighbors = []
        x, y = node
        height, width = map_array.shape

        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1),
                   (-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height and map_array[ny, nx] == 0:
                neighbors.append((nx, ny))

        return neighbors

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_path(self, path, origin, resolution):
        """Publish the global path as a ROS Path message."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for grid in path:
            world_x, world_y = self.grid_to_world(grid, origin, resolution)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published global path to /global_plan.")

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
