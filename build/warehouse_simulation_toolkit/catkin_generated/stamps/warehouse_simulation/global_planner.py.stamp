import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetMap
from tf.transformations import euler_from_quaternion
import numpy as np

class GlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner', anonymous=False)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # Get Static map
        rospy.wait_for_service('static_map')
        self.get_map = rospy.ServiceProxy('static_map', GetMap)
        self.map_data = self.get_map().map

        self.start = None
        self.goal = None

    def goal_callback(self, msg):
        self.goal = msg.pose
        rospy.loginfo(f'New goal received: {self.goal}')

        if self.start:
            self.plan_path()

    
    def get_start_position(self):
        try:
            odom_pose = rospy.wait_for_message('/amcl_pose', PoseStamped, timeout=5)
            self.start = odom_pose.pose
            rospy.loginfo(f'Start position: {self.start}')
        except rospy.ROSException:
            rospy.logwarn('Failed to get start position')


    def plan_path(self):
        """Compute a path from start to goal using A* or another algorithm."""
        if not self.start or not self.goal:
            rospy.logwarn("Start or goal is not set. Cannot plan path.")
            return

        # Convert map to a numpy array for easier manipulation
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

        # Run your custom path planning algorithm (e.g., A*)
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
        """
        A* algorithm to find the shortest path in a 2D grid.
        :param map_array: 2D numpy array representing the map (0: free, >0: obstacle)
        :param start: Tuple (x, y) start grid coordinates
        :param goal: Tuple (x, y) goal grid coordinates
        :return: List of grid coordinates representing the path
        """
        from heapq import heappop, heappush

        # Ensure start and goal are within bounds and valid
        if map_array[start[1], start[0]] > 0 or map_array[goal[1], goal[0]] > 0:
            rospy.logwarn("Start or goal is in an obstacle!")
            return None

        # A* setup
        open_set = []
        heappush(open_set, (0, start))  # Priority queue of (f_score, node)
        came_from = {}  # Tracks the optimal parent for each node

        g_score = {start: 0}  # Cost from start to this node
        f_score = {start: self.heuristic(start, goal)}  # Estimated cost to goal

        # A* loop
        while open_set:
            _, current = heappop(open_set)

            # If goal is reached, reconstruct the path
            if current == goal:
                return self.reconstruct_path(came_from, current)

            # Check neighbors
            for neighbor in self.get_neighbors(current, map_array):
                tentative_g_score = g_score[current] + 1  # All moves have a cost of 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    # Update scores
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    came_from[neighbor] = current

                    # Push to open_set if not already there
                    if neighbor not in [i[1] for i in open_set]:
                        heappush(open_set, (f_score[neighbor], neighbor))

        rospy.logwarn("Failed to find a path!")
        return None

    def heuristic(self, node, goal):
        """
        Heuristic function for A* (Euclidean distance).
        :param node: Current node as (x, y)
        :param goal: Goal node as (x, y)
        :return: Estimated cost from node to goal
        """
        return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

    def get_neighbors(self, node, map_array):
        """
        Get valid neighboring cells for a node.
        :param node: Current node as (x, y)
        :param map_array: 2D numpy array representing the map
        :return: List of valid neighbors
        """
        neighbors = []
        x, y = node
        height, width = map_array.shape

        # Potential neighbor offsets (8-connectivity)
        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1),  # Cardinal directions
                   (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Diagonals

        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:  # Ensure within bounds
                if map_array[ny, nx] == 0:  # Free cell
                    neighbors.append((nx, ny))

        return neighbors

    def reconstruct_path(self, came_from, current):
        """
        Reconstruct the path from start to goal.
        :param came_from: Dictionary of node: parent_node
        :param current: Current node (goal node)
        :return: List of nodes representing the path
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()  # Reverse the path to go from start to goal
        return path
    
    def publish_path(self, path, origin, resolution):
        """Publish the planned path as a nav_msgs/Path message."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        for grid in path:
            world_x, world_y = self.grid_to_world(grid, origin, resolution)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0  # Default orientation
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        rospy.loginfo("Published path")

    def run(self):
        """Run the global planner node."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.get_start_position()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = GlobalPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
