import rospy

def a_star(map_array, start, goal):
    from heapq import heappop, heappush

    # if map_array[start[1], start[0]] > 0 or map_array[goal[1], goal[0]] > 0:
    #     rospy.logwarn("Start or goal is in an obstacle!")
    #     return None

    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current, map_array):
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(open_set, (f_score[neighbor], neighbor))

    rospy.logwarn("Failed to find a path!")
    return None

def heuristic(node, goal):
    return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5

def get_neighbors(node, map_array):
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

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path