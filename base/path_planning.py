import numpy as np
import math

PAD_RADIUS = 2  # cells; tune based on rover width / RESOLUTION


def inflate_obstacles(grid, radius_cells):
    """Pad occupied cells outward by radius_cells so A* keeps clearance
    from obstacles instead of routing right along the edge of one."""
    inflated = grid.copy()
    occupied = np.argwhere(grid == 1)  # gives (row, col) = (y, x) pairs

    for y, x in occupied:
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:
                    if dx * dx + dy * dy <= radius_cells * radius_cells:
                        if inflated[ny][nx] != 1:
                            inflated[ny][nx] = 1
    return inflated


def get_neighbor(inflated_map, pose):
    """Return the (x,y) of all valid, unoccupied, in-bounds 8-directional neighbors."""
    neighbor = []
    x, y = pose

    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0:
                continue

            nx, ny = x + dx, y + dy

            if not (0 <= nx < inflated_map.shape[1] and 0 <= ny < inflated_map.shape[0]):
                continue

            if inflated_map[ny][nx] == 1:
                continue

            neighbor.append((nx, ny))

    return neighbor


def h(goal, current):
    """Euclidean distance heuristic."""
    dx = goal[0] - current[0]
    dy = goal[1] - current[1]
    return math.sqrt(dx ** 2 + dy ** 2)


def astar(master_map, x, y, current_pose):
    """
    master_map   : 2D numpy array (0=free, 1=occupied, 2=unknown)
    x, y         : goal coordinates
    current_pose : (x, y[, heading]) -- only x,y are used here
    returns      : list of (x,y) tuples from start to goal, or None if no path found
    """
    inflated_map = inflate_obstacles(master_map, PAD_RADIUS)
    goal = (x, y)
    start = (current_pose[0], current_pose[1])

    open_list = [{"pos": start, "g": 0, "h": h(goal, start), "parent": None}]
    closed_list = set()

    while open_list:
        current = min(open_list, key=lambda n: n["g"] + n["h"])
        open_list.remove(current)

        if current["pos"] == goal:
            path = []
            node = current
            while node is not None:
                path.append(node["pos"])
                node = node["parent"]
            return path[::-1]

        closed_list.add(current["pos"])

        for neighbor_pos in get_neighbor(inflated_map, current["pos"]):
            if neighbor_pos in closed_list:
                continue

            dx = neighbor_pos[0] - current["pos"][0]
            dy = neighbor_pos[1] - current["pos"][1]
            step_cost = math.sqrt(2) if (dx != 0 and dy != 0) else 1
            new_g = current["g"] + step_cost

            existing = next((n for n in open_list if n["pos"] == neighbor_pos), None)
            if existing and existing["g"] <= new_g:
                continue

            neighbor_node = {
                "pos": neighbor_pos,
                "g": new_g,
                "h": h(goal, neighbor_pos),
                "parent": current,
            }
            if existing:
                open_list.remove(existing)
            open_list.append(neighbor_node)

    return None  # no path found
