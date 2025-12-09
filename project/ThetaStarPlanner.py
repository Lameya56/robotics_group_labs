import numpy as np
import time
import heapq

class ThetaStarPlanner(object):
    def __init__(self, planning_env, epsilon):
        self.env = planning_env
        self.nodes = {}
        self.epsilon = epsilon
        self.visited = np.zeros(self.env.map.shape)
        self.shortcuts_taken = 0

    def line_of_sight(self, start, end):
        """Check if there's a clear line of sight between two points using Bresenham's algorithm"""
        x0, y0 = start
        x1, y1 = end

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0

        while True:
            # Check if current point is valid
            config = np.array([x, y]).reshape((2, 1))
            if not self.env.state_validity_checker(config):
                return False

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return True

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        # Theta* implementation
        start = tuple(start_config.flatten().astype(int))
        goal = tuple(goal_config.flatten().astype(int))
        # Priority queue for frontier (f, g, config)
        frontier = []
        heapq.heappush(frontier, (0, 0, start))
        # g-values and parent dictionary
        g_values = {start: 0}
        parents = {start: None}

        state_count = 0
        # Theta* main loop
        while frontier:
            _, g, current = heapq.heappop(frontier)

            # Skip if already visited
            if self.visited[current]:
                continue

            self.visited[current] = 1
            state_count += 1

            # Check goal condition
            current_config = np.array(current).reshape((2, 1))
            if self.env.goal_criterion(current_config, goal_config):
                print("Goal reached!")
                break

            # Explore neighbors (8-connected grid)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    neighbor = (current[0] + dx, current[1] + dy)
                    neighbor_config = np.array(neighbor).reshape((2, 1))

                    # Skip invalid or out-of-bounds
                    if not self.env.state_validity_checker(neighbor_config):
                        continue

                    # Theta* any-angle path check
                    parent_current = parents[current]

                    # Try path through parent if line of sight exists
                    if parent_current is not None and self.line_of_sight(parent_current, neighbor):
                        # Path 1: Connect neighbor directly to parent of current (any-angle)
                        parent_config = np.array(parent_current).reshape((2, 1))
                        new_cost_parent = g_values[parent_current] + self.env.compute_distance(parent_config, neighbor_config)

                        # Path 2: Standard A* connection through current
                        new_cost_current = g + self.env.compute_distance(current_config, neighbor_config)

                        # Choose the better path
                        if new_cost_parent < new_cost_current:
                            # Use parent path (any-angle shortcut)
                            if neighbor not in g_values or new_cost_parent < g_values[neighbor]:
                                g_values[neighbor] = new_cost_parent
                                f = new_cost_parent + self.epsilon * self.env.h(neighbor_config)
                                heapq.heappush(frontier, (f, new_cost_parent, neighbor))
                                parents[neighbor] = parent_current
                                self.shortcuts_taken += 1
                        else:
                            # Use current path
                            if neighbor not in g_values or new_cost_current < g_values[neighbor]:
                                g_values[neighbor] = new_cost_current
                                f = new_cost_current + self.epsilon * self.env.h(neighbor_config)
                                heapq.heappush(frontier, (f, new_cost_current, neighbor))
                                parents[neighbor] = current
                    else:
                        # No line of sight, use standard A* connection through current
                        new_cost = g + self.env.compute_distance(current_config, neighbor_config)

                        if neighbor not in g_values or new_cost < g_values[neighbor]:
                            g_values[neighbor] = new_cost
                            f = new_cost + self.epsilon * self.env.h(neighbor_config)
                            heapq.heappush(frontier, (f, new_cost, neighbor))
                            parents[neighbor] = current

        # Reconstruct path if goal found
        plan = []
        node = min(g_values.keys(), key=lambda n: self.env.compute_distance(np.array(n).reshape(2, 1), goal_config))
        while node is not None:
            plan.append(np.array(node).reshape((2, 1)))
            node = parents[node]
        plan.reverse()

        plan = np.concatenate(plan, axis=1)
        cost = g_values[min(g_values.keys(), key=lambda n: self.env.compute_distance(np.array(n).reshape(2, 1), goal_config))]

        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)
        print("Shortcuts Taken: %d" % self.shortcuts_taken)

        return plan