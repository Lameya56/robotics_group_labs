import numpy as np
import time
import heapq
import math

class VisibilityGraphPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env

    def _get_obstacle_vertices(self):
        """
        A heuristic to extract obstacle corners from a grid map. It identifies
        2x2 blocks that represent a corner and adds the free-space cells
        from that block as vertices.
        """
        map_ = self.env.map
        vertices = set()
        # Iterate over all possible 2x2 block top-left corners
        for r in range(map_.shape[0] - 1):
            for c in range(map_.shape[1] - 1):
                block = map_[r:r+2, c:c+2]
                # A corner is where free space meets obstacle space.
                # Sum of 1 or 3 indicates a corner in the 2x2 block.
                if np.sum(block) == 1 or np.sum(block) == 3:
                    # Find the free-space cell(s) in this block
                    for dr, dc in [(0, 0), (0, 1), (1, 0), (1, 1)]:
                        check_r, check_c = r + dr, c + dc
                        if map_[check_r, check_c] == 0:
                            # Add the coordinates of the valid, free-space cell
                            vertices.add((check_r, check_c))
        
        # Convert the set of tuple coordinates to the required list of numpy arrays
        return [np.array(v).reshape(2, 1) for v in vertices]

    def _is_visible(self, config1, config2):
        """Check if two configurations are visible to each other (no obstacles in between)."""
        # delegate to the environment's edge checker
        return self.env.edge_validity_checker(config1, config2)

    def _build_visibility_graph(self, vertices):
        """Build a visibility graph from the given vertices."""
        graph = {}
        
        # Add all vertices to the graph
        # initialize adjacency list for every vertex index
        for i in range(len(vertices)):
            graph[i] = []
        
        # Check visibility between all pairs of vertices
        # check visibility pairwise (undirected)
        for i in range(len(vertices)):
            for j in range(i + 1, len(vertices)):
                if self._is_visible(vertices[i], vertices[j]):
                    distance = self.env.compute_distance(vertices[i], vertices[j])
                    graph[i].append((j, distance))
                    graph[j].append((i, distance))
        
        return graph

    def _dijkstra(self, graph, vertices, start_idx, goal_idx):
        # Using Dijkstra's algorithm to find the shortest path.

        # Initialize distances and previous nodes
        dist = {node: float('inf') for node in graph}
        prev = {node: None for node in graph}
        dist[start_idx] = 0
        
        # Priority queue: (distance, node_index)
        pq = [(0, start_idx)]
        
        state_count = 0
        
        while pq:
            current_dist, current_node = heapq.heappop(pq)
            state_count += 1
            
            # If we reached the goal, we can stop
            if current_node == goal_idx:
                break
            
            # If we found a better path already, skip
            if current_dist > dist[current_node]:
                continue
            
            # Explore neighbors
            for neighbor, weight in graph[current_node]:
                new_dist = current_dist + weight
                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    prev[neighbor] = current_node
                    heapq.heappush(pq, (new_dist, neighbor))
        
        # Reconstruct path
        path = []
        current = goal_idx
        while current is not None:
            path.append(vertices[current])
            current = prev[current]
        
        path.reverse()
        
        return path, dist[goal_idx], state_count

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        
        # Get obstacle vertices
        obs_vertices = self._get_obstacle_vertices()
        
        # Create vertex list including start and goal
        # start and goal are first two indices (0 and 1)
        vertices = [start_config, goal_config] + obs_vertices
        
        # Build visibility graph
        graph = self._build_visibility_graph(vertices)
        
        # Run Dijkstra's algorithm (start is index 0, goal is index 1)
        plan_path, cost, state_count = self._dijkstra(graph, vertices, 0, 1)
        
        # Convert plan to the correct format [2 x n]
        # Each element in plan_path is a [2, 1] array, we need to stack them horizontally
        # stack the resulting column vectors into a [2 x n] plan
        if plan_path:
            plan = np.hstack(plan_path)
        else:
            # fallback: return direct start->goal if no path found
            plan = np.hstack([start_config, goal_config])
        
        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)

        return plan