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

    def _build_visibility_graph(self, vertices):
        '''construct the visibility graph from the group of vertices'''
        # initialize dict as the visibility graph
        graph = {}
        
        # key represents vertice # and value represents a vertice -> (vertice #, distance)
        # where distance calculates the key vertice and the value vertice
        for i in range(len(vertices)):
            graph[i] = []
        
        # check every vertice with each vertice
        # if connecting edge between two vertices doesn't hit an obstacle,
        # calculate the distance between each vertice
        # update each vertice's distance from each other in the graph dict
        for i in range(len(vertices)):
            for j in range(i + 1, len(vertices)):
                if self.env.edge_validity_checker(vertices[i], vertices[j]):
                    distance = self.env.compute_distance(vertices[i], vertices[j])
                    graph[i].append((j, distance))
                    graph[j].append((i, distance))
        
        return graph

    def _dijkstra(self, graph, vertices, start_idx, goal_idx, max_states):
        ''' find the shortest path from start to goal using Dijkstra's algorithm
            modified code from GeeksforGeeks '''
        
        # initialize distance value of each node to infinity
        # initialize previous value node to None, for pathfinding
        dist = {}
        prev = {}
        for node in graph:
            dist[node] = float('inf')
            prev[node] = None
    
        dist[start_idx] = 0 # distance to start is 0, bc we find path from start to goal
        
        # put start index tuple in priority queue, we put distance value first so pq can sort by dist
        pq = [(0, start_idx)]
        
        state_count = 0
        
        while pq:
            # remove the smallest node by dist, get the dist and node separately
            current_dist, current_node = heapq.heappop(pq)
            state_count += 1
            
            # if shortest path searching takes too long, exit loop and get results without reaching goal
            if state_count >= max_states:
                print("Maximum states reached. Exiting without reaching goal.")
                break
            
            # we reached the goal, so stop searching for best path
            if current_node == goal_idx:
                break
            
            # the current distance is bigger/worse than the prev path to node
            if current_dist > dist[current_node]:
                continue
            
            # get the adjacent nodes from curr node that "robot" is on
            # if prev path to an adjacent node is bigger/worse than path from current_node
            # update best dist for adjacent node and the prev node to it
            # push the index tuple to the pq
            # pq automatically sorts nodes so nodes w/ shortest path are searched first
            for adj, weight in graph[current_node]:
                if dist[adj] > current_dist + weight:
                    dist[adj] = current_dist + weight
                    prev[adj] = current_node
                    heapq.heappush(pq, (current_dist + weight, adj))
        
        # get shortest path once goal is reached
        path = []
        current = goal_idx
        # exits loop when current is at start
        while current is not None:
            path.append(vertices[current])  # add vertices that were visited on path
            current = prev[current] # get the next vertice in reverse order
        
        path.reverse() # flip list so start is first and goal is last
        
        # return the path list, cost of going from start->goal, and the state_count
        return path, dist[goal_idx], state_count

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        path = []
        cost = 0
        state_count = 0
        max_states = 100000 # when dijkstra infinitely runs
        
        # Get obstacle vertices
        obs_vertices = self._get_obstacle_vertices()
        
        # vertices list contains the start, goal, and obstacles
        vertices = [start_config, goal_config] + obs_vertices
        
        # build visibility graph
        graph = self._build_visibility_graph(vertices)
        
        # run dijkstra's algorithm, start is index 0 and goal is index 1 in vertices list, max_states
        path, cost, state_count = self._dijkstra(graph, vertices, 0, 1, max_states)
        
        if not path:
            # use direct start->goal path as plan
            path = [start_config, goal_config]
            print("No optimal path was found. Using direct path.")
        
        plan_time = time.time() - plan_time

        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)

        return np.hstack(path)