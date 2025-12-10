import numpy as np
import time
from collections import deque

class PotentialFieldsPlanner(object):
    def __init__(self, planning_env):
        self.env = planning_env
        self.kp = 5.0  # attractive potential gain
        self.eta = 100.0  # repulsive potential gain
        self.oscillations_detection_length = 3

    def Plan(self, start_config, goal_config):
        plan_time = time.time()
        plan = [start_config]
        state_count = 0
        
        # Get grid information
        start_x, start_y = start_config[0, 0], start_config[1, 0]
        goal_x, goal_y = goal_config[0, 0], goal_config[1, 0]
        
        # Grid resolution based on map
        reso = 1.0
        rr = 2.0  # robot radius for repulsive field
        
        # Compute potential field
        pmap, minx, miny = self._calc_potential_field(
            goal_x, goal_y, start_x, start_y, reso, rr)
        
        # Search path via gradient descent
        d = self.env.compute_distance(start_config, goal_config)
        ix = int(round((start_x - minx) / reso))
        iy = int(round((start_y - miny) / reso))
        gix = int(round((goal_x - minx) / reso))
        giy = int(round((goal_y - miny) / reso))
        
        motion = self._get_motion_model()
        previous_ids = deque()
        
        while d >= reso:
            minp = float("inf")
            minix, miniy = -1, -1
            
            # Find minimum potential in neighborhood
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                
                if inx >= len(pmap) or iny >= len(pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")
                else:
                    p = pmap[inx][iny]
                
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            
            ix = minix
            iy = miniy
            xp = ix * reso + minx
            yp = iy * reso + miny
            current_config = np.array([[xp], [yp]])
            d = self.env.compute_distance(current_config, goal_config)
            plan.append(current_config)
            state_count += 1
            
            # Detect oscillations
            if self._oscillations_detection(previous_ids, ix, iy):
                print("Oscillation detected at ({},{})!".format(ix, iy))
                break
        
        plan_time = time.time() - plan_time
        
        # Compute path cost using environment function
        cost = sum(self.env.compute_distance(plan[i], plan[i+1]) for i in range(len(plan)-1))
        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)
        
        return np.hstack(plan)
    
    def _calc_potential_field(self, goal_x, goal_y, start_x, start_y, reso, rr):
        """Calculate potential field based on grid map."""
        minx = min(start_x, goal_x) - 5
        miny = min(start_y, goal_y) - 5
        maxx = max(start_x, goal_x) + 5
        maxy = max(start_y, goal_y) + 5
        
        xw = int(round((maxx - minx) / reso))
        yw = int(round((maxy - miny) / reso))
        
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]
        
        for ix in range(xw):
            x = ix * reso + minx
            for iy in range(yw):
                y = iy * reso + miny
                
                # Check if position is valid using environment's state_validity_checker
                config = np.array([[x], [y]])
                if self.env.state_validity_checker(config):
                    ug = self._calc_attractive_potential(x, y, goal_x, goal_y)
                    uo = self._calc_repulsive_potential(x, y, rr)
                    uf = ug + uo
                    pmap[ix][iy] = uf
                else:
                    pmap[ix][iy] = float("inf")
        
        return pmap, minx, miny
    
    def _calc_attractive_potential(self, x, y, goal_x, goal_y):
        """Calculate attractive potential towards goal."""
        return 0.5 * self.kp * np.hypot(x - goal_x, y - goal_y)
    
    def _calc_repulsive_potential(self, x, y, rr):
        """Calculate repulsive potential from obstacles."""
        # Find nearest obstacle in grid
        min_dist = float("inf")
        map_x = int(round(x))
        map_y = int(round(y))
        
        for dx in range(-int(rr) - 1, int(rr) + 2):
            for dy in range(-int(rr) - 1, int(rr) + 2):
                ox, oy = map_x + dx, map_y + dy
                if (0 <= ox < self.env.map.shape[0] and 
                    0 <= oy < self.env.map.shape[1] and 
                    self.env.map[ox, oy] == 1):
                    dist = np.hypot(x - ox, y - oy)
                    if dist < min_dist:
                        min_dist = dist
        
        # Calculate repulsive potential
        if min_dist <= rr:
            if min_dist <= 0.1:
                min_dist = 0.1
            return 0.5 * self.eta * ((1.0 / min_dist) - (1.0 / rr)) ** 2
        else:
            return 0.0
    
    def _get_motion_model(self):
        """Get 8-directional motion model."""
        motion = [[1, 0], [0, 1], [-1, 0], [0, -1],
                  [-1, -1], [-1, 1], [1, -1], [1, 1]]
        return motion
    
    def _oscillations_detection(self, previous_ids, ix, iy):
        """Detect oscillations in path."""
        previous_ids.append((ix, iy))
        
        if len(previous_ids) > self.oscillations_detection_length:
            previous_ids.popleft()
        
        # Check for duplicates
        previous_ids_set = set()
        for index in previous_ids:
            if index in previous_ids_set:
                return True
            else:
                previous_ids_set.add(index)
        
        return False
