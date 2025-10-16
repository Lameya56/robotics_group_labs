import numpy as np
import time

class BugPlanner:
    def __init__(self, planning_env, anystep=1, lookahead=1):
        self.env = planning_env
        self.anystep = anystep
        self.lookahead = lookahead
        self.visited = np.zeros(planning_env.map.shape)

    def _get_next_wall_move(self, prev_pos, current_pos):
        # Helper function to find the next move when following a wall
        x,y = int(current_pos[0]), int(current_pos[1])
        # get prev directional movement and try to move in that direction first
        prev_x, prev_y = int(prev_pos[0]), int(prev_pos[1])
        dx = x - prev_x
        dy = y - prev_y
        directions = [
            (0,1*self.anystep),                   # up
            (-1*self.anystep,1*self.anystep),     # upper-left
            (-1*self.anystep,0),                  # left
            (-1*self.anystep,-1*self.anystep),    # lower-left
            (0,-1*self.anystep),                  # down
            (1*self.anystep,-1*self.anystep),     # lower-right
            (1*self.anystep,0),                   # right
            (1*self.anystep,1*self.anystep)       # upper-right
        ]
        # get prev movement direction, and reorder directions so that it prioritizes that direction first
        try:
            heading_index = directions.index((dx, dy))
        except ValueError:
            heading_index = 0
        ordered_dirs = directions[heading_index:] + directions[:heading_index]

        for nx, ny in ordered_dirs:
            next_x, next_y = x + nx*self.anystep, y + ny*self.anystep

            # go to next direction if new position was already visited or it is a obstacle, invalid, etc.
            if self.visited[next_x, next_y] == 1 or not self.env.state_validity_checker(np.array([[next_x],[next_y]])):
                continue

            # adjacent gets the 8 possible coords around the position
            adjacent = [
                (next_x-1, next_y), (next_x+1, next_y),
                (next_x, next_y-1), (next_x, next_y+1),
                (next_x-1, next_y-1), (next_x-1, next_y+1),
                (next_x+1, next_y-1), (next_x+1, next_y+1)
            ]

            # checks if any adjacent cell is an obstacle, meaning the position is circum-navigating a wall
            # also checks that adjacent cells are within the bounds of the map
            if any(
                0 <= cx < self.env.map.shape[0] and
                0 <= cy < self.env.map.shape[1] and
                self.env.map[cx, cy] == 1
                for cx, cy in adjacent
            ):
                return (next_x, next_y)

        # there was no valid move, return the current position
        return (x, y)

    def _bresenham(self, start, end):
        ''' M-line setup using Bresenham's algorithm
            referenced ChatGPT to get algorithm w/ error slope checking '''
        x1, y1 = int(round(start[0,0])), int(round(start[1,0]))
        x2, y2 = int(round(end[0,0])), int(round(end[1,0]))

        m_line = []

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        sx = 1 if x2 >= x1 else -1
        sy = 1 if y2 >= y1 else -1

        err = dx - dy
        x, y = x1, y1

        while True:
            m_line.append((x, y))
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return m_line

    def Plan(self, start, goal):
        """
        Students should implement the bug 2 algorithm
        @param start: start configuration
        @param goal: goal configuration
        @return: a path of configurations
        """
        plan_time = time.time()
        path = []
        cost = 0
        state_count = 0

        m_line = self._bresenham(start, goal)
        m_line_dupe = m_line.copy() # used to validate hit_point and leave_point
        print("m_line: ", m_line)
        while m_line:
            x,y = m_line[0]
            x = int(x)
            y = int(y)
            # if location is an obstacle or invalid location
            if not self.env.state_validity_checker(np.array([[x],[y]])):
                # follow the wall until
                hit_point = path[-1] # point right before wall is hit
                wall_following = True
                while wall_following:
                    if len(path) < 2:
                        prev_pos = path[-1] # prev_pos is the same as current position
                    else:
                        prev_pos = path[-2]
                    nextx, nexty = self._get_next_wall_move(prev_pos, path[-1])
                    state_count += 1
                        #print("Next wall move: ", (nextx, nexty))
                        #print("Path taken: ", path)
                        #print("m_line: ", m_line)
                    if (nextx, nexty) != path[-1] and (nextx, nexty) in m_line:
                        leave_point = m_line.index((nextx, nexty))
                        # m_line is reached and leave_point closer than hit_point
                        if m_line_dupe.index(hit_point) < m_line_dupe.index((nextx, nexty)):
                            path.append((nextx,nexty))
                            self.visited[nextx,nexty] = 1
                            del m_line[:leave_point+1]
                            wall_following = False
                    elif (nextx, nexty) != path[-1]: # m_line not reached, wall_following continues
                        path.append((nextx,nexty))
                        self.visited[nextx,nexty] = 1
                    else:
                        print("Did not reach goal.")
                        cost = self.calculateCost(path)
                        plan_time = time.time() - plan_time
                        print("States Expanded: %d" % state_count)
                        print("Cost: %f" % cost)
                        print("Planning Time: %ss" % plan_time)
                        return np.array(path).T
                
            else:
                # travel along the m-line
                path.append(m_line[0])
                m_line.pop(0)
                state_count += 1

        print("Reached goal!")
        print("Path taken: ", path)

        cost = self.calculateCost(path)
        plan_time = time.time() - plan_time
        print("States Expanded: %d" % state_count)
        print("Cost: %f" % cost)
        print("Planning Time: %ss" % plan_time)
        
        return np.array(path).T

    def calculateCost(self, path):
        cost = 0
        for index in range(len(path) - 1):
            start_config = np.array([[path[index][0]], [path[index][1]]])
            end_config = np.array([[path[index+1][0]], [path[index+1][1]]])
            cost += self.env.compute_distance(start_config, end_config)
        return cost