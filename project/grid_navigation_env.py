import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

# 8 directional movement
_DIRECTION_MOVE = {
    0: (-1, 0),      # up
    1: (1, 0),       # down
    2: (0, -1),      # left
    3: (0, 1),       # right
    4: (-1, -1),     # up-left
    5: (-1, 1),      # up-right
    6: (1, -1),      # down-left
    7: (1, 1),       # down-right
}


class GridNavigationEnv:
    """
    Sets up the 2D grid-based navigation environment.
    - Start position
    - Goal position
    - Max Steps per episode
    - Creates the grid map with obstacles
    - Calculates where the agent would move and learn based on reward function
    """
    def __init__(self, map_file, start, goal, max_steps=500):
        self.grid = np.loadtxt(map_file)
        self.h, self.w = self.grid.shape
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.max_steps = max_steps
        self._fig = None
        self._ax = None
        self._frames = []
        self.reset()

    def state_size(self):
        return 11  # 11 possible states to help the agent determine where to move next

    def action_size(self):
        return 8  # 8 posssible movement directions

    def reset(self, random_goal=False):
        """Reset the environment for the next episode."""
        self.pos = (int(self.start[0]), int(self.start[1]))
        self.steps = 0
        
        # Pick a random valid goal if applied
        if random_goal:
            self.goal = self._sample_random_goal()
        
        self.prev_dist = self._distance(self.pos, self.goal)
        self.path = [self.pos]
        self._frames = []
        return self._get_state()
    
    def _sample_random_goal(self):
        """Returns a random goal coordinate that is not the same as the start."""
        possible_goals = np.argwhere(self.grid == 0) # get all non-obstacle cells
        if len(possible_goals) == 0:
            return self.goal  # use default goal stated if no other cell is available
        
        while True:
            num = np.random.randint(len(possible_goals))
            coord = tuple(possible_goals[num])
            if coord != self.start:
                return coord

    def _in_bounds(self, pos):
        x, y = pos
        return 0 <= x < self.h and 0 <= y < self.w

    def _is_obstacle(self, pos):
        x, y = pos
        return self.grid[x, y] == 1

    def _distance(self, a, b):
        return float(np.linalg.norm(np.array(a) - np.array(b)))

    def _blocked(self, pos, delta):
        nxt = (pos[0] + delta[0], pos[1] + delta[1])
        return (not self._in_bounds(nxt)) or self._is_obstacle(nxt)

    def _get_state(self):
        x, y = self.pos
        gx, gy = self.goal

        # Check all 8 directions for obstacles
        up = float(self._blocked(self.pos, _DIRECTION_MOVE[0])) 
        down = float(self._blocked(self.pos, _DIRECTION_MOVE[1]))
        left = float(self._blocked(self.pos, _DIRECTION_MOVE[2]))
        right = float(self._blocked(self.pos, _DIRECTION_MOVE[3]))
        up_left = float(self._blocked(self.pos, _DIRECTION_MOVE[4]))
        up_right = float(self._blocked(self.pos, _DIRECTION_MOVE[5]))
        down_left = float(self._blocked(self.pos, _DIRECTION_MOVE[6]))
        down_right = float(self._blocked(self.pos, _DIRECTION_MOVE[7]))

        dx_norm = (gx-x)/max(1, self.h-1)
        dy_norm = (gy-y)/max(1, self.w-1)
        dist_norm = self._distance(self.pos, self.goal)/max(1.0, np.hypot(self.h - 1, self.w - 1))

        state = np.array([
            up,
            down,
            left,
            right,
            up_left,
            up_right,
            down_left,
            down_right,
            dx_norm,
            dy_norm,
            dist_norm,
        ], dtype=np.float32)
        return state

    def step(self, action):
        if action not in _DIRECTION_MOVE:
            raise ValueError("Invalid action index")

        delta = _DIRECTION_MOVE[action]
        candidate = (self.pos[0] + delta[0], self.pos[1] + delta[1])

        self.steps += 1
        done = False

        # Obstacle collision penalty
        if (not self._in_bounds(candidate)) or self._is_obstacle(candidate):
            reward = -5.0
            done = True
            return self._get_state(), reward, done, {"reason": "collision"}

        # Move agent
        self.pos = candidate
        dist = self._distance(self.pos, self.goal)
        progress = self.prev_dist - dist
        self.prev_dist = dist
        self.path.append(self.pos)
        # Capture frame for GIF
        if self._fig is not None and self._ax is not None:
            self._capture_frame()

        # Reward Function Calculation
        reward = 0.0

        # Penalize each step to encourage efficiency
        reward -= 0.1

        # Goal navigation: reward for reducing distance
        reward += 1.0 * progress

        # Penalize distance increase (overshoot or moving away from goal)
        if progress < 0:
            reward -= 1.0 + 0.5 * abs(progress)  # strong overshoot penalty

        # Penalize revisiting cells (avoiding loops)
        visits_here = self.path.count(self.pos)
        if visits_here > 1:
            reward -= 0.6 * (visits_here - 1)  # increasing cost per repeat

        # If agent backtracks, penalize
        if len(self.path) >= 3 and self.pos == self.path[-3]:
            reward -= 2.0

        # If agent does not get closer to goal, penalize
        if progress == 0:
            reward -= 0.3

        # When agent is very close to goal
        if self.pos != self.goal and dist < 1.5:
            reward += 0.5  # encourage agent to go into goal when adjacent/diagonal

        # Penalty for being near obstacles
        near_obstacle = any(
            self._in_bounds((self.pos[0] + d[0], self.pos[1] + d[1])) and self._is_obstacle((self.pos[0] + d[0], self.pos[1] + d[1]))
            for d in _DIRECTION_MOVE.values()
        )
        if near_obstacle:
            reward -= 0.2 

        # Heading alignment toward goal
        goal_vec = np.array(self.goal) - np.array(self.pos)
        if np.linalg.norm(goal_vec) > 1e-6:
            goal_dir = goal_vec / np.linalg.norm(goal_vec)
            move_dir = np.array(delta) / (np.linalg.norm(delta) + 1e-8)
            align = float(np.dot(goal_dir, move_dir))  # in [-1, 1]
            reward += 0.3 * align  # guide toward goal

        # Reward for reaching goal with efficiency bonus
        if self.pos == self.goal:
            # Base goal reward
            reward += 10.0 
            # Give a bonus if goal was reached in fewer steps
            efficiency_bonus = max(0, (self.max_steps - self.steps) / self.max_steps) * 10.0
            reward += efficiency_bonus
            done = True
            return self._get_state(), reward, done, {"reason": "goal"}

        if self.steps >= self.max_steps:
            done = True
            reward -= 0.5
            return self._get_state(), reward, done, {"reason": "timeout"}

        return self._get_state(), reward, done, {}

    def render(self, pause: float = 0.05):
        """Visualization of the current episode."""
        if self._fig is None or self._ax is None:
            self._fig, self._ax = plt.subplots(figsize=(6, 6))

        self._ax.clear()
        self._ax.imshow(1 - self.grid, cmap="gray", origin="upper")

        # draw path so far
        if len(self.path) > 1:
            ys = [p[1] for p in self.path]
            xs = [p[0] for p in self.path]
            self._ax.plot(ys, xs, color="cyan", linewidth=2, label="path")

        # start, goal, current agent
        self._ax.scatter(self.start[1], self.start[0], c="blue", marker="s", s=80, label="start")
        self._ax.scatter(self.goal[1], self.goal[0], c="green", marker="*", s=120, label="goal")
        self._ax.scatter(self.pos[1], self.pos[0], c="red", marker="o", s=80, label="agent")

        self._ax.set_xticks([])
        self._ax.set_yticks([])
        self._ax.set_title(f"Step {self.steps} | Pos {self.pos} → Goal {self.goal}")
        self._ax.legend(loc="upper right", fontsize=8)
        plt.pause(pause)

        self._capture_frame()

    def _capture_frame(self):
        # Capture frame for later GIF saving
        if self._fig is None:
            return
        self._fig.canvas.draw()
        buf = np.asarray(self._fig.canvas.buffer_rgba())
        frame = buf[..., :3].copy()  # drop alpha
        self._frames.append(frame)

    def save_gif(self, path: str, fps: int = 5):
        """Save captured frames as a GIF. Call after an episode (e.g., after evaluation)."""
        if not self._frames:
            print("No frames captured.")
            return

        fig = plt.figure(figsize=(6, 6))
        ax = plt.axes()
        plt.axis('off')

        im = plt.imshow(self._frames[0])

        def _animate(i):
            im.set_array(self._frames[i])
            return [im]

        anim = animation.FuncAnimation(fig, _animate, frames=len(self._frames), interval=1000 / fps)
        anim.save(path, writer='pillow', fps=fps)
        plt.close(fig)
        print(f"Saved GIF to {path}")
