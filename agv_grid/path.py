import numpy as np
from collections import deque


class Grassfire:

    def __init__(self, grid):
        """
        grid: numpy 2D array
              0 = free
              1 = obstacle
        """
        self.grid = grid
        self.rows = grid.shape[0]
        self.cols = grid.shape[1]

    def in_bounds(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def neighbors(self, r, c):
        # 4-connected grid
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if self.in_bounds(nr, nc):
                yield nr, nc

    def compute(self, start, goal):

        sr, sc = start
        gr, gc = goal

        if self.grid[gr][gc] == 1:
            return None  # goal blocked

        # Distance map initialized to -1
        dist = -np.ones_like(self.grid, dtype=int)

        queue = deque()
        queue.append((sr, sc))
        dist[sr][sc] = 0

        # --- Wave Expansion ---
        while queue:
            r, c = queue.popleft()

            if (r, c) == (gr, gc):
                break

            for nr, nc in self.neighbors(r, c):
                if self.grid[nr][nc] == 0 and dist[nr][nc] == -1:
                    dist[nr][nc] = dist[r][c] + 1
                    queue.append((nr, nc))

        if dist[gr][gc] == -1:
            return None  # unreachable

        # --- Backtrack Path ---
        path = []
        r, c = gr, gc
        path.append((r, c))

        while (r, c) != (sr, sc):
            for nr, nc in self.neighbors(r, c):
                if dist[nr][nc] == dist[r][c] - 1:
                    r, c = nr, nc
                    path.append((r, c))
                    break

        path.reverse()
        return path

'''
class MPC:
    def __init__(self, grid):
'''
        
