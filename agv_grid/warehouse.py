import random
import numpy as np

class Warehouse:
    def __init__(self, rows, columns, shelves):

        self.rows = rows
        self.columns = columns
        self.shelves = shelves


    def random_map(self):
        total_elements = self.rows*self.columns
        total_nonzero = self.shelves

        if total_nonzero > total_elements:
            raise ValueError('Too many shelves.')

        random_map = np.zeros((self.rows, self.columns), dtype=int)

        indices = np.random.choice(total_elements, self.shelves, replace=False)
        random_map.flat[indices] = 1

        return random_map


    def preset_map(self):
        preset_map = []
        return preset_map
