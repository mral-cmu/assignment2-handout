import numpy as np

from .grid import Point

class Observer:
    def __init__(self, gt_grid):
        self.grid = gt_grid

    def observe_along_ray(self, ray, max_range):
        success, cells = self.grid.traverse(ray.o, ray.point_at_dist(max_range))

        if success:
            for c in cells:
                found_occ = False
                if self.grid.occupiedQ(c):
                    found_occ = True
                    break

            if found_occ:
                return self.grid.cell_to_point(c) + Point(self.grid.resolution / 2, self.grid.resolution / 2)
            else:
                return ray.point_at_dist(max_range)
        else:
            return None
