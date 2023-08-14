import numpy as np
from cprint import cprint

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

from data_structures.grid import Grid2D, Point
from data_structures.sensor import Sensor

from utils import visualize

def plot_rays(ax, pos, rays, max_range):
    for r in rays:
        ep = r.point_at_dist(max_range)
        ax.plot([pos.x, ep.x], [pos.y, ep.y], color='black')

# Scene
scene_fig, scene_ax = plt.subplots()

# Initialize an empty grid along with visualization updates
grid = Grid2D(0.1, 40, 40, 0.001, 0.999)
scene_ax.xaxis.set_major_locator(MultipleLocator(1.0))
scene_ax.yaxis.set_major_locator(MultipleLocator(1.0))
scene_ax.xaxis.set_minor_locator(MultipleLocator(grid.resolution))
scene_ax.yaxis.set_minor_locator(MultipleLocator(grid.resolution))
scene_ax.grid(which='major', axis='both', linestyle='-')
scene_ax.grid(which='minor', axis='both', linestyle='-')

grid_hl = visualize(grid, ax=scene_ax)
scene_ax.set_xlim([0.0, grid.resolution * grid.width])
scene_ax.set_ylim([0.0, grid.resolution * grid.height])
scene_ax.set_xlabel('Cell Space: Cols, Point Space: X (meters)')
scene_ax.set_ylabel('Cell Space: Rows, Point Space: Y (meters)')
scene_ax.set_aspect('equal')

sensor = Sensor()

pos = Point(1.23, 3.2)
rays = sensor.rays(pos)

rays_np = np.zeros((len(rays), 4))
for i, r in enumerate(rays):
    rays_np[i, :] = r.to_numpy()

rays_np_correct = np.load('test_data/sensor_test.npz')['rays_np']
if (np.abs(rays_np_correct - rays_np) < 1e-6).all():
    cprint.info('sensor_test successful.')
else:
    cprint.err('sensor_test failed.', interrupt=False)

plot_rays(scene_ax, pos, rays, sensor.max_range)

plt.show()