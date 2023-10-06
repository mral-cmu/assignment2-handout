import numpy as np
from cprint import cprint

from data_structures.grid import Grid2D, Point
from mapper import Mapper
from data_structures.sensor import Sensor
from data_structures.observer import Observer

from utils import visualize, png_to_grid2d

import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from matplotlib.collections import LineCollection


def test_qualitative(positions, map_name='simple_obstacle'):
    scene_fig, scene_ax = plt.subplots(1, 2, figsize=(16, 8))

    png_map_path = f'test_data/{map_name}.png'
    gt_grid = Grid2D(0.1, 40, 40, 0.001, 0.999)
    gt_grid = png_to_grid2d(gt_grid, png_map_path)

    observer_obj = Observer(gt_grid)

    grid = Grid2D(0.1, 40, 40, 0.001, 0.999)
    sensor_obj = Sensor(max_range=2.0, num_rays=200)

    mapper_obj = Mapper(grid, sensor_obj, observer_obj)

    scene_ax[0].xaxis.set_major_locator(MultipleLocator(1.0))
    scene_ax[0].yaxis.set_major_locator(MultipleLocator(1.0))
    scene_ax[0].xaxis.set_minor_locator(MultipleLocator(gt_grid.resolution))
    scene_ax[0].yaxis.set_minor_locator(MultipleLocator(gt_grid.resolution))
    scene_ax[0].grid(which='major', axis='both', linestyle='-')
    scene_ax[0].grid(which='minor', axis='both', linestyle='-')
    scene_ax[0].set_xlabel('Cell Space: Cols, Point Space: X (meters)')
    scene_ax[0].set_ylabel('Cell Space: Rows, Point Space: Y (meters)')
    scene_ax[0].set_aspect('equal')
    scene_ax[0].set_title('Occupancy Grid Map')

    scene_ax[1].xaxis.set_major_locator(MultipleLocator(1.0))
    scene_ax[1].yaxis.set_major_locator(MultipleLocator(1.0))
    scene_ax[1].xaxis.set_minor_locator(MultipleLocator(gt_grid.resolution))
    scene_ax[1].yaxis.set_minor_locator(MultipleLocator(gt_grid.resolution))
    scene_ax[1].grid(which='major', axis='both', linestyle='-')
    scene_ax[1].grid(which='minor', axis='both', linestyle='-')
    scene_ax[1].set_title('Range sensor getting data in real world')

    visualize(gt_grid, ax=scene_ax[1])
    scene_ax[1].set_xlim([0.0, gt_grid.resolution * gt_grid.width])
    scene_ax[1].set_ylim([0.0, gt_grid.resolution * gt_grid.height])
    scene_ax[1].set_xlabel('Cell Space: Cols, Point Space: X (meters)')
    scene_ax[1].set_ylabel('Cell Space: Rows, Point Space: Y (meters)')
    scene_ax[1].set_aspect('equal')

    rays_vis = []
    rays_collection = LineCollection(rays_vis, color='black', alpha=0.5)
    scene_ax[1].add_collection(rays_collection)
    rob = plt.Circle((0.0, 0.0), grid.resolution / 2, color='r')
    scene_ax[1].add_artist(rob)
    for pos in positions:
        rob.center = pos.x, pos.y

        endpoints = mapper_obj.add_obs(pos)
        segs = []
        colors = []
        for i in range(len(endpoints)):
            segs.append(
                np.array([[pos.x, pos.y], [endpoints[i].x, endpoints[i].y]]))

        rays_collection.set_segments(segs)

        vis_obj = visualize(grid, ax=scene_ax[0])
        scene_ax[0].set_xlim([0.0, grid.resolution * grid.width])
        scene_ax[0].set_ylim([0.0, grid.resolution * grid.height])

        hl = scene_fig.colorbar(vis_obj, ax=scene_ax[0])

        plt.draw()
        plt.pause(0.2)
        hl.remove()


def test_quantitative(positions, map_name='simple_obstacle'):
    png_map_path = f'test_data/{map_name}.png'
    gt_grid = Grid2D(0.1, 40, 40, 0.001, 0.999)
    gt_grid = png_to_grid2d(gt_grid, png_map_path)

    observer_obj = Observer(gt_grid)

    grid = Grid2D(0.1, 40, 40, 0.001, 0.999)
    sensor_obj = Sensor(max_range=2.0, num_rays=200)

    mapper_obj = Mapper(grid, sensor_obj, observer_obj)

    scores_arr = []
    for i, pos in enumerate(positions):
        mapper_obj.add_obs(pos)
        npz_data_file = f'test_data/{map_name}_mapper_test_{str(i)}.npz'
        grid_numpy_correct = np.load(npz_data_file)['grid_numpy']
        grid_numpy = grid.to_numpy()
        corrects = np.abs(grid_numpy_correct - grid_numpy) < 1e-3
        avg = np.sum(corrects) / grid.width / grid.height
        if avg >= 0.95:
            scores_arr.append(1.0)
        else:
            scores_arr.append(avg)
    
    return np.sum(np.array(scores_arr)) / len(positions)


if __name__ == "__main__":
    simple_obs_positions = [
        Point(3.36, 3.42),
        Point(2.61, 3.36),
        Point(1.72, 3.36),
        Point(0.75, 3.36),
        Point(0.27, 3.36),
        Point(0.55, 3.07),
        Point(0.55, 2.55),
        Point(0.55, 2.05),
        Point(0.55, 1.45),
        Point(0.55, 1.03),
        Point(0.55, 0.53),
        Point(1.05, 1.53),
        Point(1.45, 1.53),
        Point(1.97, 1.53),
        Point(2.57, 1.53),
        Point(3.17, 1.53),
        Point(3.45, 1.53),
        Point(3.17, 2.05),
        Point(3.17, 2.55),
        Point(3.17, 3.07),
        Point(3.17, 3.37)
    ]

    cprint.info('Running qualitative test on simple_obstacle map')
    test_qualitative(simple_obs_positions, 'simple_obstacle')
    cprint.info('Running quantitative test on simple_obstacle map')
    score = test_quantitative(simple_obs_positions, 'simple_obstacle')
    cprint.ok('Quantitative test score for simple_obstacle map %f' % (score))

    office_positions = [
        Point(1.72, 3.13),
        Point(1.72, 3.52),
        Point(1.13, 3.52),
        Point(0.58, 3.52),
        Point(0.58, 3.13),
        Point(0.58, 2.65),
        Point(1.22, 2.65),
        Point(1.55, 2.65),
        Point(1.72, 2.65),
        Point(1.76, 2.12),
        Point(1.76, 1.58),
        Point(1.76, 1.58),
        Point(1.05, 1.37),
        Point(0.52, 1.37),
        Point(0.52, 1.01),
        Point(0.52, 0.36),
        Point(1.52, 0.36),
        Point(2.52, 0.36),
        Point(3.52, 0.36),
        Point(3.52, 1.36)
    ]

    cprint.info('Running qualitative test on office map')
    test_qualitative(office_positions, 'office')
    cprint.info('Running quantitative test on office map')
    score = test_quantitative(office_positions, 'office')
    cprint.ok('Quantitative test score for office map %f' % (score))

    plt.show()