import argparse
from cprint import cprint
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

from data_structures.grid import Grid2D, Point
from utils import png_to_grid2d, visualize

def test_data_structure(map_name, grid_visible=True):
    # Path to the png file corresponding to the environment
    # Black regions are occupied, white regions are free
    # In this test, a grid map should be created for this map
    png_map_path = f'test_data/{map_name}.png'

    # Grid map at resolution 0.1 of size 60 cells x 80 cells
    # Minimum probability (i.e. highest confidence about free space) is 0.001
    # Maximum probability (i.e. highest confidence about occupied space) is 0.999
    grid = Grid2D(0.1, 60, 80, 0.001, 0.999)

    # Update the grid using the png image
    grid = png_to_grid2d(grid, png_map_path)

    # Get a numpy array corresponding to the grid
    grid_numpy = grid.to_numpy()

    # Load the correct answer
    b = np.load(f'test_data/{map_name}.npz')
    grid_numpy_correct = b['grid_numpy']

    # Check if all the values are close
    # If you get "test_data_structure failed", check your grid2d implementation
    if (np.abs(grid_numpy_correct - grid_numpy) < 1e-6).all():
        cprint.info('test_data_structure successful.')
    else:
        cprint.err('test_data_structure failed.', interrupt=False)

    # Visualize for extra clarity
    grid_fig, grid_ax = plt.subplots()
    pos = visualize(grid, ax=grid_ax)

    # The colorbar indicates occupancy probabilities in range [0.0, 1.0]
    grid_fig.colorbar(pos, ax=grid_ax)

    # Create grid lines -- you can disable them if you like for clarity
    if grid_visible:
        grid_ax.xaxis.set_major_locator(MultipleLocator(1.0))
        grid_ax.yaxis.set_major_locator(MultipleLocator(1.0))
        grid_ax.xaxis.set_minor_locator(MultipleLocator(grid.resolution))
        grid_ax.yaxis.set_minor_locator(MultipleLocator(grid.resolution))
        grid_ax.grid(which='major', axis='both', linestyle='-')
        grid_ax.grid(which='minor', axis='both', linestyle='-')

    # Labels for clarity on the cell space and point space
    grid_ax.set_xlabel('Cell Space: Cols, Point Space: X (meters)')
    grid_ax.set_ylabel('Cell Space: Rows, Point Space: Y (meters)')

    # Only display the relevant part of the grid
    grid_ax.set_xlim([0.0, grid.resolution * grid.width])
    grid_ax.set_ylim([0.0, grid.resolution * grid.height])

    # Show the grid
    grid_ax.set_aspect('equal')
    plt.show()

def test_traversal(grid_ax, start=Point(1.2, 1.2), end=Point(2.2, 1.5), test_file='traced_cells_1',
                   c='navy', grid_visible=True):
    # Initialize an empty grid
    grid = Grid2D(0.1, 40, 40, 0.001, 0.999)

    # Helper function to highlight a cell
    def highlight_cell(x, y, res, ax=None, **kwargs):
        rect = plt.Rectangle((x, y), res, res, fill=True, **kwargs)
        ax = ax or plt.gca()
        ax.add_patch(rect)
        return rect

    # Helper function to plot the traced cells on the grid
    def plot_test_output(traced_cells, ax=None):
        for t in traced_cells:
            p = grid.cell_to_point(t)
            highlight_cell(p.x, p.y, grid.resolution, ax=ax,
                           color=c, alpha=0.2, linewidth=0)

    # Visualize the empty grid
    visualize(grid, ax=grid_ax)

    # Plot the ray from start to end
    grid_ax.plot([start.x, end.x], [start.y, end.y], color=c)

    # Get the traced cells
    success, traced_cells = grid.traverse(start, end)

    # First, the traverse function should succeed
    # If your code fails this check, check your traverse function implementation
    if success:
        cprint.info(f"traverse function succeeded, number of traced cells: {len(traced_cells)}")
    else:
        cprint.err("traverse function failed.", interrupt=True)

    # Then, check if the traced cells are correct
    # If your code fails this check, check your traverse function implementation
    traced_cells_np = np.array([a.to_numpy() for a in traced_cells])
    traced_cells_np_correct = np.load(f'test_data/{test_file}.npz')['traced_cells']

    if (np.abs(traced_cells_np_correct - traced_cells_np) < 1e-6).all():
        cprint.info('test_traversal successful.')
    else:
        cprint.err('test_traversal failed.', interrupt=False)

    plot_test_output(traced_cells, ax=grid_ax)

    grid_ax.xaxis.set_major_locator(MultipleLocator(1.0))
    grid_ax.yaxis.set_major_locator(MultipleLocator(1.0))
    grid_ax.xaxis.set_minor_locator(MultipleLocator(grid.resolution))
    grid_ax.yaxis.set_minor_locator(MultipleLocator(grid.resolution))
    if grid_visible:
        grid_ax.grid(which='major', axis='both', linestyle='-')
        grid_ax.grid(which='minor', axis='both', linestyle='-')
    grid_ax.set_xlabel('Cell Space: Cols, Point Space: X (meters)')
    grid_ax.set_ylabel('Cell Space: Rows, Point Space: Y (meters)')
    grid_ax.set_aspect('equal')
    grid_ax.set_xlim([0.0, grid.resolution * grid.width])
    grid_ax.set_ylim([0.0, grid.resolution * grid.height])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='simple_obstacle')

    args = parser.parse_args()

    test_data_structure(args.map)

    trav_fig, trav_ax = plt.subplots()

    # Test slopped rays
    test_traversal(trav_ax, test_file='traced_cells_01')
    test_traversal(trav_ax, start=Point(3.52, 2.57), end=Point(1.56, 3.92), test_file='traced_cells_02')

    # Test edge rays (horizontal and vertical)
    test_traversal(trav_ax, start=Point(0.5, 2.9), end=Point(2.0, 2.9), c='maroon', test_file='traced_cells_03')
    test_traversal(trav_ax, start=Point(3.8, 0.5), end=Point(3.8, 2.0), c='maroon', test_file='traced_cells_04')

    # Test finer rays (horizontal and vertical)
    test_traversal(trav_ax, start=Point(0.52, 0.85), end=Point(3.15, 0.85), c='darkgreen', test_file='traced_cells_05')
    test_traversal(trav_ax, start=Point(0.15, 0.52), end=Point(0.15, 3.15), c='darkgreen', test_file='traced_cells_06')

    # Test rays going outside of the map bounds
    test_traversal(trav_ax, start=Point(3.32, 0.52), end=Point(7.15, -3.15), c='purple', test_file='traced_cells_07')
    test_traversal(trav_ax, start=Point(2.12, 0.12), end=Point(-7.15, 1.15), c='purple', test_file='traced_cells_08')
    test_traversal(trav_ax, start=Point(3.72, 3.53), end=Point(7.15, 9.18), c='purple', test_file='traced_cells_09')
    test_traversal(trav_ax, start=Point(1.72, 3.53), end=Point(-7.15, 6.18), c='purple', test_file='traced_cells_10')

    plt.show()