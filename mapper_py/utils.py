"""Mapping visualization utility functions for 16-362: Mobile Robot Algorithms Laboratory

Author(s): Kshitij Goel, Wennie Tabib
"""

from os.path import exists
import numpy as np
import matplotlib.pyplot as plt


def png_to_grid2d(grid, image_filepath):
    """Update the grid cell values using a PNG image.

    Args:
        grid : (Grid2D) 2D grid that will be changed during this function
        image_filepath : (str) Path to the image file used to generate cell values

    Returns:
        grid : (Grid2D) The updated grid object
    """

    # OpenCV is only required for this function
    import cv2

    # Fail if the image path does not exist
    if not exists(image_filepath):
        raise Exception('Bad filepath')

    # read the image to convert
    map_image = cv2.imread(image_filepath)

    # convert to grayscale image and flip the y-axis to place the origin at bottom left
    map_image = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)[::-1, :]

    # Resize image to match the grid dimensions
    map_image = cv2.resize(
        map_image, (grid.width, grid.height), interpolation=cv2.INTER_NEAREST)

    # iterate through grid and update the probability according to:
    # occupied (max clamp, p = g.max_clamp) if the pixel value is less than 50
    # free (min clamp, p = g.min_clamp) if the pixel value is greater than 250
    # unknown (p = 0.5) otherwise
    for row in range(grid.height):
        for col in range(grid.width):
            if map_image[row, col] < 50:
                l = grid.probability(grid.max_clamp)
            elif map_image[row, col] > 250:
                l = grid.probability(grid.min_clamp)
            else:
                # Unknown space is assumed at a probability of 0.5
                l = 0.5

            grid.set_row_col(row, col, grid.logodds(l))

    return grid


def visualize(grid, val_fn=lambda x, v: x.probability(v), ax=None):
    """Visualize the grid on canvas.

    Args:
        grid : (Grid2D) 2D grid to visualize
        ax : (optional) Axis object if you want to plot on an existing figure

    Returns:
        plot : Plotted matplotlib object
    """

    # convert grid to image and display
    g = np.zeros((grid.height, grid.width))
    for row in range(grid.height):
        for col in range(grid.width):
            v = grid.get_row_col(row, col)
            g[row][col] = val_fn(grid, v)

    min = grid.cell_to_point_row_col(0, 0)
    max = grid.cell_to_point_row_col(grid.height, grid.width)

    if ax is None:
        return plt.imshow(g, cmap='Greys', origin='lower', extent=(min.x, max.x, min.y, max.y))
    else:
        return ax.imshow(g, cmap='Greys', origin='lower', extent=(min.x, max.x, min.y, max.y))
