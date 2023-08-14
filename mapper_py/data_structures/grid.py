"""Cell, Point, and Grid classes for 16-362: Mobile Robot Algorithms Laboratory
"""

import numpy as np
from copy import copy


class Cell:
    """A single cell in the occupancy grid map.

    Attributes:
        row: Row number of the cell. Corresponds to Y-axis in 2D plane.
        col: Col number of the cell. Corresponds to X-axis in 2D plane.
    """

    def __init__(self, row=0, col=0):
        """Initializes the row and col for this cell to be 0."""
        self.row = row
        self.col = col

    def __str__(self):
        return f'Cell(row: {self.row}, col: {self.col})'

    def to_numpy(self):
        """Return a numpy array with the cell row and col."""
        return np.array([self.row, self.col], dtype=int)


class Point:
    """A point in the 2D space.

    Attributes:
        x: A floating point value for the x coordinate of the 2D point.
        y: A floating point value for the y coordinate of the 2D point.
    """

    def __init__(self, x=0.0, y=0.0):
        """Initializes the x and y for this point to be 0.0"""
        self.x = x
        self.y = y

    def __abs__(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5

    def __str__(self):
        return f'Point(x: {self.x}, y: {self.y})'

    def __eq__(self, second):
        if isinstance(second, Point):
            return ((self.x == second.x) and (self.y == second.y))
        else:
            raise TypeError('Argument type must be Point.')

    def __ne__(self, second):
        if isinstance(second, Point):
            return ((self.x != second.x) or (self.y != second.y))
        else:
            raise TypeError('Argument type must be Point.')

    def __add__(self, second):
        if isinstance(second, Point):
            return Point(self.x + second.x, self.y + second.y)
        elif isinstance(second, float):
            return Point(self.x + second, self.y + second)
        else:
            raise TypeError('Argument type must be either float or Point.')

    def __sub__(self, second):
        if isinstance(second, Point):
            return Point(self.x - second.x, self.y - second.y)
        elif isinstance(second, float):
            # when subtracting with a float, always post-subtract
            # YES: Point(1.2, 3.2) - 5.0
            # NO: 5.0 - Point(1.2, 3.2)
            return Point(self.x - second, self.y - second)
        else:
            raise TypeError('Argument type must be either float or Point.')

    def __mul__(self, second):
        if isinstance(second, Point):
            return (self.x * second.x + self.y * second.y)
        elif isinstance(second, float):
            # when multiplying with a float, always post-multiply
            # YES: Point(1.2, 3.2) * 5.0
            # NO: 5.0 * Point(1.2, 3.2)
            return Point(self.x * second, self.y * second)
        else:
            raise TypeError('Argument type must be either float or Point.')

    def __truediv__(self, second):
        if isinstance(second, float):
            # when dividing by a float, always post-divide
            # YES: Point(1.2, 3.2) / 5.0
            # NO: 5.0 / Point(1.2, 3.2)
            if np.abs(second - 0.0) < 1e-12:
                raise ValueError(
                    'Divide by zero error. Second argument is too close to zero.')
            else:
                return Point(self.x / second, self.y / second)
        else:
            raise TypeError('Argument type must be float.')

    def to_numpy(self):
        """Return a numpy array with the x and y coordinates."""
        return np.array([self.x, self.y], dtype=float)


class Grid2D:
    """Occupancy grid data structure.

    Attributes:
        resolution: (float) The size of each cell in meters.
        width: (int) Maximum number of columns in the grid.
        height: (int) Maximum number of rows in the grid.
        min_clamp: (float) Logodds corresponding to minimum possible probability
        (to ensure numerical stability).
        max_clamp: (float) Logodds corresponding to maximum possible probability
        (to ensure numerical stability).
        free_thres: (float) Logodds below which a cell is considered free
        occ_thres: (float) Logodds above which a cell is considered occupied
        N: (int) Total number of cells in the grid
        data: Linear array of containing the logodds of this occupancy grid
    """

    def __init__(self, res, W, H, min_clamp, max_clamp, free_thres=0.13, occ_thres=0.7):
        """Initialize the grid data structure.

        Note that min_clamp, max_clamp, free_thres, and occ_thres inputs to this constructor
        are probabilities. You have to convert them to logodds internally for numerical stability.
        """
        self.resolution = res

        self.width = int(W)
        self.height = int(H)

        self.min_clamp = self.logodds(min_clamp)
        self.max_clamp = self.logodds(max_clamp)
        self.free_thres = self.logodds(free_thres)
        self.occ_thres = self.logodds(occ_thres)

        self.N = self.width * self.height

        # Initially all the logodds values are zero.
        # A logodds value of zero corresponds to an occupancy probability of 0.5.
        self.data = [0.0] * self.N

    def to_numpy(self):
        """Export the grid in the form of a 2D numpy matrix.

        Each entry in this matrix is the probability of occupancy for the cell.
        """
        g = np.zeros((self.height, self.width))
        for row in range(self.height):
            for col in range(self.width):
                v = self.get_row_col(row, col)
                g[row][col] = self.probability(v)

        return g

    def to_index(self, cell):
        """Return the index into the data array (self.data) for the input cell.

        Args:
            cell: (Cell) The input cell for which the index in data array is requested.

        Returns:
            idx: (int) Index in the data array for the cell
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        raise NotImplementedError

    def from_index(self, idx):
        """Return the cell in grid for the input index.

        Args:
            idx: (int) Index in the data array for which the cell is requested.

        Returns:
            cell: (Cell) Cell corresponding to the index.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        raise NotImplementedError

    def get(self, idx):
        """Return the cell value corresponding to the input index.

        Args:
            idx: (int) Index in the data array for which the data is requested.

        Returns:
            val: (float) Value in the data array for idx
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        raise NotImplementedError

    def get_cell(self, cell):
        """Return the cell value corresponding to the input index."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `to_index` and `get` methods.
        raise NotImplementedError

    def get_row_col(self, row, col):
        """Return the cell value corresponding to the row and col."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `get_cell` method and the `Cell` constructor.
        raise NotImplementedError

    def set(self, idx, value):
        """Set the cell to value corresponding to the input idx."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        raise NotImplementedError

    def set_cell(self, cell, value):
        """Set the cell to value corresponding to the input cell."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use `to_index` and `set` methods.
        raise NotImplementedError

    def set_row_col(self, row, col, value):
        """Set the cell to value corresponding to the input row and col."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `set_cell` method and the `Cell` constructor.
        raise NotImplementedError

    def probability(self, logodds):
        """Convert input logodds to probability.

        Args:
            logodds: (float) Logodds representation of occupancy.

        Returns:
            prob: (float) Probability representation of occupancy.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        raise NotImplementedError

    def logodds(self, probability):
        """Convert input probability to logodds.

        Args:
            logodds: (float) Logodds representation of occupancy.

        Returns:
            prob: (float) Probability representation of occupancy.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        raise NotImplementedError

    def cell_to_point(self, cell):
        """Get the cell's lower-left corner in 2D point space.

        Args:
            cell: (Cell) Input cell.

        Returns:
            point: (Point) Lower-left corner in 2D space.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        raise NotImplementedError

    def cell_to_point_row_col(self, row, col):
        """Get the point for the lower-left corner of the cell represented by input row and col."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `cell_to_point` function and the `Cell` constructor.
        raise NotImplementedError

    def point_to_cell(self, point):
        """Get the cell position (i.e., bottom left hand corner) given the point.

        Args:
            point: (Point) Query point

        Returns:
            cell: (Cell) Cell in the grid corresponding to the query point.
        """
        # TODO: Assignment 2, Problem 1.1 (test_traversal)
        raise NotImplementedError

    def inQ(self, cell):
        """Is the cell inside this grid? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.1 (test_traversal)
        raise NotImplementedError

    def traverse(self, start, end):
        """Figure out the cells that the ray from start to end traverses.

        Corner cases that must be accounted for:
        - If start and end points coincide, return (False, None).
        - Check that the start point is inside the grid. Return (False, None) otherwise.
        - End point can be outside the grid. The ray tracing must stop at the edges of the grid.
        - Perfectly horizontal and vertical rays.
        - Ray starts and ends in the same cell.

        Args:
            start: (Point) Start point of the ray
            end: (Point) End point of the ray

        Returns:
            success, raycells: (bool, list of Cell) If the traversal was successful, success is True
                                and raycells is the list of traversed cells (including the starting
                                cell). Otherwise, success is False and raycells is None.
        """
        # TODO: Assignment 2, Problem 1.1 (test_traversal)
        raise NotImplementedError

    def freeQ(self, cell):
        """Is the cell free? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.3
        # Hint: Use `get_cell` and `free_thres`
        raise NotImplementedError

    def occupiedQ(self, cell):
        """Is the cell occupied? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.3
        # Hint: Use `get_cell` and `occ_thres`
        raise NotImplementedError

    def unknownQ(self, cell):
        """Is the cell unknown? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.3
        # Hint: Use `get_cell`, `occ_thres`, and `free_thres`
        raise NotImplementedError
