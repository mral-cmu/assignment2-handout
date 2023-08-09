# 16-362 Assignment 2: Mapping and Localization

Goal: In this assignment, you will implement data structures and algorithms
for occupancy grid maps, range sensors, and filtering-based localization.
These methods are widely deployed in the robotics industry.

There are two subdirectories in this handout: `mapper_py` and `state_est_py`.
The `mapper_py` directory will contain the occupancy grid map implementation that
you will create through this assignment. Likewise, `state_est_py` will contain
all python scripts that enable filtering-based localization.

### Setup
Create a python virtual environment.
```bash
python3.8 -m .venv venv
```
Source the environment
```bash
source .venv/bin/activate
```
You will need to install the following dependencies.
```bash
pip install numpy matplotlib opencv-python
```

## 1. Occupancy Grid Mapping (60 points)
The occupancy grid map is a spatial grid where each cell denotes the occupancy
state for all the points within that cell. In the first part of this assignment,
our goal is to implement an occupancy grid mapping system. To simplify the problem,
we make the following assumptions:
- The robot is perfectly localized on a 2D plane
- The robot size is negligible ("point" robot) and it can move in any direction on a 2D plane

To create a 2D map of the environment, we first need to create a data structure
to store and query occupancy values.

### 1.1 Grid Data Structure (30 points)
In this section, we will write the class `Grid2D` that implements a data
structure for such a grid for 2D environments.

Open the Python script `mapper_py/data_structures/grid.py`. You will see several class definitions.

1. `Cell`: A single cell in the occupancy grid map.
2. `Point`: A point in the 2D space.
3. `Grid2D`: Occupancy grid data structure.

All of the graded parts of this part are within the `Grid2D` class. These
parts are marked by `TODO` and are necessary to complete for full credit.

1. `to_index`: Get the index in the `Grid2D` object for the input `cell` object.
2. `from_index`: Get the `Cell` corresponding to an index `idx`.
3. `get`, `get_cell`, `get_row_col`: Get the occupancy data at the input index `idx` or cells.
4. `set`, `set_cell`, `set_row_col`: Set the occupancy data at the input index `idx` (or cells) to `value`.
5. `probability`: Convert logodds represention to probability.
6. `logodds`: Convert probability representation to logodds.
7. `cell_to_point`, `cell_to_point_row_col`: Get the lower left hand corner of the input `cell` in the 2D point space.
8. `point_to_cell`: Get the `Cell` corresponding to the input 2D point.
9. `inQ`: Check if the input 2D point is in the grid bounds or not.
10. `traverse`: Given a line segment (start and end points), return a tuple (`success`, `raycells`)
where `success` is a bool indicating whether the traversal through the grid was successful and `raycells`
is a list of cells that were traversed if successful.

**Debugging and Grading**
We have provided a testing script `mapper_py/grid_test.py`. It contains two
functions that we will use for autograding: `test_data_structure` and
`test_traversal`.

`test_data_structure` tests your solutions for `1` through `7` above. It reads one of the example
maps from the `test_data` subdirectory and creates a `Grid2D` object from it using the `png_to_grid2d`
method in `mapper_py/utils.py`. The grid is then visualized using the `visualize` method from
`mapper_py/utils.py`. `png_to_grid2d` and `visualize` internally call the methods `1` to `7` mentioned above.
Example output for:

```bash
python3 grid_test.py --map simple_obstacle
```
should look like:

![](example-output-1_1.png)

In the terminal, you should see:
```txt
test_data_structure successful.
```

Additionally, there is a `office` map available for testing. In the autograder, we will
run several unseen maps to test the accuracy of your solution. Passing `test_data_structure`
is worth 10 points.

`test_traversal` tests your solutions for `8` through `10`. It creates an empty grid
of pre-specified parameters and then plots the traced cells for a given ray. In the
main function of `mapper_py/grid_test.py`, you will see several tests for this function
that test corner conditions such as: slopped rays, rays going outside of the map bounds, etc.
Example output (second figure) after running `python3 grid_test.py --map simple_obstacle`:

![](example-output-1_2.png)

In the terminal, you should see:
```txt
traverse function succeeded, number of traced cells: 16
test_traversal successful.
traverse function succeeded, number of traced cells: 35
test_traversal successful.
traverse function succeeded, number of traced cells: 16
test_traversal successful.
traverse function succeeded, number of traced cells: 16
test_traversal successful.
traverse function succeeded, number of traced cells: 27
test_traversal successful.
traverse function succeeded, number of traced cells: 27
test_traversal successful.
traverse function succeeded, number of traced cells: 11
test_traversal successful.
traverse function succeeded, number of traced cells: 24
test_traversal successful.
traverse function succeeded, number of traced cells: 7
test_traversal successful.
traverse function succeeded, number of traced cells: 21
test_traversal successful.
```

There are several different tests with changed start and end points that will be used within
the autograder to test the accuracy of your solution. Passing `test_traversal` is worth
20 points.

### 1.2 Range Sensor (10 points)
We will now create a simple geometric model of a range sensor. We will assume that the range sensor
is mounted on a point (i.e., negligible size) robot.

Look at the file `mapper_py/data_structures/sensor.py`. There are two classes defined, `Ray` and `Sensor`.

The `Ray` class implements, as expected, a ray with an origin and a direction. You have to fill the function
`point_at_dist` that returns a `Point` along the ray for the input distance `t`.

The `Sensor` class implements a geometric model for the range sensor with a maximum range `max_range` and
total number of rays `num_rays`. You have to implement the `rays` method which generates `num_rays` number
of rays around the given position `pos` at equal angular intervals within `[0, 2.0 * np.pi]` (i.e., 0 to 360 degrees).
Do not double count `0.0` and `2.0 * np.pi` (Hint: Utilize the `endpoint` option in the `np.linspace` function.)

**Debugging and Grading**
We have provided a testing script `mapper_py/sensor_test.py`. Running this script will plot
all the rays at the position `Point(1.23, 3.2)` on an empty grid.

The output of
```bash
python sensor_test.py
```
should look like

![](example-output-1_3.png)

and
```text
sensor_test successful.
```
should print out in the terminal.

### 1.3 Occupancy Mapping (20 points)
Now we create a class called `Mapper` in `mapper_py/mapper.py` to put these elements together to
perform occupancy grid mapping via logodds update.

You have to implement the methods `update_logodds`, `update_miss`, `update_hit`, and `add_ray`
in this part. The instructions are provided in the docstrings for each of the functions.

**Debugging and Grading**
We have provided a testing script `mapper_py/mapper_test.py`. Running this script will show
an animation of the occupancy grid map being created using ray observations of the environment.
There is also a quantitative test included.

After running
```bash
python mapper_test.py
```
the final figure for `simple_obstacle` map will look like

![](example-output-1_4.png)

the final figure for `office` map will look like

![](example-output-1_5.png)

and the terminal printouts will be

```txt
Running qualitative test on simple_obstacle map
Running quantitative test on simple_obstacle map
Quantitative test successful for simple_obstacle map.
Running qualitative test on office map
Running quantitative test on office map
Quantitative test successful for office map.
```

## 2. Filtering-based Localization (40 points)

## Grading with AutoLab
TODO

```
tar -cvf <todo> handin.tar
```

Autolab will run tests on each function you implement and you will
receive a score out of 100. You may upload as many times as you like.
Note that we may regrade submissions after the deadline passes.

## References