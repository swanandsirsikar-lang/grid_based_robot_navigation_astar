# Detailed Software Design

## 1. Project Overview

This project implements grid-based robot navigation using the A* pathfinding algorithm in C. The environment is represented by a 100 × 100 occupancy grid. The robot is modelled as a circular footprint, and obstacles are inflated to maintain the required clearance.

The software is divided into modules so that each part has a specific responsibility and can be developed and tested independently.

## 2. System Architecture

The application follows this sequence:

1. Initialise the grid.
2. Load the occupancy map from the input file.
3. Validate the map data.
4. Validate the robot dimensions and safety clearance.
5. Inflate obstacles and account for map boundaries.
6. Accept the start and goal coordinates.
7. Validate the selected coordinates.
8. Run the A* pathfinding algorithm.
9. Validate the generated path.
10. Display the result and save the path report.

### Architecture Diagram

```text
              User
               |
               v
          main.c
               |
       +-------+-------+
       |               |
       v               v
  File Handler     Validation
       |               |
       v               |
    Grid Module <------+
       |
       v
 Obstacle Inflation
       |
       v
  A* Pathfinding
       |
       v
  Path Validation
       |
       v
   Report Module
       |
       v
  Display / Output File
```

## 3. Module Description

| Module | Files | Responsibility |
|---|---|---|
| Main | `main.c` | Controls the overall program flow and calls the required modules. |
| Common | `common.c`, `common.h` | Defines shared constants, enumerations, structures and common status values. |
| Grid | `grid.c`, `grid.h` | Initialises the grid, checks cells and handles obstacle inflation. |
| Validation | `validation.c`, `validation.h` | Validates coordinates and user input. |
| A* | `astar.c`, `astar.h` | Performs A* search, calculates heuristic costs and reconstructs the path. |
| File Handler | `file_handler.c`, `file_handler.h` | Reads the occupancy map and saves the generated path. |
| Report | `report.c`, `report.h` | Displays the grid summary and path information. |

## 4. Data Design

### 4.1 Grid

The grid is represented using a two-dimensional array of 100 rows and 100 columns.

Each cell has one of the following types:

- `CELL_FREE`: The cell is available for movement.
- `CELL_OBSTACLE`: The cell contains an obstacle.
- `CELL_INFLATED`: The cell is blocked because of obstacle inflation.

### 4.2 Robot

The robot structure stores:

- `diameter_cells`: Diameter of the circular robot footprint, in grid cells.
- `safety_clearance_cells`: Additional clearance maintained around the robot.

The default diameter is 3 cells and the default safety clearance is 0.5 cells.

### 4.3 Path

The path structure stores:

- The sequence of grid coordinates.
- The total number of points in the path.
- The total path cost.
- The number of nodes expanded during the search.

### 4.4 Important Constants

| Constant | Purpose |
|---|---|
| `GRID_ROWS` | Number of grid rows (100). |
| `GRID_COLS` | Number of grid columns (100). |
| `GRID_CELL_COUNT` | Total number of grid cells (10,000). |
| `DEFAULT_ROBOT_DIAMETER` | Default robot diameter (3 cells). |
| `DEFAULT_SAFETY_CLEARANCE` | Default safety clearance (0.5 cells). |
| `COST_INFINITY` | Represents an initially unreachable path cost. |

## 5. Algorithm Design

### 5.1 Obstacle Inflation

The robot cannot be treated as a point because it occupies physical space. Therefore, obstacles are expanded by considering the robot's radius and safety clearance.

The effective clearance radius is:

\[
R = \frac{D}{2} + S
\]

Where:

- \(D\) is the robot diameter.
- \(S\) is the safety clearance.

Cells that fall within the required clearance of an obstacle are marked as inflated. The map boundaries are also considered to prevent the robot from extending outside the grid.

### 5.2 A* Pathfinding

A* selects the next cell based on:

\[
f(n) = g(n) + h(n)
\]

Where:

- \(g(n)\) is the cost from the start cell to the current cell.
- \(h(n)\) is the estimated cost from the current cell to the goal.
- \(f(n)\) is the estimated total path cost.

The algorithm uses eight-direction movement:

- Horizontal and vertical movement cost: 1.
- Diagonal movement cost: \(\sqrt{2}\).

The octile distance heuristic is used:

\[
h(n) = \max(dx,dy) + (\sqrt{2}-1)\min(dx,dy)
\]

Here, \(dx\) and \(dy\) are the absolute differences between the current cell and the goal cell coordinates.

The algorithm maintains an open priority queue to select the cell with the lowest estimated total cost. A closed set keeps track of cells that have already been processed. Parent information is stored to reconstruct the final path.

Diagonal movement is not allowed if either of the two adjacent side cells is blocked. This prevents the robot from cutting through obstacle corners.

## 6. Important Function Design

| Function | Purpose |
|---|---|
| `initialiseGrid()` | Initialises all grid cells. |
| `setCell()` | Sets the type of a particular grid cell. |
| `isCoordinateValid()` | Checks whether coordinates are within the grid. |
| `isCellBlocked()` | Checks whether a cell is blocked. |
| `inflateObstacles()` | Expands obstacles according to the robot dimensions and clearance. |
| `validateRobot()` | Checks whether the robot parameters are valid. |
| `validatePoint()` | Validates a grid coordinate. |
| `validateEndpoints()` | Checks the start and goal positions. |
| `parseInteger()` | Converts and validates integer input. |
| `loadGridFromFile()` | Reads the occupancy grid from a file. |
| `calculateOctileHeuristic()` | Calculates the heuristic distance used by A*. |
| `findAStarPath()` | Searches for a path between the start and goal. |
| `savePathToFile()` | Saves the generated path to an output file. |
| `displayPathReport()` | Displays information about the generated path. |

## 7. Input Validation

The program checks the following before performing path planning:

- The map file must be available and contain valid occupancy values.
- The grid dimensions and map data must be valid.
- Robot dimensions and safety clearance must be valid.
- Start and goal coordinates must lie within the grid.
- The start and goal cells must not be blocked.
- Invalid or incomplete user input must be rejected.

## 8. File Handling

The occupancy map is read from:

`data/sample_map.txt`

The map contains 100 × 100 values representing free and occupied cells.

The generated path report is saved in the output directory:

`output/planned_path.txt`

File operations are checked so that missing files, invalid data and write failures can be reported.

## 9. Error Handling

The program uses status values to identify different outcomes, such as:

- Successful operation.
- Invalid arguments or coordinates.
- Invalid map data.
- Blocked start or goal.
- No path found.
- File operation failure.

Functions return appropriate status values so that errors can be handled by the calling module instead of causing unexpected program termination.

## 10. Unit Testing Plan

Google Test is used to test the C functions through C++ test files.

The tests cover:

- Coordinate validation, including valid and invalid boundary values.
- Obstacle inflation and robot parameter validation.
- A* heuristic calculations.
- Pathfinding in open and blocked environments.
- Invalid start and goal positions.
- Cases where no path exists.
- Prevention of diagonal corner cutting.
- Integer input validation.

The test suite contains 24 test cases across the grid, A* and validation modules. The tests can be executed using:

```bash
make test
```

## 11. Build and Execution

The Makefile provides the following targets:

| Command | Purpose |
|---|---|
| `make all` | Compiles and builds the application. |
| `make run` | Runs the navigation application. |
| `make test` | Builds and executes the unit tests. |
| `make clean` | Removes generated build files and outputs. |

## 12. Limitations and Future Improvements

The current implementation uses a static occupancy grid and a circular approximation of the robot. It does not use live sensor data or control a physical robot.

Future improvements could include dynamic obstacle updates, real-time sensor integration, path smoothing, grid visualisation and integration with a physical mobile robot.
