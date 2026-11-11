# Project Proposal

## 1. Project Information

- **Project Title:** Grid-Based Autonomous Robot Navigation and Shortest Path Planning System Using A* Algorithm
- **Domain:** Embedded Systems / Robotics
- **Student Name:** Swanand Sirsikar
- **Roll Number:** 612505157
- **Division / Batch:** Batch 2
- **Repository Name:** grid_based_robot_navigation_astar
- **GitHub Repository:** https://github.com/swanandsirsikar-lang/grid_based_robot_navigation_astar

## 2. Introduction and Problem Statement

A mobile robot needs to find a safe route from its starting position to a destination while avoiding obstacles. Finding a route becomes difficult when the environment contains several obstacles or narrow passages.

The aim of this project is to develop a C-based navigation program that finds a shortest collision-free path on a two-dimensional grid. The project uses the A* search algorithm and considers the physical size of the robot instead of treating it as a single point.

## 3. Proposed Solution

The environment is represented using a 100 × 100 occupancy grid. Each cell represents either free space or an obstacle.

The program first reads the map and validates the input. It then expands obstacles to account for the robot's size and the required safety clearance. After receiving the start and goal coordinates, the A* algorithm searches for a suitable path.

The algorithm supports movement in eight directions. It uses the octile distance heuristic to estimate the remaining distance and prevents diagonal movement through blocked corners. If a valid path is found, the program displays and saves the path report.

## 4. Objectives

- Represent the environment using a two-dimensional occupancy grid.
- Implement the A* pathfinding algorithm in C.
- Support horizontal, vertical and diagonal movement.
- Consider the robot's dimensions and safety clearance during navigation.
- Prevent collisions with obstacles and map boundaries.
- Validate user inputs and map files.
- Test important functions using Google Test.
- Maintain a modular and well-documented software structure.

## 5. Target Users

The project is intended for students and beginners learning robotics, embedded C programming, grid-based navigation and pathfinding algorithms. It can also serve as a basic software model for understanding autonomous robot navigation.

## 6. Scope

### In Scope
- A fixed 100 × 100 occupancy grid.
- Eight-direction movement.
- A* shortest-path search.
- Circular approximation of the robot's footprint.
- Obstacle inflation and safety clearance.
- Input validation and file handling.
- Path reporting and automated unit testing.

### Out of Scope
- Real-time sensor input.
- Physical robot movement and motor control.
- Dynamic obstacle detection and avoidance.
- Live map updates during navigation.

## 7. Assumptions and Constraints

- The map is provided as a text file containing occupancy values.
- The grid dimensions remain fixed at 100 × 100 cells.
- The robot is approximated by a circular footprint.
- The default robot diameter is 3 cells, with a safety clearance of 0.5 cells.
- The environment is static while the path is being planned.
- The program is implemented in C and tested on a system with the required compiler and Google Test libraries.

## 8. Functional Requirements

| ID | Requirement |
|---|---|
| FR-01 | The program shall load a 100 × 100 occupancy grid from a file. |
| FR-02 | The program shall validate the map and user-entered coordinates. |
| FR-03 | The program shall identify obstacles and inflate them according to the robot footprint and safety clearance. |
| FR-04 | The program shall accept valid start and goal coordinates. |
| FR-05 | The program shall find a collision-free path using A*. |
| FR-06 | The program shall support eight-direction movement and prevent diagonal corner cutting. |
| FR-07 | The program shall report when no valid path is available. |
| FR-08 | The program shall display and save the generated path report. |

## 9. Non-Functional Requirements

- **Reliability:** Invalid maps, coordinates and blocked endpoints should be handled safely.
- **Maintainability:** The program should be divided into separate modules and header files.
- **Readability:** Meaningful names, constants and comments should be used.
- **Testability:** Important functions should be covered by automated unit tests.
- **Portability:** The project should avoid machine-specific file paths and use standard C features.

## 10. Expected Outcome

The expected outcome is a working C application that accepts an occupancy grid, start and goal coordinates, and generates a shortest collision-free path when one exists. The project also provides automated tests, a Makefile for building and testing, and documentation explaining the implementation.

The completed project is a software-based navigation model. It does not directly control a physical robot.
