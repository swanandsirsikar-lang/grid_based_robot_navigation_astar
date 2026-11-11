# Grid-Based Autonomous Robot Navigation Using A*

## Project Overview

This project implements a grid-based autonomous robot
navigation system using the A* path planning algorithm.

The system uses a 100 x 100 occupancy grid and supports
eight-connected movement.

The physical dimensions of the robot are considered
through obstacle inflation.

## Features

- 100 x 100 occupancy grid
- Eight-connected neighbourhood
- A* shortest path planning
- Octile distance heuristic
- Circular robot footprint
- Obstacle inflation
- Diagonal corner-cut prevention
- Input validation
- Map file handling
- Path report generation

## Robot Model

The robot is approximated as a circular footprint.

Default diameter: 3 cells

Safety clearance: 0.5 cells

## Repository Structure

- src: C implementation files
- include: Header files
- data: Input occupancy grid
- output: Generated navigation report
- docs: Project documentation
- tests: Automated tests

## Current Development Stage

The C implementation, modular architecture, Makefile integration and
Google Test suite have been completed.

The application has been built and tested using the provided Makefile
targets. The project includes automated tests for grid operations,
A* pathfinding and input validation.

## Limitations

- Static occupancy grid
- Circular robot approximation
- No real-time sensor integration
- No physical robot control
