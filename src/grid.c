#include <math.h>
#include <stddef.h>

#include "grid.h"

/* Initialise the entire grid as free space. */
void initialiseGrid(Grid_t *grid)
{
    int x;
    int y;

    if (grid == NULL)
    {
        return;
    }

    for (y = 0; y < GRID_ROWS; y++)
    {
        for (x = 0; x < GRID_COLS; x++)
        {
            grid->cells[y][x] = CELL_FREE;
        }
    }
}

/* Check whether coordinates belong to the grid. */
int isCoordinateValid(int x, int y)
{
    return (
        x >= 0 &&
        x < GRID_COLS &&
        y >= 0 &&
        y < GRID_ROWS
    );
}

/* Set the state of a particular cell. */
Status_t setCell(
    Grid_t *grid,
    int x,
    int y,
    CellType_t type
)
{
    if (grid == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    if (!isCoordinateValid(x, y))
    {
        return STATUS_INVALID_COORDINATE;
    }

    if (type != CELL_FREE && type != CELL_OBSTACLE)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    grid->cells[y][x] = type;

    return STATUS_SUCCESS;
}

/* Check whether a cell is blocked. */
int isCellBlocked(
    const Grid_t *grid,
    int x,
    int y
)
{
    if (grid == NULL || !isCoordinateValid(x, y))
    {
        return 1;
    }

    return grid->cells[y][x] != CELL_FREE;
}

/* Validate the robot dimensions. */
Status_t validateRobot(const Robot_t *robot)
{
    if (robot == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    if (!isfinite(robot->diameter_cells) ||
        !isfinite(robot->safety_clearance_cells))
    {
        return STATUS_INVALID_ARGUMENT;
    }

    if (robot->diameter_cells <= 0.0 ||
        robot->safety_clearance_cells < 0.0)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    return STATUS_SUCCESS;
}

/*
 * Inflate obstacles according to the robot's circular footprint.
 *
 * Effective radius = robot radius + safety clearance.
 *
 * The source grid is preserved; inflation is written into
 * a separate destination grid.
 */
Status_t inflateObstacles(
    const Grid_t *source,
    Grid_t *inflated,
    const Robot_t *robot
)
{
    int x;
    int y;
    int obstacle_x;
    int obstacle_y;

    double radius;
    double radius_squared;

    if (source == NULL ||
        inflated == NULL ||
        source == inflated)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    if (validateRobot(robot) != STATUS_SUCCESS)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    *inflated = *source;

    radius =
        (robot->diameter_cells / 2.0) +
        robot->safety_clearance_cells;

    radius_squared = radius * radius;

    for (obstacle_y = 0;
         obstacle_y < GRID_ROWS;
         obstacle_y++)
    {
        for (obstacle_x = 0;
             obstacle_x < GRID_COLS;
             obstacle_x++)
        {
            if (source->cells[obstacle_y][obstacle_x]
                != CELL_OBSTACLE)
            {
                continue;
            }

            for (y = 0; y < GRID_ROWS; y++)
            {
                for (x = 0; x < GRID_COLS; x++)
                {
                    double dx = (double)(x - obstacle_x);
                    double dy = (double)(y - obstacle_y);

                    double distance_squared =
                        (dx * dx) + (dy * dy);

                    if (distance_squared <= radius_squared)
                    {
                        inflated->cells[y][x] = CELL_INFLATED;
                    }
                }
            }
        }
    }

    return STATUS_SUCCESS;
}
