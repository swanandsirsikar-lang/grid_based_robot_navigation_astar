#ifndef GRID_H
#define GRID_H

#include "common.h"

typedef struct
{
    CellType_t cells[GRID_ROWS][GRID_COLS];

} Grid_t;

typedef struct
{
    double diameter_cells;
    double safety_clearance_cells;

} Robot_t;

void initialiseGrid(Grid_t *grid);

Status_t setCell(
    Grid_t *grid,
    int x,
    int y,
    CellType_t type
);

int isCoordinateValid(int x, int y);

int isCellBlocked(
    const Grid_t *grid,
    int x,
    int y
);

Status_t inflateObstacles(
    const Grid_t *source,
    Grid_t *inflated,
    const Robot_t *robot
);

Status_t validateRobot(const Robot_t *robot);

#endif
