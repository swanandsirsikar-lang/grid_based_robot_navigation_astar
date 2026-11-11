#ifndef ASTAR_H
#define ASTAR_H

#include "grid.h"

typedef struct
{
    GridPoint_t points[GRID_CELL_COUNT];

    int length;

    double total_cost;

    int nodes_expanded;

} Path_t;

Status_t findAStarPath(
    const Grid_t *grid,
    GridPoint_t start,
    GridPoint_t goal,
    Path_t *path
);

double calculateOctileHeuristic(
    GridPoint_t from,
    GridPoint_t to
);

#endif
