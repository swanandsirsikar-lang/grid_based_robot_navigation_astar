#ifndef FILE_HANDLER_H
#define FILE_HANDLER_H

#include "grid.h"
#include "astar.h"

Status_t loadGridFromFile(
    const char *filename,
    Grid_t *grid
);

Status_t savePathToFile(
    const char *filename,
    const Path_t *path,
    GridPoint_t start,
    GridPoint_t goal,
    const Robot_t *robot
);

#endif
