#ifndef REPORT_H
#define REPORT_H

#include "astar.h"

void displayGridSummary(const Grid_t *grid);

void displayPathReport(
    const Path_t *path,
    GridPoint_t start,
    GridPoint_t goal,
    const Robot_t *robot
);

#endif
