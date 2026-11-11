#include <stdio.h>
#include <stddef.h>

#include "report.h"

/* Display the number of free and obstacle cells. */
void displayGridSummary(const Grid_t *grid)
{
    int x;
    int y;

    int obstacles = 0;

    if (grid == NULL)
    {
        printf("Invalid grid.\n");
        return;
    }

    for (y = 0; y < GRID_ROWS; y++)
    {
        for (x = 0; x < GRID_COLS; x++)
        {
            if (grid->cells[y][x] == CELL_OBSTACLE)
            {
                obstacles++;
            }
        }
    }

    printf("\nGrid dimensions : %d x %d\n",
           GRID_COLS, GRID_ROWS);

    printf("Total cells     : %d\n",
           GRID_CELL_COUNT);

    printf("Obstacle cells  : %d\n",
           obstacles);

    printf("Free cells      : %d\n",
           GRID_CELL_COUNT - obstacles);
}

/* Display the calculated path and its performance statistics. */
void displayPathReport(
    const Path_t *path,
    GridPoint_t start,
    GridPoint_t goal,
    const Robot_t *robot
)
{
    int i;

    if (path == NULL ||
        robot == NULL ||
        path->length <= 0)
    {
        printf("No valid path report available.\n");
        return;
    }

    printf("\n========== A* PATH PLANNING REPORT ==========\n");

    printf("Start coordinate : (%d, %d)\n",
           start.x, start.y);

    printf("Goal coordinate  : (%d, %d)\n",
           goal.x, goal.y);

    printf("Robot diameter   : %.2f cells\n",
           robot->diameter_cells);

    printf("Safety clearance : %.2f cells\n",
           robot->safety_clearance_cells);

    printf("Path nodes       : %d\n",
           path->length);

    printf("Path cost        : %.3f cell units\n",
           path->total_cost);

    printf("Nodes expanded   : %d\n",
           path->nodes_expanded);

    printf("\nPath coordinates:\n");

    for (i = 0; i < path->length; i++)
    {
        printf("(%d, %d)", 
               path->points[i].x,
               path->points[i].y);

        if (i < path->length - 1)
        {
            printf(" -> ");
        }

        if ((i + 1) % 6 == 0 &&
            i < path->length - 1)
        {
            printf("\n");
        }
    }

    printf("\n=============================================\n");
}
