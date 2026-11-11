#include <ctype.h>
#include <stdio.h>
#include <stddef.h>

#include "filehandler.h"

/*
 * Load an occupancy grid from a text file.
 *
 * The file must contain exactly 100 x 100 values.
 * 0 = free cell.
 * 1 = obstacle cell.
 *
 * Additional non-whitespace content is rejected.
 */
Status_t loadGridFromFile(
    const char *filename,
    Grid_t *grid
)
{
    FILE *file;

    int x;
    int y;
    int value;
    int character;

    if (filename == NULL || grid == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    file = fopen(filename, "r");

    if (file == NULL)
    {
        return STATUS_FILE_ERROR;
    }

    for (y = 0; y < GRID_ROWS; y++)
    {
        for (x = 0; x < GRID_COLS; x++)
        {
            if (fscanf(file, "%d", &value) != 1 ||
                (value != 0 && value != 1))
            {
                (void)fclose(file);

                return STATUS_INVALID_MAP;
            }

            grid->cells[y][x] =
                (value == 0) ? CELL_FREE : CELL_OBSTACLE;
        }
    }

    /*
     * After reading the expected grid, allow whitespace only.
     */
    while ((character = fgetc(file)) != EOF)
    {
        if (!isspace((unsigned char)character))
        {
            (void)fclose(file);

            return STATUS_INVALID_MAP;
        }
    }

    if (ferror(file))
    {
        (void)fclose(file);

        return STATUS_FILE_ERROR;
    }

    if (fclose(file) != 0)
    {
        return STATUS_FILE_ERROR;
    }

    return STATUS_SUCCESS;
}

/* Save the calculated route and its statistics. */
Status_t savePathToFile(
    const char *filename,
    const Path_t *path,
    GridPoint_t start,
    GridPoint_t goal,
    const Robot_t *robot
)
{
    FILE *file;

    int i;

    if (filename == NULL ||
        path == NULL ||
        robot == NULL ||
        path->length <= 0)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    file = fopen(filename, "w");

    if (file == NULL)
    {
        return STATUS_FILE_ERROR;
    }

    if (fprintf(file, "A* GRID NAVIGATION REPORT\n") < 0 ||
        fprintf(file, "Grid: %d x %d\n",
                GRID_COLS, GRID_ROWS) < 0 ||
        fprintf(file, "Start: (%d, %d)\n",
                start.x, start.y) < 0 ||
        fprintf(file, "Goal: (%d, %d)\n",
                goal.x, goal.y) < 0 ||
        fprintf(file, "Robot diameter: %.2f cells\n",
                robot->diameter_cells) < 0 ||
        fprintf(file, "Safety clearance: %.2f cells\n",
                robot->safety_clearance_cells) < 0 ||
        fprintf(file, "Path nodes: %d\n",
                path->length) < 0 ||
        fprintf(file, "Path cost: %.3f\n",
                path->total_cost) < 0 ||
        fprintf(file, "Expanded nodes: %d\n",
                path->nodes_expanded) < 0 ||
        fprintf(file, "\nPath coordinates:\n") < 0)
    {
        (void)fclose(file);

        return STATUS_FILE_ERROR;
    }

    for (i = 0; i < path->length; i++)
    {
        if (fprintf(
                file,
                "%d,%d\n",
                path->points[i].x,
                path->points[i].y
            ) < 0)
        {
            (void)fclose(file);

            return STATUS_FILE_ERROR;
        }
    }

    if (fclose(file) != 0)
    {
        return STATUS_FILE_ERROR;
    }

    return STATUS_SUCCESS;
}
