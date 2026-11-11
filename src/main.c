#include <stdio.h>
#include <stdlib.h>

#include "common.h"
#include "grid.h"
#include "validation.h"
#include "astar.h"
#include "filehandler.h"
#include "report.h"

/*
 * Read and validate a coordinate pair.
 *
 * Expected input format: x y
 */
static Status_t readCoordinate(
    const char *prompt,
    GridPoint_t *point
)
{
    char line[64];

    char *separator;
    char *second_value;

    int x;
    int y;

    if (prompt == NULL || point == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    printf("%s", prompt);

    if (fgets(line, sizeof(line), stdin) == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    separator = line;

    while (*separator != '\0' &&
           *separator != ' ' &&
           *separator != '\t' &&
           *separator != '\n')
    {
        separator++;
    }

    if (*separator == '\0' || *separator == '\n')
    {
        return STATUS_INVALID_ARGUMENT;
    }

    *separator = '\0';

    second_value = separator + 1;

    if (parseInteger(line, &x) != STATUS_SUCCESS ||
        parseInteger(second_value, &y) != STATUS_SUCCESS)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    point->x = x;
    point->y = y;

    return validatePoint(*point);
}
/* Main application entry point. */
int main(void)
{
    Grid_t original_grid;

    Grid_t planning_grid;

    Robot_t robot =
    {
        DEFAULT_ROBOT_DIAMETER,
        DEFAULT_SAFETY_CLEARANCE
    };

    GridPoint_t start;
    GridPoint_t goal;

    Path_t path;

    Status_t status;

    printf("=============================================\n");

    printf(" GRID-BASED AUTONOMOUS ROBOT NAVIGATION\n");

    printf(" A* SHORTEST PATH PLANNING\n");

    printf(" 8-CONNECTED NEIGHBOURHOOD\n");

    printf("=============================================\n");

    /* Initialise the occupancy grid. */
    initialiseGrid(&original_grid);

    /* Load the obstacle map. */
    status = loadGridFromFile(
        DEFAULT_MAP_FILE,
        &original_grid
    );

    if (status != STATUS_SUCCESS)
    {
        fprintf(
            stderr,
            "Map loading failed: %s\n",
            statusToString(status)
        );

        return EXIT_FAILURE;
    }

    displayGridSummary(&original_grid);

    /* Display the assumed robot dimensions. */
    printf("\nRobot model: Circular footprint\n");

    printf("Robot diameter: %.2f cells\n",
           robot.diameter_cells);

    printf("Safety clearance: %.2f cells\n",
           robot.safety_clearance_cells);

    /* Account for the robot's physical dimensions. */
    status = inflateObstacles(
        &original_grid,
        &planning_grid,
        &robot
    );

    if (status != STATUS_SUCCESS)
    {
        fprintf(
            stderr,
            "Obstacle inflation failed: %s\n",
            statusToString(status)
        );

        return EXIT_FAILURE;
    }

    /* Receive the starting coordinate. */
    status = readCoordinate(
        "\nEnter start coordinate (x y): ",
        &start
    );

    if (status != STATUS_SUCCESS)
    {
        fprintf(
            stderr,
            "Invalid starting coordinate.\n"
        );

        return EXIT_FAILURE;
    }

    /* Receive the destination coordinate. */
    status = readCoordinate(
        "Enter goal coordinate (x y): ",
        &goal
    );

    if (status != STATUS_SUCCESS)
    {
        fprintf(
            stderr,
            "Invalid goal coordinate.\n"
        );

        return EXIT_FAILURE;
    }

    /* Validate both endpoints. */
    status = validateEndpoints(&start, &goal);

    if (status != STATUS_SUCCESS)
    {
        fprintf(
            stderr,
            "Endpoint validation failed: %s\n",
            statusToString(status)
        );

        return EXIT_FAILURE;
    }

    /* Execute A* path planning. */
    status = findAStarPath(
        &planning_grid,
        start,
        goal,
        &path
    );

    if (status != STATUS_SUCCESS)
    {
        fprintf(
            stderr,
            "Path planning failed: %s\n",
            statusToString(status)
        );

        return EXIT_FAILURE;
    }

    /* Display the calculated route. */
    displayPathReport(
        &path,
        start,
        goal,
        &robot
    );

    /* Save the route to a text file. */
    status = savePathToFile(
        DEFAULT_PATH_FILE,
        &path,
        start,
        goal,
        &robot
    );

    if (status != STATUS_SUCCESS)
    {
        fprintf(
            stderr,
            "Path saving failed: %s\n",
            statusToString(status)
        );

        return EXIT_FAILURE;
    }

    printf(
        "\nPath successfully saved to %s\n",
        DEFAULT_PATH_FILE
    );

    return EXIT_SUCCESS;
}
