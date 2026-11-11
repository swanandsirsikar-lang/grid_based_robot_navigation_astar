#ifndef COMMON_H
#define COMMON_H

#define GRID_ROWS 100
#define GRID_COLS 100

#define GRID_CELL_COUNT (GRID_ROWS * GRID_COLS)

#define DEFAULT_ROBOT_DIAMETER 3.0
#define DEFAULT_SAFETY_CLEARANCE 0.5

#define DEFAULT_MAP_FILE "data/sample_map.txt"
#define DEFAULT_PATH_FILE "output/planned_path.txt"

#define COST_INFINITY 1.0e30
#define FLOAT_EPSILON 1.0e-9

typedef enum
{
    STATUS_SUCCESS = 0,
    STATUS_INVALID_ARGUMENT,
    STATUS_INVALID_COORDINATE,
    STATUS_INVALID_MAP,
    STATUS_START_BLOCKED,
    STATUS_GOAL_BLOCKED,
    STATUS_NO_PATH,
    STATUS_FILE_ERROR,
    STATUS_MEMORY_ERROR

} Status_t;

typedef enum
{
    CELL_FREE = 0,
    CELL_OBSTACLE = 1,
    CELL_INFLATED = 2

} CellType_t;

typedef struct
{
    int x;
    int y;

} GridPoint_t;

const char *statusToString(Status_t status);

#endif
