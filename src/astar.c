#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "astar.h"

#define NEIGHBOUR_COUNT 8
#define HEAP_CAPACITY (GRID_CELL_COUNT * 8)

typedef struct
{
    int cell_index;
    double priority;

} HeapNode_t;

typedef struct
{
    HeapNode_t nodes[HEAP_CAPACITY];
    int size;

} MinHeap_t;

/* Eight possible movement directions. */
static const int MOVE_X[NEIGHBOUR_COUNT] =
{
    1, -1, 0, 0, 1, 1, -1, -1
};

static const int MOVE_Y[NEIGHBOUR_COUNT] =
{
    0, 0, 1, -1, 1, -1, 1, -1
};

/* Convert a coordinate into a one-dimensional array index. */
static int pointToIndex(GridPoint_t point)
{
    return point.y * GRID_COLS + point.x;
}

/* Convert a one-dimensional index into grid coordinates. */
static GridPoint_t indexToPoint(int index)
{
    GridPoint_t point;

    point.x = index % GRID_COLS;
    point.y = index / GRID_COLS;

    return point;
}

/* Exchange two heap elements. */
static void swapHeapNodes(
    HeapNode_t *first,
    HeapNode_t *second
)
{
    HeapNode_t temporary = *first;

    *first = *second;
    *second = temporary;
}

/* Insert a node while maintaining min-heap ordering. */
static int heapPush(
    MinHeap_t *heap,
    int cell_index,
    double priority
)
{
    int child;

    if (heap == NULL || heap->size >= HEAP_CAPACITY)
    {
        return 0;
    }

    child = heap->size++;

    heap->nodes[child].cell_index = cell_index;
    heap->nodes[child].priority = priority;

    while (child > 0)
    {
        int parent = (child - 1) / 2;

        if (heap->nodes[parent].priority <=
            heap->nodes[child].priority)
        {
            break;
        }

        swapHeapNodes(
            &heap->nodes[parent],
            &heap->nodes[child]
        );

        child = parent;
    }

    return 1;
}

/* Remove and return the node having the smallest priority. */
static HeapNode_t heapPop(MinHeap_t *heap)
{
    HeapNode_t result = heap->nodes[0];

    int parent = 0;

    heap->size--;

    if (heap->size > 0)
    {
        heap->nodes[0] = heap->nodes[heap->size];

        while (1)
        {
            int left = (2 * parent) + 1;
            int right = left + 1;

            int smallest = parent;

            if (left < heap->size &&
                heap->nodes[left].priority <
                heap->nodes[smallest].priority)
            {
                smallest = left;
            }

            if (right < heap->size &&
                heap->nodes[right].priority <
                heap->nodes[smallest].priority)
            {
                smallest = right;
            }

            if (smallest == parent)
            {
                break;
            }

            swapHeapNodes(
                &heap->nodes[parent],
                &heap->nodes[smallest]
            );

            parent = smallest;
        }
    }

    return result;
}

/*
 * Calculate the octile-distance heuristic.
 *
 * Straight movement cost = 1.
 * Diagonal movement cost = sqrt(2).
 */
double calculateOctileHeuristic(
    GridPoint_t from,
    GridPoint_t to
)
{
    int dx = abs(from.x - to.x);
    int dy = abs(from.y - to.y);

    int minimum = dx < dy ? dx : dy;
    int maximum = dx > dy ? dx : dy;

    return (double)maximum +
           ((sqrt(2.0) - 1.0) * (double)minimum);
}

/*
 * Prevent diagonal corner cutting.
 *
 * Both side-adjacent cells must be free before
 * a diagonal movement is permitted.
 */
static int canMoveDiagonally(
    const Grid_t *grid,
    GridPoint_t current,
    int next_x,
    int next_y
)
{
    int dx = next_x - current.x;
    int dy = next_y - current.y;

    if (dx != 0 && dy != 0)
    {
        if (isCellBlocked(
                grid,
                current.x + dx,
                current.y
            ) ||
            isCellBlocked(
                grid,
                current.x,
                current.y + dy
            ))
        {
            return 0;
        }
    }

    return 1;
}

/* Reconstruct the final path by following parent indexes backwards. */
static Status_t reconstructPath(
    const int came_from[GRID_CELL_COUNT],
    int start_index,
    int goal_index,
    Path_t *path
)
{
    int reverse_path[GRID_CELL_COUNT];

    int count = 0;
    int current = goal_index;
    int i;

    while (current != start_index)
    {
        if (current < 0 || count >= GRID_CELL_COUNT)
        {
            return STATUS_NO_PATH;
        }

        reverse_path[count++] = current;

        current = came_from[current];

        if (current < 0 || current >= GRID_CELL_COUNT)
        {
            return STATUS_NO_PATH;
        }
    }

    reverse_path[count++] = start_index;

    path->length = count;

    for (i = 0; i < count; i++)
    {
        path->points[i] =
            indexToPoint(reverse_path[count - i - 1]);
    }

    return STATUS_SUCCESS;
}

/*
 * Main A* search function.
 *
 * The supplied grid must already contain inflated obstacles.
 */
Status_t findAStarPath(
    const Grid_t *grid,
    GridPoint_t start,
    GridPoint_t goal,
    Path_t *path
)
{
    double g_score[GRID_CELL_COUNT];

    int came_from[GRID_CELL_COUNT];

    unsigned char closed[GRID_CELL_COUNT] = {0};

    MinHeap_t heap;

    int start_index;
    int goal_index;

    int i;

    if (grid == NULL || path == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    if (!isCoordinateValid(start.x, start.y) ||
        !isCoordinateValid(goal.x, goal.y))
    {
        return STATUS_INVALID_COORDINATE;
    }

    path->length = 0;
    path->total_cost = 0.0;
    path->nodes_expanded = 0;

    if (isCellBlocked(grid, start.x, start.y))
    {
        return STATUS_START_BLOCKED;
    }

    if (isCellBlocked(grid, goal.x, goal.y))
    {
        return STATUS_GOAL_BLOCKED;
    }

    start_index = pointToIndex(start);
    goal_index = pointToIndex(goal);

    heap.size = 0;

    /* Initialise all search information. */
    for (i = 0; i < GRID_CELL_COUNT; i++)
    {
        g_score[i] = COST_INFINITY;
        came_from[i] = -1;
    }

    /* Starting node has zero accumulated cost. */
    g_score[start_index] = 0.0;

    if (!heapPush(
            &heap,
            start_index,
            calculateOctileHeuristic(start, goal)
        ))
    {
        return STATUS_MEMORY_ERROR;
    }

    /* Continue until the priority queue becomes empty. */
    while (heap.size > 0)
    {
        HeapNode_t current_node = heapPop(&heap);

        int current_index = current_node.cell_index;

        GridPoint_t current =
            indexToPoint(current_index);

        /* Ignore duplicate entries already processed. */
        if (closed[current_index])
        {
            continue;
        }

        /* Destination reached: reconstruct the route. */
        if (current_index == goal_index)
        {
            Status_t result = reconstructPath(
                came_from,
                start_index,
                goal_index,
                path
            );

            if (result == STATUS_SUCCESS)
            {
                path->total_cost = g_score[goal_index];
            }

            return result;
        }

        closed[current_index] = 1;

        path->nodes_expanded++;

        /* Examine all eight neighbouring cells. */
        for (i = 0; i < NEIGHBOUR_COUNT; i++)
        {
            int next_x = current.x + MOVE_X[i];

            int next_y = current.y + MOVE_Y[i];

            GridPoint_t next;

            int next_index;

            double step_cost;
            double tentative_g;

            /* Check grid boundaries and obstacles. */
            if (!isCoordinateValid(next_x, next_y) ||
                isCellBlocked(grid, next_x, next_y))
            {
                continue;
            }

            /* Reject unsafe diagonal movement. */
            if (!canMoveDiagonally(
                    grid,
                    current,
                    next_x,
                    next_y
                ))
            {
                continue;
            }

            next.x = next_x;
            next.y = next_y;

            next_index = pointToIndex(next);

            if (closed[next_index])
            {
                continue;
            }

            /* Determine the movement cost. */
            if (MOVE_X[i] != 0 && MOVE_Y[i] != 0)
            {
                step_cost = sqrt(2.0);
            }
            else
            {
                step_cost = 1.0;
            }

            tentative_g =
                g_score[current_index] + step_cost;

            /* Update the node if a better route is found. */
            if (tentative_g + FLOAT_EPSILON <
                g_score[next_index])
            {
                came_from[next_index] = current_index;

                g_score[next_index] = tentative_g;

                double heuristic =
                    calculateOctileHeuristic(next, goal);

                if (!heapPush(
                        &heap,
                        next_index,
                        tentative_g + heuristic
                    ))
                {
                    return STATUS_MEMORY_ERROR;
                }
            }
        }
    }

    /* An empty priority queue means no route was found. */
    return STATUS_NO_PATH;
}
