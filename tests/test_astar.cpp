#include <gtest/gtest.h>
#include <cmath>

extern "C"
{
    #include "astar.h"
}

/*
 * Test Suite 1:
 * Octile distance heuristic.
 */

TEST(AStarHeuristicTest, DiagonalDistance)
{
    GridPoint_t start = {0, 0};
    GridPoint_t goal = {3, 3};

    double expected = 3.0 * std::sqrt(2.0);

    EXPECT_NEAR(
        calculateOctileHeuristic(start, goal),
        expected,
        1.0e-6
    );
}

TEST(AStarHeuristicTest, StraightDistance)
{
    GridPoint_t start = {0, 0};
    GridPoint_t goal = {5, 0};

    EXPECT_NEAR(
        calculateOctileHeuristic(start, goal),
        5.0,
        1.0e-6
    );
}

TEST(AStarHeuristicTest, MixedDistance)
{
    GridPoint_t start = {0, 0};
    GridPoint_t goal = {5, 3};

    double expected =
        5.0 + ((std::sqrt(2.0) - 1.0) * 3.0);

    EXPECT_NEAR(
        calculateOctileHeuristic(start, goal),
        expected,
        1.0e-6
    );
}


/*
 * Test Suite 2:
 * A* pathfinding.
 */

TEST(AStarPathTest, FindsPathInOpenGrid)
{
    Grid_t grid;
    Path_t path;

    GridPoint_t start = {10, 10};
    GridPoint_t goal = {13, 13};

    initialiseGrid(&grid);

    ASSERT_EQ(
        findAStarPath(
            &grid,
            start,
            goal,
            &path
        ),
        STATUS_SUCCESS
    );

    EXPECT_EQ(path.length, 4);

    EXPECT_NEAR(
        path.total_cost,
        3.0 * std::sqrt(2.0),
        1.0e-6
    );
}

TEST(AStarPathTest, FindsStraightPath)
{
    Grid_t grid;
    Path_t path;

    GridPoint_t start = {10, 10};
    GridPoint_t goal = {15, 10};

    initialiseGrid(&grid);

    ASSERT_EQ(
        findAStarPath(
            &grid,
            start,
            goal,
            &path
        ),
        STATUS_SUCCESS
    );

    EXPECT_EQ(path.length, 6);

    EXPECT_NEAR(
        path.total_cost,
        5.0,
        1.0e-6
    );
}

TEST(AStarPathTest, RejectsBlockedStartingCell)
{
    Grid_t grid;
    Path_t path;

    GridPoint_t start = {10, 10};
    GridPoint_t goal = {20, 20};

    initialiseGrid(&grid);

    ASSERT_EQ(
        setCell(&grid, 10, 10, CELL_OBSTACLE),
        STATUS_SUCCESS
    );

    EXPECT_EQ(
        findAStarPath(
            &grid,
            start,
            goal,
            &path
        ),
        STATUS_START_BLOCKED
    );
}

TEST(AStarPathTest, RejectsBlockedGoalCell)
{
    Grid_t grid;
    Path_t path;

    GridPoint_t start = {10, 10};
    GridPoint_t goal = {20, 20};

    initialiseGrid(&grid);

    ASSERT_EQ(
        setCell(&grid, 20, 20, CELL_OBSTACLE),
        STATUS_SUCCESS
    );

    EXPECT_EQ(
        findAStarPath(
            &grid,
            start,
            goal,
            &path
        ),
        STATUS_GOAL_BLOCKED
    );
}

TEST(AStarPathTest, ReturnsNoPathWhenGoalIsSeparated)
{
    Grid_t grid;
    Path_t path;

    GridPoint_t start = {10, 10};
    GridPoint_t goal = {20, 10};

    int y;

    initialiseGrid(&grid);

    /*
     * Create a complete vertical obstacle wall.
     */
    for (y = 0; y < GRID_ROWS; y++)
    {
        ASSERT_EQ(
            setCell(&grid, 15, y, CELL_OBSTACLE),
            STATUS_SUCCESS
        );
    }

    EXPECT_EQ(
        findAStarPath(
            &grid,
            start,
            goal,
            &path
        ),
        STATUS_NO_PATH
    );
}

TEST(AStarPathTest, RejectsInvalidCoordinates)
{
    Grid_t grid;
    Path_t path;

    GridPoint_t start = {-1, 10};
    GridPoint_t goal = {20, 20};

    initialiseGrid(&grid);

    EXPECT_EQ(
        findAStarPath(
            &grid,
            start,
            goal,
            &path
        ),
        STATUS_INVALID_COORDINATE
    );
}
