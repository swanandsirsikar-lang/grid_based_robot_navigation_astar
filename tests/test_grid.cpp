#include <gtest/gtest.h>

extern "C"
{
    #include "grid.h"
}

/*
 * Test Suite 1:
 * Coordinate validation.
 */

TEST(GridCoordinateTest, ValidCoordinate)
{
    EXPECT_EQ(
        isCoordinateValid(10, 20),
        1
    );
}

TEST(GridCoordinateTest, MinimumBoundaryCoordinate)
{
    EXPECT_EQ(
        isCoordinateValid(0, 0),
        1
    );
}

TEST(GridCoordinateTest, MaximumBoundaryCoordinate)
{
    EXPECT_EQ(
        isCoordinateValid(99, 99),
        1
    );
}

TEST(GridCoordinateTest, NegativeCoordinate)
{
    EXPECT_EQ(
        isCoordinateValid(-1, 20),
        0
    );
}

TEST(GridCoordinateTest, CoordinateOutsideGrid)
{
    EXPECT_EQ(
        isCoordinateValid(100, 50),
        0
    );
}


/*
 * Test Suite 2:
 * Obstacle inflation.
 */

TEST(GridInflationTest, ObstacleCellRemainsBlocked)
{
    Grid_t original_grid;
    Grid_t inflated_grid;

    Robot_t robot = {3.0, 0.5};

    initialiseGrid(&original_grid);

    ASSERT_EQ(
        setCell(&original_grid, 50, 50, CELL_OBSTACLE),
        STATUS_SUCCESS
    );

    ASSERT_EQ(
        inflateObstacles(
            &original_grid,
            &inflated_grid,
            &robot
        ),
        STATUS_SUCCESS
    );

    EXPECT_EQ(
        isCellBlocked(&inflated_grid, 50, 50),
        1
    );
}

TEST(GridInflationTest, NearbyCellBecomesInflated)
{
    Grid_t original_grid;
    Grid_t inflated_grid;

    Robot_t robot = {3.0, 0.5};

    initialiseGrid(&original_grid);

    ASSERT_EQ(
        setCell(&original_grid, 50, 50, CELL_OBSTACLE),
        STATUS_SUCCESS
    );

    ASSERT_EQ(
        inflateObstacles(
            &original_grid,
            &inflated_grid,
            &robot
        ),
        STATUS_SUCCESS
    );

    EXPECT_EQ(
        inflated_grid.cells[50][52],
        CELL_INFLATED
    );
}

TEST(GridInflationTest, DistantCellRemainsFree)
{
    Grid_t original_grid;
    Grid_t inflated_grid;

    Robot_t robot = {3.0, 0.5};

    initialiseGrid(&original_grid);

    ASSERT_EQ(
        setCell(&original_grid, 50, 50, CELL_OBSTACLE),
        STATUS_SUCCESS
    );

    ASSERT_EQ(
        inflateObstacles(
            &original_grid,
            &inflated_grid,
            &robot
        ),
        STATUS_SUCCESS
    );

    EXPECT_EQ(
        inflated_grid.cells[50][55],
        CELL_FREE
    );
}

TEST(GridInflationTest, RobotCannotApproachMapBoundary)
{
    Grid_t original_grid;
    Grid_t inflated_grid;

    Robot_t robot = {3.0, 0.5};

    initialiseGrid(&original_grid);

    ASSERT_EQ(
        inflateObstacles(
            &original_grid,
            &inflated_grid,
            &robot
        ),
        STATUS_SUCCESS
    );

    EXPECT_EQ(
        inflated_grid.cells[50][0],
        CELL_INFLATED
    );

    EXPECT_EQ(
        inflated_grid.cells[50][2],
        CELL_FREE
    );
}

TEST(GridInflationTest, InvalidRobotDimensions)
{
    Grid_t original_grid;
    Grid_t inflated_grid;

    Robot_t robot = {-3.0, 0.5};

    initialiseGrid(&original_grid);

    EXPECT_EQ(
        inflateObstacles(
            &original_grid,
            &inflated_grid,
            &robot
        ),
        STATUS_INVALID_ARGUMENT
    );
}
