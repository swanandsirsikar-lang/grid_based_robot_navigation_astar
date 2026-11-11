#include <gtest/gtest.h>

extern "C"
{
    #include "validation.h"
}

/*
 * Test Suite:
 * Integer input validation.
 */

TEST(InputValidationTest, AcceptsValidInteger)
{
    int value = 0;

    EXPECT_EQ(
        parseInteger("123", &value),
        STATUS_SUCCESS
    );

    EXPECT_EQ(value, 123);
}

TEST(InputValidationTest, AcceptsNegativeInteger)
{
    int value = 0;

    EXPECT_EQ(
        parseInteger("-25", &value),
        STATUS_SUCCESS
    );

    EXPECT_EQ(value, -25);
}

TEST(InputValidationTest, RejectsAlphabeticInput)
{
    int value = 0;

    EXPECT_EQ(
        parseInteger("abc", &value),
        STATUS_INVALID_ARGUMENT
    );
}

TEST(InputValidationTest, RejectsNullInput)
{
    int value = 0;

    EXPECT_EQ(
        parseInteger(nullptr, &value),
        STATUS_INVALID_ARGUMENT
    );
}
