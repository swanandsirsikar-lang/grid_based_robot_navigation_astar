#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <ctype.h>

#include "validation.h"
#include "grid.h"

/* Validate one grid coordinate. */
Status_t validatePoint(GridPoint_t point)
{
    if (!isCoordinateValid(point.x, point.y))
    {
        return STATUS_INVALID_COORDINATE;
    }

    return STATUS_SUCCESS;
}

/* Validate both starting and destination coordinates. */
Status_t validateEndpoints(
    const GridPoint_t *start,
    const GridPoint_t *goal
)
{
    if (start == NULL || goal == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    if (validatePoint(*start) != STATUS_SUCCESS ||
        validatePoint(*goal) != STATUS_SUCCESS)
    {
        return STATUS_INVALID_COORDINATE;
    }

    return STATUS_SUCCESS;
}

/*
 * Convert a text input into an integer.
 * Reject empty input, invalid characters and overflow.
 */
Status_t parseInteger(
    const char *text,
    int *value
)
{
    char *end_pointer = NULL;
    long parsed_value;

    if (text == NULL || value == NULL)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    while (isspace((unsigned char)*text))
    {
        text++;
    }

    if (*text == '\0')
    {
        return STATUS_INVALID_ARGUMENT;
    }

    errno = 0;

    parsed_value = strtol(text, &end_pointer, 10);

    if (errno == ERANGE ||
        parsed_value < INT_MIN ||
        parsed_value > INT_MAX ||
        end_pointer == text)
    {
        return STATUS_INVALID_ARGUMENT;
    }

    while (isspace((unsigned char)*end_pointer))
    {
        end_pointer++;
    }

    if (*end_pointer != '\0')
    {
        return STATUS_INVALID_ARGUMENT;
    }

    *value = (int)parsed_value;

    return STATUS_SUCCESS;
}
