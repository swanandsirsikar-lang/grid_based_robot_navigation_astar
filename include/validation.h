#ifndef VALIDATION_H
#define VALIDATION_H

#include "common.h"

Status_t validatePoint(GridPoint_t point);

Status_t validateEndpoints(
    const GridPoint_t *start,
    const GridPoint_t *goal
);

Status_t parseInteger(
    const char *text,
    int *value
);

#endif
