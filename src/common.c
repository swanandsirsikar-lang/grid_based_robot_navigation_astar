#include "common.h"

const char *statusToString(Status_t status)
{
    switch (status)
    {
        case STATUS_SUCCESS:
            return "Success";

        case STATUS_INVALID_ARGUMENT:
            return "Invalid argument";

        case STATUS_INVALID_COORDINATE:
            return "Invalid coordinate";

        case STATUS_INVALID_MAP:
            return "Invalid map";

        case STATUS_START_BLOCKED:
            return "Starting cell is blocked";

        case STATUS_GOAL_BLOCKED:
            return "Goal cell is blocked";

        case STATUS_NO_PATH:
            return "No valid path exists";

        case STATUS_FILE_ERROR:
            return "File operation failed";

        case STATUS_MEMORY_ERROR:
            return "Memory allocation or capacity error";

        default:
            return "Unknown error";
    }
}
