# ==========================================================
# Grid-Based Autonomous Robot Navigation Using A*
# Makefile
# ==========================================================

# ----------------------------------------------------------
# Compiler configuration
# ----------------------------------------------------------

CC = gcc
CXX = g++

# ----------------------------------------------------------
# Compiler flags
# ----------------------------------------------------------

CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -Werror
CXXFLAGS = -std=c++17 -Wall -Wextra -Wpedantic -Werror

CPPFLAGS = -Iinclude

# ----------------------------------------------------------
# Linker configuration
# ----------------------------------------------------------

LDLIBS = -lm

GTEST_LIBS = -lgtest -lgtest_main -pthread

# ----------------------------------------------------------
# Directories
# ----------------------------------------------------------

SOURCE_DIR = src
HEADER_DIR = include
TEST_DIR = tests

BUILD_DIR = build
OBJECT_DIR = $(BUILD_DIR)/obj
TEST_OBJECT_DIR = $(BUILD_DIR)/test

# ----------------------------------------------------------
# Application configuration
# ----------------------------------------------------------

APP_NAME = grid_astar

APP_SOURCES = \
    $(SOURCE_DIR)/main.c \
    $(SOURCE_DIR)/common.c \
    $(SOURCE_DIR)/grid.c \
    $(SOURCE_DIR)/validation.c \
    $(SOURCE_DIR)/astar.c \
    $(SOURCE_DIR)/filehandler.c \
    $(SOURCE_DIR)/report.c

APP_OBJECTS = $(patsubst \
    $(SOURCE_DIR)/%.c, \
    $(OBJECT_DIR)/%.o, \
    $(APP_SOURCES))

# ----------------------------------------------------------
# Google Test configuration
# ----------------------------------------------------------

TEST_SOURCES = \
    $(TEST_DIR)/test_grid.cpp \
    $(TEST_DIR)/test_astar.cpp \
    $(TEST_DIR)/test_validation.cpp

TEST_OBJECTS = $(patsubst \
    $(TEST_DIR)/%.cpp, \
    $(TEST_OBJECT_DIR)/%.o, \
    $(TEST_SOURCES))

# C modules required by the tests.
TEST_C_OBJECTS = \
    $(OBJECT_DIR)/common.o \
    $(OBJECT_DIR)/grid.o \
    $(OBJECT_DIR)/validation.o \
    $(OBJECT_DIR)/astar.o

TEST_EXECUTABLE = $(BUILD_DIR)/grid_astar_tests

# ----------------------------------------------------------
# Default target
# ----------------------------------------------------------

.PHONY: all

all: $(APP_NAME)

# ----------------------------------------------------------
# Application linking
# ----------------------------------------------------------

$(APP_NAME): $(APP_OBJECTS)
	$(CC) $(APP_OBJECTS) -o $@ $(LDLIBS)

# ----------------------------------------------------------
# Compile C source files separately
# ----------------------------------------------------------

$(OBJECT_DIR)/%.o: $(SOURCE_DIR)/%.c | $(OBJECT_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -MMD -MP -c $< -o $@

# ----------------------------------------------------------
# Compile Google Test C++ source files
# ----------------------------------------------------------

$(TEST_OBJECT_DIR)/%.o: $(TEST_DIR)/%.cpp | $(TEST_OBJECT_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

# ----------------------------------------------------------
# Build directories
# ----------------------------------------------------------

$(OBJECT_DIR):
	mkdir -p $(OBJECT_DIR)

$(TEST_OBJECT_DIR):
	mkdir -p $(TEST_OBJECT_DIR)

# ----------------------------------------------------------
# Build Google Test executable
# ----------------------------------------------------------

$(TEST_EXECUTABLE): $(TEST_OBJECTS) $(TEST_C_OBJECTS)
	$(CXX) $(TEST_OBJECTS) $(TEST_C_OBJECTS) \
	-o $@ $(GTEST_LIBS) $(LDLIBS)

# ----------------------------------------------------------
# Run application
# ----------------------------------------------------------

.PHONY: run

run: all
	./$(APP_NAME)

# ----------------------------------------------------------
# Execute Google Test cases
# ----------------------------------------------------------

.PHONY: test

test: $(TEST_EXECUTABLE)
	./$(TEST_EXECUTABLE)

# ----------------------------------------------------------
# Remove generated files
# ----------------------------------------------------------

.PHONY: clean

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(APP_NAME)
	rm -f output/planned_path.txt

# ----------------------------------------------------------
# Header dependency tracking
# ----------------------------------------------------------

-include $(APP_OBJECTS:.o=.d)
-include $(TEST_OBJECTS:.o=.d)
