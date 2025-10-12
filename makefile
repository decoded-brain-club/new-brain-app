# Basic Makefile for a simple C++ project

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2

# Target executable name
TARGET = main

# Source files and object files
SRC = $(wildcard *.cpp)
OBJ = $(SRC:.cpp=.o)

# Default rule
all: $(TARGET)

# Link object files into executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	rm -f $(OBJ) $(TARGET)

# Run program (optional shortcut)
run: $(TARGET)
	./$(TARGET)