cmake_minimum_required(VERSION 3.0.0)

# Project name
project(OccupancyGridMapping)

# Find Python3 (Only if used)
find_package(Python3 3.13 REQUIRED COMPONENTS Interpreter Development NumPy)

# Manually specify NumPy include directory (Adjust path accordingly)
set(NUMPY_INCLUDE_DIR "C:/Python313/Lib/site-packages/numpy/_core/include")

# Include directories
include_directories(
    ${Python3_INCLUDE_DIRS}  # Python headers
    include  # Your local headers
    ${NUMPY_INCLUDE_DIR}  # NumPy headers
)

# Collect all source files
set(SOURCES 
    src/main.cpp
    src/map.cpp
    src/robot.cpp
    src/sensor.cpp
    src/visualization.cpp
)

# Create executable
add_executable(output ${SOURCES})

# Link libraries (if using Python)
target_link_libraries(output
    ${Python3_LIBRARIES}
    ${Python3_NumPy_LIBRARIES}
    C:/Python313/python313.dll
)
