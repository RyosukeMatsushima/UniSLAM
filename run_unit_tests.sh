#!/bin/bash

# Function to run a test and check its exit code
run_test() {
    echo "Running test: $1"
    # create result directory if not exists
    # directory name is result/$1_test
    mkdir -p result/$1
    ./$1_test
    if [ $? -eq 0 ]; then
        echo "Test $1 passed!"
        return 0
    else
        echo "Test $1 failed."
        return 1
    fi
}

# List of test executables
tests=(
       "frame"
       "edges_space"
       "polygons_space"
       "edge"
       "edge_point"
       "cpp_copy"
       "line_3d"
       "edge_point_check"
       "discrete_angle_edge_intensity"
       "edge_point_finder"
       "frame_node"
       )

# Variable to keep track of overall test result
overall_result=0

cd build

# Loop through each test and run it
for test_executable in "${tests[@]}"; do
    run_test $test_executable
    result=$?
    if [ $result -ne 0 ]; then
        overall_result=1
    fi
done

cd ..

# Print overall result
if [ $overall_result -eq 0 ]; then
    echo "All tests passed!"
else
    echo "Some tests failed."
fi
