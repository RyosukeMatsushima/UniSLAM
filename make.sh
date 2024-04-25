
cmake -S . -B build

cmake --build build

cd build

# create result directory if not exists
mkdir -p result
rm result/*

./frame_test
./edges_space_test
./polygons_space_test
./edge_test
./cpp_copy_test

mkdir -p result/edge_point_check
./edge_point_check_test

mkdir -p result/discrete_angle_edge_intensity
./discrete_angle_edge_intensity_test

mkdir -p result/edge_point_finder
./edge_point_finder_test

cd ..

