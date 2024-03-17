
cmake -S . -B build

cmake --build build

cd build

./frame_test
./edges_space_test
./polygons_space_test
./edge_test
./cpp_copy_test

cd ..

