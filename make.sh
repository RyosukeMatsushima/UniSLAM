
cmake -S . -B build

cmake --build build

cd build

./frame_test
./edges_space_test
./polygons_space_test

cd ..

