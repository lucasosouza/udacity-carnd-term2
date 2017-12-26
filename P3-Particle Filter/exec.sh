#!/bin/bash
# Customized script to clean, build and run

# Remove the dedicated output directories
cd `dirname $0`

rm -rf build

# We're done!
echo Cleaned up the project!

# Go into the directory where this bash script is contained.
cd `dirname $0`

# Compile code.
mkdir -p build
cd build
cmake ..
make -j `nproc` $*

# Run particle filter
cd ./build
./particle_filter
