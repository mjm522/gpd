#!/usr/bin/bash

set -e

git clone https://github.com/pybind/pybind11.git gpd_py/pybind11

mkdir -p build

rm -rf build/* && cd build && cmake .. && sudo make install


