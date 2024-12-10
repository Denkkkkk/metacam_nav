#!/bin/bash

#install yaml-cpp
cd quickStart/yaml-cpp
mkdir build
cd build
cmake ..
make -j4
sudo make installmkdir build
cd build
cmake ..
make -j4
sudo make install