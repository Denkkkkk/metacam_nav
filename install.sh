#!/bin/bash
catkin_make install -DCMAKE_INSTALL_PREFIX=output/metacam_nav/opt/skyland/metacam_nav -DCMAKE_BUILD_TYPE=Release

catkin_make -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release

