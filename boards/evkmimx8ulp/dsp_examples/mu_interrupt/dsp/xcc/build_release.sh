#!/bin/sh
cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../../tools/cmake_toolchain_files/xcc.cmake" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=release .
make -j4
