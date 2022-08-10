#!/bin/sh
VERFILE="../version.h"
if [ -d "CMakeFiles" ];then rm -rf CMakeFiles; fi
if [ -f "Makefile" ];then rm -f Makefile; fi
if [ -f "cmake_install.cmake" ];then rm -f cmake_install.cmake; fi
if [ -f "CMakeCache.txt" ];then rm -f CMakeCache.txt; fi
if [ -f "$VERFILE" ];then rm -f $VERFILE; fi

head=$(git rev-parse --verify HEAD 2>/dev/null)
gitver=$(echo $head | cut -c1-12)
echo "#define SDK_VERSION \"g${gitver}\"" > $VERFILE

cmake -DCMAKE_TOOLCHAIN_FILE="../../../../../tools/cmake_toolchain_files/armgcc.cmake" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=release  .
make -j 2>&1 | tee build_log.txt
